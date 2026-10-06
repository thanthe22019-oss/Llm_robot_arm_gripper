"""ROS 2 node that turns overhead RGB images into validated SceneState JSON."""

import json
from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ur3_llm_control.camera_geometry import load_camera_calibration
from ur3_perception.perception_core import (
    build_scene_state,
    detect_blocks,
    draw_overlay,
    load_detector_config,
    load_workspace_geometry,
    state_signature,
)


def _share(package: str, *parts: str) -> str:
    return str(Path(get_package_share_directory(package)).joinpath(*parts))


class BlockDetectorNode(Node):
    """Detect five color-coded cubes and publish the observed workcell state."""

    def __init__(self) -> None:
        super().__init__('block_detector')
        self.declare_parameter(
            'config_file', _share('ur3_perception', 'config', 'perception.yaml')
        )
        self.declare_parameter(
            'calibration_file',
            _share('ur3_llm_control', 'config', 'camera_calibration.yaml'),
        )
        self.declare_parameter(
            'scene_file',
            _share('ur3_llm_control', 'config', 'scene_clear.yaml'),
        )
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('state_topic', '/scene_state')
        self.declare_parameter('overlay_topic', '/camera/detections_image')

        self._config = load_detector_config(
            self.get_parameter('config_file').get_parameter_value().string_value
        )
        self._calibration = load_camera_calibration(
            self.get_parameter('calibration_file').get_parameter_value().string_value
        )
        self._workspace = load_workspace_geometry(
            self.get_parameter('scene_file').get_parameter_value().string_value
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_publisher = self.create_publisher(
            String,
            self.get_parameter('state_topic').get_parameter_value().string_value,
            state_qos,
        )
        self._overlay_publisher = self.create_publisher(
            Image,
            self.get_parameter('overlay_topic').get_parameter_value().string_value,
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            Image,
            self.get_parameter('image_topic').get_parameter_value().string_value,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self._service = self.create_service(
            Trigger, '/get_scene_state', self._get_scene_state
        )
        self._latest_state: Optional[dict] = None
        self._latest_json = ''
        self._latest_receipt_ns = 0
        self._last_publish_ns = 0
        self._signature = None
        self._version = 0
        self._reported_valid_version = 0
        self.get_logger().info(
            'Camera perception ready: /scene_state and /get_scene_state'
        )

    @staticmethod
    def _to_bgr(message: Image) -> np.ndarray:
        channels = {
            'bgr8': (3, None),
            'rgb8': (3, cv2.COLOR_RGB2BGR),
            'bgra8': (4, cv2.COLOR_BGRA2BGR),
            'rgba8': (4, cv2.COLOR_RGBA2BGR),
        }
        if message.encoding not in channels:
            raise ValueError(f'unsupported image encoding: {message.encoding}')
        count, conversion = channels[message.encoding]
        row_bytes = int(message.step)
        raw = np.frombuffer(message.data, dtype=np.uint8)
        expected = int(message.height) * row_bytes
        if raw.size < expected:
            raise ValueError('image data is shorter than height * step')
        rows = raw[:expected].reshape((int(message.height), row_bytes))
        packed = rows[:, : int(message.width) * count]
        image = packed.reshape((int(message.height), int(message.width), count))
        if conversion is not None:
            image = cv2.cvtColor(image, conversion)
        return np.ascontiguousarray(image)

    @staticmethod
    def _overlay_message(source: Image, image_bgr: np.ndarray) -> Image:
        output = Image()
        output.header = source.header
        output.height = int(image_bgr.shape[0])
        output.width = int(image_bgr.shape[1])
        output.encoding = 'bgr8'
        output.is_bigendian = False
        output.step = output.width * 3
        output.data = image_bgr.tobytes()
        return output

    def _image_callback(self, message: Image) -> None:
        now_ns = self.get_clock().now().nanoseconds
        minimum_period_ns = int(1.0e9 / self._config.publish_rate_hz)
        if now_ns - self._last_publish_ns < minimum_period_ns:
            return
        try:
            image = self._to_bgr(message)
            detections = detect_blocks(
                image, self._calibration, self._config, self._workspace
            )
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1.0e9
            provisional = build_scene_state(
                detections,
                self._workspace,
                stamp=stamp,
                version=self._version,
            )
            signature = state_signature(provisional)
            if signature != self._signature:
                self._version += 1
                self._signature = signature
            state = build_scene_state(
                detections,
                self._workspace,
                stamp=stamp,
                version=self._version,
            )
            serialized = json.dumps(state, sort_keys=True, separators=(',', ':'))
            self._state_publisher.publish(String(data=serialized))
            overlay = draw_overlay(
                image, detections, state, self._calibration, self._workspace
            )
            self._overlay_publisher.publish(self._overlay_message(message, overlay))
            self._latest_state = state
            self._latest_json = serialized
            self._latest_receipt_ns = now_ns
            self._last_publish_ns = now_ns
            if state['valid'] and state['version'] != self._reported_valid_version:
                self._reported_valid_version = state['version']
                self.get_logger().info(
                    f'Camera SceneState v{state["version"]} is valid for 5 blocks'
                )
        except (ValueError, cv2.error) as error:
            self.get_logger().error(f'Perception frame rejected: {error}')

    def _get_scene_state(self, request, response):
        del request
        if self._latest_state is None:
            response.success = False
            response.message = 'No camera SceneState has been received yet'
            return response
        age = (self.get_clock().now().nanoseconds - self._latest_receipt_ns) / 1.0e9
        response.success = bool(
            self._latest_state.get('valid') and age <= self._config.stale_after_sec
        )
        response.message = self._latest_json
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlockDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
