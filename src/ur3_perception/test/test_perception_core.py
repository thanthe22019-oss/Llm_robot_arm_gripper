"""Tests for camera-only block detection and SceneState inference."""

from pathlib import Path

import cv2
import numpy as np
import pytest

from ur3_llm_control.camera_geometry import (
    load_camera_calibration,
    table_to_pixel,
)
from ur3_perception.perception_core import (
    build_scene_state,
    detect_blocks,
    load_detector_config,
    load_workspace_geometry,
)


ROOT = Path(__file__).parents[2]
CONTROL_CONFIG = ROOT / 'ur3_llm_control' / 'config'
PERCEPTION_CONFIG = Path(__file__).parents[1] / 'config' / 'perception.yaml'


@pytest.fixture
def calibration():
    return load_camera_calibration(str(CONTROL_CONFIG / 'camera_calibration.yaml'))


@pytest.fixture
def detector_config():
    return load_detector_config(str(PERCEPTION_CONFIG))


@pytest.fixture
def clear_workspace():
    return load_workspace_geometry(str(CONTROL_CONFIG / 'scene_clear.yaml'))


def _synthetic_image(calibration, positions):
    image = np.zeros((calibration.height, calibration.width, 3), dtype=np.uint8)
    colors = {
        'red_cube': (0, 0, 230),
        'yellow_cube': (0, 220, 240),
        'blue_cube': (230, 50, 10),
        'green_cube': (30, 190, 20),
        'purple_cube': (190, 30, 150),
    }
    for name, (x, y) in positions.items():
        u, v = table_to_pixel(calibration, x, y)
        center = (int(round(u)), int(round(v)))
        cv2.rectangle(
            image,
            (center[0] - 18, center[1] - 18),
            (center[0] + 18, center[1] + 18),
            colors[name],
            -1,
        )
    # A large yellow robot-like distractor must not replace the cube.
    cv2.rectangle(image, (230, 175), (320, 230), colors['yellow_cube'], -1)
    return image


def test_detector_finds_five_blocks_without_using_spawn_poses(
    calibration, detector_config, clear_workspace
):
    positions = {
        'red_cube': (0.20, 0.18),
        'yellow_cube': (0.20, 0.06),
        'blue_cube': (0.20, -0.06),
        'green_cube': (0.20, -0.18),
        'purple_cube': (0.44, -0.18),
    }
    detections = detect_blocks(
        _synthetic_image(calibration, positions),
        calibration,
        detector_config,
        clear_workspace,
    )
    assert set(detections) == set(positions)
    for name, (expected_x, expected_y) in positions.items():
        assert detections[name].x == pytest.approx(expected_x, abs=0.01)
        assert detections[name].y == pytest.approx(expected_y, abs=0.01)
        assert detections[name].z == pytest.approx(0.18)


def test_clear_scene_reports_all_zones_and_temp_slot_empty(
    calibration, detector_config, clear_workspace
):
    positions = {
        'red_cube': (0.20, 0.18),
        'yellow_cube': (0.20, 0.06),
        'blue_cube': (0.20, -0.06),
        'green_cube': (0.20, -0.18),
        'purple_cube': (0.44, -0.18),
    }
    detections = detect_blocks(
        _synthetic_image(calibration, positions),
        calibration,
        detector_config,
        clear_workspace,
    )
    state = build_scene_state(detections, clear_workspace, stamp=1.5, version=1)
    assert state['valid'] is True
    assert all(not zone['occupied'] for zone in state['zones'].values())
    assert state['temporary_slots']['temp_1']['occupied'] is False
    assert all(item['location'] == 'table' for item in state['objects'].values())


def test_camera_position_in_zone_b_sets_blue_occupancy(
    calibration, detector_config, clear_workspace
):
    positions = {
        'red_cube': (0.20, 0.18),
        'yellow_cube': (0.20, 0.06),
        'blue_cube': (0.32, 0.06),
        'green_cube': (0.20, -0.18),
        'purple_cube': (0.44, -0.18),
    }
    detections = detect_blocks(
        _synthetic_image(calibration, positions),
        calibration,
        detector_config,
        clear_workspace,
    )
    state = build_scene_state(detections, clear_workspace, stamp=2.0, version=2)
    assert state['valid'] is True
    assert state['objects']['blue_cube']['location'] == 'zone_b'
    assert state['zones']['zone_b'] == {
        'occupied': True,
        'object': 'blue_cube',
    }


def test_missing_detection_invalidates_scene_state(clear_workspace):
    state = build_scene_state({}, clear_workspace, stamp=3.0, version=1)
    assert state['valid'] is False
    assert len(state['issues']) == 5
    assert state['objects']['red_cube']['visible'] is False


def test_workspace_geometry_does_not_expose_dynamic_object_poses(clear_workspace):
    assert not hasattr(clear_workspace, 'objects')
    assert set(clear_workspace.object_heights) == {
        'red_cube',
        'yellow_cube',
        'blue_cube',
        'green_cube',
        'purple_cube',
    }
