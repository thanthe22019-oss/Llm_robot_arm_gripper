"""Pure image-processing and SceneState logic for the fixed workcell camera."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict, List, Mapping, Tuple

import cv2
import numpy as np
import yaml

from ur3_llm_control.camera_geometry import (
    CameraCalibration,
    pixel_to_table,
    table_to_pixel,
)


@dataclass(frozen=True)
class HsvRange:
    """Inclusive HSV threshold bounds used by OpenCV."""

    lower: Tuple[int, int, int]
    upper: Tuple[int, int, int]


@dataclass(frozen=True)
class DetectorConfig:
    """Validated color detector settings."""

    minimum_area_px: float
    maximum_area_px: float
    expected_area_px: float
    minimum_aspect_ratio: float
    maximum_aspect_ratio: float
    morphology_kernel_px: int
    table_margin_m: float
    publish_rate_hz: float
    stale_after_sec: float
    colors: Mapping[str, Tuple[HsvRange, ...]]


@dataclass(frozen=True)
class Region:
    """Known fixed target geometry on the table."""

    name: str
    x: float
    y: float
    size_x: float
    size_y: float

    def contains(self, x: float, y: float) -> bool:
        return (
            abs(x - self.x) <= self.size_x / 2.0
            and abs(y - self.y) <= self.size_y / 2.0
        )


@dataclass(frozen=True)
class WorkspaceGeometry:
    """Fixed geometry; dynamic object poses are deliberately absent."""

    frame_id: str
    table: Region
    object_heights: Mapping[str, float]
    zones: Mapping[str, Region]
    temporary_slots: Mapping[str, Region]


@dataclass(frozen=True)
class Detection:
    """One color detection projected into the robot base frame."""

    name: str
    u: float
    v: float
    x: float
    y: float
    z: float
    area_px: float
    confidence: float
    bbox: Tuple[int, int, int, int]


def _triplet(raw: object, field: str) -> Tuple[int, int, int]:
    if not isinstance(raw, list) or len(raw) != 3:
        raise ValueError(f'{field} must contain three integer values')
    values = tuple(int(value) for value in raw)
    if any(value < 0 or value > 255 for value in values):
        raise ValueError(f'{field} entries must be in [0, 255]')
    return values


def load_detector_config(path: str) -> DetectorConfig:
    """Load and validate HSV and contour filtering settings."""
    with Path(path).open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or not isinstance(
        document.get('perception'), dict
    ):
        raise ValueError("detector config must contain a 'perception' mapping")
    raw = document['perception']
    raw_colors = raw.get('colors')
    if not isinstance(raw_colors, dict) or not raw_colors:
        raise ValueError('perception.colors must be a non-empty mapping')
    colors: Dict[str, Tuple[HsvRange, ...]] = {}
    for name, entries in raw_colors.items():
        if not isinstance(entries, list) or not entries:
            raise ValueError(f'perception.colors.{name} must be a non-empty list')
        ranges = []
        for index, entry in enumerate(entries):
            if not isinstance(entry, dict):
                raise ValueError(f'color range {name}[{index}] must be a mapping')
            ranges.append(
                HsvRange(
                    _triplet(entry.get('lower'), f'{name}[{index}].lower'),
                    _triplet(entry.get('upper'), f'{name}[{index}].upper'),
                )
            )
        colors[str(name)] = tuple(ranges)

    config = DetectorConfig(
        minimum_area_px=float(raw['minimum_area_px']),
        maximum_area_px=float(raw['maximum_area_px']),
        expected_area_px=float(raw['expected_area_px']),
        minimum_aspect_ratio=float(raw['minimum_aspect_ratio']),
        maximum_aspect_ratio=float(raw['maximum_aspect_ratio']),
        morphology_kernel_px=int(raw['morphology_kernel_px']),
        table_margin_m=float(raw['table_margin_m']),
        publish_rate_hz=float(raw['publish_rate_hz']),
        stale_after_sec=float(raw['stale_after_sec']),
        colors=colors,
    )
    if not 0.0 < config.minimum_area_px < config.maximum_area_px:
        raise ValueError('contour area limits are invalid')
    if not config.minimum_area_px <= config.expected_area_px <= config.maximum_area_px:
        raise ValueError('expected_area_px must lie inside the area limits')
    if config.morphology_kernel_px <= 0:
        raise ValueError('morphology_kernel_px must be positive')
    if config.publish_rate_hz <= 0.0 or config.stale_after_sec <= 0.0:
        raise ValueError('publish rate and stale timeout must be positive')
    return config


def _region(name: str, raw: object) -> Region:
    if not isinstance(raw, dict):
        raise ValueError(f"scene region '{name}' must be a mapping")
    pose = raw.get('pose')
    size = raw.get('size')
    if not isinstance(pose, list) or len(pose) != 6:
        raise ValueError(f'{name}.pose must contain six values')
    if not isinstance(size, list) or len(size) != 3:
        raise ValueError(f'{name}.size must contain three values')
    return Region(name, float(pose[0]), float(pose[1]), float(size[0]), float(size[1]))


def load_workspace_geometry(path: str) -> WorkspaceGeometry:
    """Load fixed regions and dimensions, never dynamic object poses."""
    with Path(path).open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or not isinstance(document.get('scene'), dict):
        raise ValueError("scene file must contain a top-level 'scene' mapping")
    scene = document['scene']
    objects = scene.get('objects')
    zones = scene.get('zones')
    slots = scene.get('temporary_slots')
    if not all(isinstance(item, dict) for item in (objects, zones, slots)):
        raise ValueError('scene objects, zones and temporary_slots must be mappings')
    heights = {}
    for name, entry in objects.items():
        if not isinstance(entry, dict) or not isinstance(entry.get('size'), list):
            raise ValueError(f'{name}.size is required')
        heights[str(name)] = float(entry['size'][2])
    return WorkspaceGeometry(
        frame_id=str(scene.get('frame_id', 'base_link')),
        table=_region('table', scene.get('table')),
        object_heights=heights,
        zones={name: _region(name, value) for name, value in zones.items()},
        temporary_slots={name: _region(name, value) for name, value in slots.items()},
    )


def _inside_table(
    workspace: WorkspaceGeometry, x: float, y: float, margin: float
) -> bool:
    table = workspace.table
    return (
        abs(x - table.x) <= table.size_x / 2.0 + margin
        and abs(y - table.y) <= table.size_y / 2.0 + margin
    )


def detect_blocks(
    image_bgr: np.ndarray,
    calibration: CameraCalibration,
    config: DetectorConfig,
    workspace: WorkspaceGeometry,
) -> Dict[str, Detection]:
    """Detect one best contour per configured block color."""
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError('image_bgr must be an HxWx3 image')
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    kernel = np.ones(
        (config.morphology_kernel_px, config.morphology_kernel_px), np.uint8
    )
    detections: Dict[str, Detection] = {}
    for name, ranges in config.colors.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for hsv_range in ranges:
            mask = cv2.bitwise_or(
                mask,
                cv2.inRange(
                    hsv,
                    np.asarray(hsv_range.lower, dtype=np.uint8),
                    np.asarray(hsv_range.upper, dtype=np.uint8),
                ),
            )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if not config.minimum_area_px <= area <= config.maximum_area_px:
                continue
            bx, by, width, height = cv2.boundingRect(contour)
            if height <= 0:
                continue
            aspect = width / float(height)
            if not config.minimum_aspect_ratio <= aspect <= config.maximum_aspect_ratio:
                continue
            moments = cv2.moments(contour)
            if abs(moments['m00']) < 1.0e-9:
                continue
            u = moments['m10'] / moments['m00']
            v = moments['m01'] / moments['m00']
            x, y, table_z = pixel_to_table(calibration, u, v)
            if not _inside_table(workspace, x, y, config.table_margin_m):
                continue
            area_score = math.exp(
                -abs(math.log(max(area, 1.0) / config.expected_area_px))
            )
            shape_score = min(aspect, 1.0 / aspect)
            confidence = min(0.99, 0.55 * area_score + 0.45 * shape_score)
            z = table_z + workspace.object_heights.get(name, 0.04) / 2.0
            detection = Detection(
                name=name,
                u=float(u),
                v=float(v),
                x=float(x),
                y=float(y),
                z=float(z),
                area_px=area,
                confidence=float(confidence),
                bbox=(int(bx), int(by), int(width), int(height)),
            )
            distance = abs(math.log(max(area, 1.0) / config.expected_area_px))
            candidates.append((distance, -confidence, detection))
        if candidates:
            candidates.sort(key=lambda item: (item[0], item[1]))
            detections[name] = candidates[0][2]
    return detections


def _location(detection: Detection, workspace: WorkspaceGeometry) -> str:
    for region in workspace.zones.values():
        if region.contains(detection.x, detection.y):
            return region.name
    for region in workspace.temporary_slots.values():
        if region.contains(detection.x, detection.y):
            return region.name
    return 'table'


def build_scene_state(
    detections: Mapping[str, Detection],
    workspace: WorkspaceGeometry,
    *,
    stamp: float,
    version: int,
) -> dict:
    """Build a JSON-serializable snapshot and infer zone occupancy."""
    issues: List[str] = []
    objects = {}
    occupancy: Dict[str, List[str]] = {
        name: []
        for name in list(workspace.zones) + list(workspace.temporary_slots)
    }
    for name in sorted(workspace.object_heights):
        detection = detections.get(name)
        if detection is None:
            objects[name] = {'visible': False, 'location': 'unknown'}
            issues.append(f'{name} is not visible')
            continue
        location = _location(detection, workspace)
        if location in occupancy:
            occupancy[location].append(name)
        objects[name] = {
            'visible': True,
            'location': location,
            'x': round(detection.x, 6),
            'y': round(detection.y, 6),
            'z': round(detection.z, 6),
            'confidence': round(detection.confidence, 4),
            'pixel': {'u': round(detection.u, 2), 'v': round(detection.v, 2)},
        }

    def region_states(regions: Mapping[str, Region]) -> dict:
        result = {}
        for name in sorted(regions):
            occupants = sorted(occupancy[name])
            if len(occupants) > 1:
                issues.append(f'{name} contains multiple detected objects')
            result[name] = {
                'occupied': bool(occupants),
                'object': occupants[0] if len(occupants) == 1 else None,
            }
        return result

    zones = region_states(workspace.zones)
    temporary_slots = region_states(workspace.temporary_slots)
    return {
        'version': int(version),
        'stamp': round(float(stamp), 6),
        'frame_id': workspace.frame_id,
        'source': 'camera',
        'valid': not issues,
        'objects': objects,
        'zones': zones,
        'temporary_slots': temporary_slots,
        'issues': issues,
    }


def state_signature(state: Mapping[str, object]) -> tuple:
    """Return a stable signature used to increment scene versions on change."""
    objects = state.get('objects', {})
    return tuple(
        (
            name,
            value.get('visible'),
            value.get('location'),
            round(float(value.get('x', 0.0)), 2),
            round(float(value.get('y', 0.0)), 2),
        )
        for name, value in sorted(objects.items())
    )


def draw_overlay(
    image_bgr: np.ndarray,
    detections: Mapping[str, Detection],
    state: Mapping[str, object],
    calibration: CameraCalibration,
    workspace: WorkspaceGeometry,
) -> np.ndarray:
    """Draw fixed regions, detections and current occupancy on an image."""
    overlay = image_bgr.copy()
    for region in list(workspace.zones.values()) + list(
        workspace.temporary_slots.values()
    ):
        lower = (region.x - region.size_x / 2.0, region.y - region.size_y / 2.0)
        upper = (region.x + region.size_x / 2.0, region.y + region.size_y / 2.0)
        u0, v0 = table_to_pixel(calibration, *lower)
        u1, v1 = table_to_pixel(calibration, *upper)
        first = (int(round(min(u0, u1))), int(round(min(v0, v1))))
        second = (int(round(max(u0, u1))), int(round(max(v0, v1))))
        cv2.rectangle(overlay, first, second, (255, 255, 255), 1)
        cv2.putText(
            overlay,
            region.name,
            (first[0], max(12, first[1] - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    for detection in detections.values():
        bx, by, width, height = detection.bbox
        cv2.rectangle(overlay, (bx, by), (bx + width, by + height), (0, 255, 255), 2)
        label = f'{detection.name} {detection.confidence:.2f}'
        cv2.putText(
            overlay,
            label,
            (bx, max(14, by - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
    status = 'VALID' if state.get('valid') else 'INVALID'
    cv2.putText(
        overlay,
        f'SceneState v{state.get("version", 0)} {status}',
        (12, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (0, 220, 0) if state.get('valid') else (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    return overlay
