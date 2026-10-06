"""Pixel/table-plane conversion for the fixed Bài 03 overhead camera."""

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence, Tuple

from ament_index_python.packages import get_package_share_directory
import yaml


@dataclass(frozen=True)
class CameraCalibration:
    """Validated pinhole and planar calibration parameters."""

    width: int
    height: int
    frame_id: str
    optical_frame: str
    table_z: float
    homography: Tuple[Tuple[float, float, float], ...]


def _matrix3(raw: object, field: str) -> Tuple[Tuple[float, float, float], ...]:
    if not isinstance(raw, list) or len(raw) != 3:
        raise ValueError(f'{field} must contain three rows')
    rows = []
    for row in raw:
        if not isinstance(row, list) or len(row) != 3:
            raise ValueError(f'{field} rows must contain three values')
        rows.append(tuple(float(value) for value in row))
    return tuple(rows)


def load_camera_calibration(path: str) -> CameraCalibration:
    """Load the fixed-camera calibration from YAML."""
    with Path(path).open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or not isinstance(document.get('camera'), dict):
        raise ValueError("calibration must contain a top-level 'camera' mapping")
    camera = document['camera']
    width = int(camera.get('image_width', 0))
    height = int(camera.get('image_height', 0))
    frame_id = camera.get('frame_id')
    optical_frame = camera.get('optical_frame')
    if width <= 0 or height <= 0:
        raise ValueError('camera image dimensions must be positive')
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError('camera.frame_id must be a non-empty string')
    if not isinstance(optical_frame, str) or not optical_frame:
        raise ValueError('camera.optical_frame must be a non-empty string')
    return CameraCalibration(
        width=width,
        height=height,
        frame_id=frame_id,
        optical_frame=optical_frame,
        table_z=float(camera['table_z']),
        homography=_matrix3(
            camera.get('image_to_table_homography'),
            'camera.image_to_table_homography',
        ),
    )


def pixel_to_table(
    calibration: CameraCalibration, u: float, v: float
) -> Tuple[float, float, float]:
    """Project pixel ``(u, v)`` onto the calibrated tabletop plane."""
    h = calibration.homography
    scale = h[2][0] * u + h[2][1] * v + h[2][2]
    if abs(scale) < 1.0e-12:
        raise ValueError('pixel projects to an invalid point at infinity')
    x = (h[0][0] * u + h[0][1] * v + h[0][2]) / scale
    y = (h[1][0] * u + h[1][1] * v + h[1][2]) / scale
    return x, y, calibration.table_z


def table_to_pixel(
    calibration: CameraCalibration, x: float, y: float
) -> Tuple[float, float]:
    """Invert the axis-aligned overhead homography for calibration tests."""
    h = calibration.homography
    if abs(h[0][1]) < 1.0e-12 or abs(h[1][0]) < 1.0e-12:
        raise ValueError('calibration homography is not invertible in x/y')
    v = (x - h[0][2]) / h[0][1]
    u = (y - h[1][2]) / h[1][0]
    return u, v


def _default_calibration_path() -> str:
    return str(
        Path(get_package_share_directory('ur3_llm_control'))
        / 'config'
        / 'camera_calibration.yaml'
    )


def main(argv: Sequence[str] = None) -> int:
    parser = argparse.ArgumentParser(
        description='Convert an overhead-camera pixel to the table coordinate.'
    )
    parser.add_argument('u', type=float, help='pixel column')
    parser.add_argument('v', type=float, help='pixel row')
    parser.add_argument('--config', default=_default_calibration_path())
    args = parser.parse_args(argv)
    calibration = load_camera_calibration(args.config)
    x, y, z = pixel_to_table(calibration, args.u, args.v)
    print(f'x={x:.6f} y={y:.6f} z={z:.6f}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
