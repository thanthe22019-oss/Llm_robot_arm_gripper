"""Calibration checks for the fixed overhead camera."""

from pathlib import Path

import pytest

from ur3_llm_control.camera_geometry import (
    load_camera_calibration,
    pixel_to_table,
    table_to_pixel,
)
from ur3_llm_control.scene_config import load_scene


CONFIG_DIR = Path(__file__).parents[1] / 'config'


@pytest.fixture
def calibration():
    return load_camera_calibration(str(CONFIG_DIR / 'camera_calibration.yaml'))


def test_table_and_all_targets_are_inside_camera_image(calibration):
    scene = load_scene(str(CONFIG_DIR / 'scene_clear.yaml'))
    entries = [scene.table]
    entries.extend(scene.objects.values())
    entries.extend(scene.zones.values())
    entries.extend(scene.temporary_slots.values())
    for entry in entries:
        u, v = table_to_pixel(calibration, entry.pose[0], entry.pose[1])
        assert 0.0 <= u < calibration.width
        assert 0.0 <= v < calibration.height


def test_pixel_round_trip_error_is_below_one_centimetre(calibration):
    scene = load_scene(str(CONFIG_DIR / 'scene_clear.yaml'))
    for cube in scene.objects.values():
        u, v = table_to_pixel(calibration, cube.pose[0], cube.pose[1])
        x, y, z = pixel_to_table(calibration, u, v)
        assert abs(x - cube.pose[0]) < 0.01
        assert abs(y - cube.pose[1]) < 0.01
        assert z == pytest.approx(0.16)


def test_red_cube_expected_pixel_maps_to_ground_truth(calibration):
    x, y, _ = pixel_to_table(calibration, 185.184, 322.388)
    assert x == pytest.approx(0.20, abs=0.001)
    assert y == pytest.approx(0.18, abs=0.001)
