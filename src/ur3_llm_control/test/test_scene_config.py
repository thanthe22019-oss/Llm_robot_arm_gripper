"""Tests for the YAML-backed Gazebo and MoveIt scene scenarios."""

from pathlib import Path
from xml.etree import ElementTree

import pytest

from ur3_llm_control.scene_config import box_sdf, gazebo_boxes, load_scene


CONFIG_DIR = Path(__file__).parents[1] / 'config'
CLEAR_SCENE_FILE = CONFIG_DIR / 'scene_clear.yaml'
BLOCKED_SCENE_FILE = CONFIG_DIR / 'scene_blocked.yaml'
COMPATIBILITY_SCENE_FILE = CONFIG_DIR / 'scene.yaml'
EXPECTED_OBJECTS = {
    'red_cube',
    'yellow_cube',
    'blue_cube',
    'green_cube',
    'purple_cube',
}
EXPECTED_ZONES = {'zone_a', 'zone_b', 'zone_c'}


@pytest.fixture
def scene():
    """Load the default clear scenario."""
    return load_scene(str(CLEAR_SCENE_FILE))


@pytest.fixture
def blocked_scene():
    """Load the scenario where blue_cube occupies zone_b."""
    return load_scene(str(BLOCKED_SCENE_FILE))


def _footprints_overlap(first, second):
    return (
        abs(first.pose[0] - second.pose[0])
        < (first.size[0] + second.size[0]) / 2.0
        and abs(first.pose[1] - second.pose[1])
        < (first.size[1] + second.size[1]) / 2.0
    )


def test_required_entities_are_present(scene):
    """Bài 03 requires five cubes, three zones, and a temporary slot."""
    assert set(scene.objects) == EXPECTED_OBJECTS
    assert set(scene.zones) == EXPECTED_ZONES
    assert set(scene.temporary_slots) == {'temp_1'}
    assert all(spec.collision for spec in scene.objects.values())
    assert all(not spec.collision for spec in scene.zones.values())
    assert all(not spec.collision for spec in scene.temporary_slots.values())


def test_compatibility_scene_is_the_clear_scenario(scene):
    """Keep scene.yaml compatible with commands retained from Bài 02."""
    assert load_scene(str(COMPATIBILITY_SCENE_FILE)) == scene


def test_gazebo_sdf_uses_yaml_pose_and_size(scene):
    """Every generated model must preserve its configured pose and size."""
    for spec in gazebo_boxes(scene):
        model = ElementTree.fromstring(box_sdf(spec)).find('model')
        assert model is not None
        assert model.attrib['name'] == spec.name
        pose = [float(value) for value in model.findtext('pose').split()]
        size = [
            float(value)
            for value in model.find('link/visual/geometry/box/size').text.split()
        ]
        assert pose == pytest.approx(spec.pose)
        assert size == pytest.approx(spec.size)


def test_dynamic_cubes_expose_physical_grasp_joint_topics(scene):
    """Every movable cube can be constrained after the fingers close."""
    for cube in scene.objects.values():
        model = ElementTree.fromstring(box_sdf(cube)).find('model')
        plugin = model.find(
            "plugin[@name='ignition::gazebo::systems::DetachableJoint']"
        )
        assert plugin is not None
        assert plugin.attrib['filename'] == (
            'ignition-gazebo-detachable-joint-system'
        )
        assert plugin.findtext('parent_link') == 'body'
        assert plugin.findtext('child_model') == 'ur'
        # Fixed gripper links are collapsed into wrist_3_link by URDF-to-SDF.
        # Lock to that rigid central link. Locking to one moving fingertip
        # makes the cube follow only one side of the mechanism and diverge
        # from MoveIt's attachment to the fixed TCP.
        assert plugin.findtext('child_link') == 'wrist_3_link'
        assert plugin.findtext('attach_topic') == (
            f'/gripper/{cube.name}/attach'
        )
        assert plugin.findtext('detach_topic') == (
            f'/gripper/{cube.name}/detach'
        )

    table_model = ElementTree.fromstring(box_sdf(scene.table)).find('model')
    assert table_model.find('plugin') is None


@pytest.mark.parametrize('filename', ['scene_clear.yaml', 'scene_blocked.yaml'])
def test_cubes_and_target_areas_rest_on_table_surface(filename):
    """Objects and target markers must share the physical table surface."""
    configured_scene = load_scene(str(CONFIG_DIR / filename))
    table_surface = (
        configured_scene.table.pose[2] + configured_scene.table.size[2] / 2.0
    )
    for cube in configured_scene.objects.values():
        cube_bottom = cube.pose[2] - cube.size[2] / 2.0
        assert cube_bottom == pytest.approx(table_surface)
    target_areas = list(configured_scene.zones.values())
    target_areas.extend(configured_scene.temporary_slots.values())
    for target in target_areas:
        target_top = target.pose[2] + target.size[2] / 2.0
        assert target_top == pytest.approx(table_surface)


def test_every_entity_stays_inside_table(scene):
    """All five cubes and four target areas must fit on the tabletop."""
    table = scene.table
    entities = list(scene.objects.values()) + list(scene.zones.values())
    entities.extend(scene.temporary_slots.values())
    for entity in entities:
        assert abs(entity.pose[0] - table.pose[0]) + entity.size[0] / 2.0 <= (
            table.size[0] / 2.0
        )
        assert abs(entity.pose[1] - table.pose[1]) + entity.size[1] / 2.0 <= (
            table.size[1] / 2.0
        )



def test_target_zone_visuals_do_not_overlap_cube_hsv_colours(scene):
    """Zone fills stay neutral so same-colour cubes remain camera-visible."""
    for zone in scene.zones.values():
        red, green, blue, _ = zone.color
        assert max(red, green, blue) - min(red, green, blue) <= 0.02

def test_target_zones_form_an_even_row(scene):
    """The three visual targets retain the tested Bài 02 layout."""
    zone_a = scene.zones['zone_a'].pose
    zone_b = scene.zones['zone_b'].pose
    zone_c = scene.zones['zone_c'].pose
    assert zone_a[0] == pytest.approx(zone_b[0])
    assert zone_b[0] == pytest.approx(zone_c[0])
    assert zone_a[1] - zone_b[1] == pytest.approx(0.12)
    assert zone_b[1] - zone_c[1] == pytest.approx(0.12)


def test_original_cubes_keep_their_matching_zone_alignment(scene):
    """Do not shift the three known cube/zone pairs while adding two cubes."""
    pairs = (
        ('red_cube', 'zone_a'),
        ('yellow_cube', 'zone_b'),
        ('blue_cube', 'zone_c'),
    )
    row_x = None
    separation = None
    for cube_name, zone_name in pairs:
        cube = scene.objects[cube_name].pose
        zone = scene.zones[zone_name].pose
        if row_x is None:
            row_x = cube[0]
            separation = zone[0] - cube[0]
        assert cube[0] == pytest.approx(row_x)
        assert cube[1] == pytest.approx(zone[1])
        assert zone[0] - cube[0] == pytest.approx(separation)
        assert separation > 0.0


def test_clear_scenario_has_no_occupied_target_area(scene):
    """No cube may overlap a zone or temp_1 in the clear scenario."""
    target_areas = list(scene.zones.values())
    target_areas.extend(scene.temporary_slots.values())
    for cube in scene.objects.values():
        for target in target_areas:
            assert not _footprints_overlap(cube, target), (
                f'{cube.name} unexpectedly overlaps {target.name}'
            )


def test_blocked_scenario_places_only_blue_cube_in_zone_b(scene, blocked_scene):
    """The demo obstruction is isolated to blue_cube occupying zone_b."""
    assert blocked_scene.floor == scene.floor
    assert blocked_scene.table == scene.table
    assert blocked_scene.zones == scene.zones
    assert blocked_scene.temporary_slots == scene.temporary_slots

    for object_name in EXPECTED_OBJECTS - {'blue_cube'}:
        assert blocked_scene.objects[object_name] == scene.objects[object_name]

    blue_cube = blocked_scene.objects['blue_cube']
    zone_b = blocked_scene.zones['zone_b']
    assert blue_cube.pose[:2] == pytest.approx(zone_b.pose[:2])
    assert _footprints_overlap(blue_cube, zone_b)
    assert not _footprints_overlap(
        blocked_scene.objects['red_cube'], zone_b
    )


def test_temporary_slot_does_not_overlap_a_zone(scene):
    """temp_1 must be physically separate from all destination zones."""
    temp_1 = scene.temporary_slots['temp_1']
    assert all(not _footprints_overlap(temp_1, zone) for zone in scene.zones.values())
