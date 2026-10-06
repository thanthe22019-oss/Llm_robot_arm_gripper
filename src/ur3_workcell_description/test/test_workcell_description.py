"""Structural checks for the composed UR3e and Robotiq 2F-85 model."""

from pathlib import Path
import subprocess
from xml.etree import ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent


def _expand_xacro(path, *arguments):
    command = ['xacro', str(path), *arguments]
    return subprocess.run(command, check=True, capture_output=True, text=True).stdout


def test_urdf_contains_arm_gripper_tcp_and_two_control_systems():
    xml = _expand_xacro(
        PACKAGE_ROOT / 'urdf' / 'ur3e_robotiq.urdf.xacro',
        'name:=ur',
        'ur_type:=ur3e',
        'sim_ignition:=true',
        f'simulation_controllers:={PACKAGE_ROOT / "config" / "ur_controllers.yaml"}',
    )
    robot = ElementTree.fromstring(xml)
    links = {link.attrib['name'] for link in robot.findall('link')}
    joints = {joint.attrib['name'] for joint in robot.findall('joint')}
    tcp_joint = next(
        joint for joint in robot.findall('joint')
        if joint.attrib['name'] == 'gripper_tcp_joint'
    )
    systems = {system.attrib['name'] for system in robot.findall('ros2_control')}

    assert {'base_link', 'tool0', 'robotiq_85_base_link', 'gripper_tcp'} <= links
    assert 'robotiq_85_left_knuckle_joint' in joints
    assert tcp_joint.find('origin').attrib['xyz'] == '0 0 0.135'
    assert systems == {'ur', 'robotiq_2f_85'}


def test_gazebo_mimic_joints_export_state_interfaces():
    """Gazebo must keep every passive finger joint aligned with RViz."""
    xml = _expand_xacro(
        PACKAGE_ROOT / 'urdf' / 'ur3e_robotiq.urdf.xacro',
        'name:=ur',
        'ur_type:=ur3e',
        'sim_ignition:=true',
        f'simulation_controllers:={PACKAGE_ROOT / "config" / "ur_controllers.yaml"}',
    )
    robot = ElementTree.fromstring(xml)
    gripper_system = next(
        system for system in robot.findall('ros2_control')
        if system.attrib['name'] == 'robotiq_2f_85'
    )
    passive_names = {
        'robotiq_85_right_knuckle_joint',
        'robotiq_85_left_inner_knuckle_joint',
        'robotiq_85_right_inner_knuckle_joint',
        'robotiq_85_left_finger_tip_joint',
        'robotiq_85_right_finger_tip_joint',
    }
    joints = {joint.attrib['name']: joint for joint in gripper_system.findall('joint')}
    assert passive_names <= joints.keys()
    for name in passive_names:
        state_interfaces = {
            item.attrib['name'] for item in joints[name].findall('state_interface')
        }
        assert {'position', 'velocity'} <= state_interfaces
        assert not joints[name].findall('command_interface')
        parameters = {
            item.attrib['name']: item.text for item in joints[name].findall('param')
        }
        assert parameters['mimic'] == 'robotiq_85_left_knuckle_joint'
        assert float(parameters['multiplier']) in {-1.0, 1.0}


def test_urdf_contains_fixed_overhead_camera_and_optical_frame():
    xml = _expand_xacro(
        PACKAGE_ROOT / 'urdf' / 'ur3e_robotiq.urdf.xacro',
        'name:=ur',
        'ur_type:=ur3e',
        'sim_ignition:=true',
        f'simulation_controllers:={PACKAGE_ROOT / "config" / "ur_controllers.yaml"}',
    )
    robot = ElementTree.fromstring(xml)
    links = {link.attrib['name'] for link in robot.findall('link')}
    joints = {
        joint.attrib['name']: joint for joint in robot.findall('joint')
    }
    camera_joint = joints['world_to_camera_joint']
    sensor = robot.find("gazebo[@reference='camera_link']/sensor")

    assert {'camera_link', 'camera_optical_frame'} <= links
    assert camera_joint.find('parent').attrib['link'] == 'world'
    assert camera_joint.find('child').attrib['link'] == 'camera_link'
    assert camera_joint.find('origin').attrib['xyz'] == '0.31 0 0.90'
    assert sensor is not None
    assert sensor.attrib['type'] == 'camera'
    assert sensor.findtext('topic') == '/camera'
    assert sensor.findtext('update_rate') == '15'
    assert sensor.findtext('camera/image/width') == '640'
    assert sensor.findtext('camera/image/height') == '480'


def test_camera_world_and_launch_enable_sensor_bridge():
    world = ElementTree.parse(
        PACKAGE_ROOT / 'worlds' / 'camera_empty.sdf'
    ).getroot()
    sensor_plugin = world.find(
        ".//plugin[@filename='ignition-gazebo-sensors-system']"
    )
    launch_source = (
        WORKSPACE_SRC / 'ur3_llm_control' / 'launch' / 'llm_robot.launch.py'
    ).read_text(encoding='utf-8')

    assert sensor_plugin is not None
    assert sensor_plugin.findtext('render_engine') == 'ogre2'
    assert '/camera@sensor_msgs/msg/Image[ignition.msgs.Image' in launch_source
    assert '/camera_info@sensor_msgs/msg/CameraInfo' in launch_source
    assert "('/camera', '/camera/image_raw')" in launch_source
    assert "('/camera_info', '/camera/camera_info')" in launch_source
    assert "'--frame-id', 'camera_optical_frame'" in launch_source


def test_srdf_defines_gripper_group_states_and_end_effector():
    xml = _expand_xacro(
        PACKAGE_ROOT / 'srdf' / 'ur_robotiq.srdf.xacro',
        'name:=ur',
        'prefix:=',
    )
    robot = ElementTree.fromstring(xml)
    groups = {group.attrib['name'] for group in robot.findall('group')}
    states = {
        (state.attrib['group'], state.attrib['name'])
        for state in robot.findall('group_state')
    }
    end_effectors = robot.findall('end_effector')
    disabled_pairs = {
        frozenset((item.attrib['link1'], item.attrib['link2']))
        for item in robot.findall('disable_collisions')
    }

    assert {'ur_manipulator', 'gripper'} <= groups
    assert {('gripper', 'open'), ('gripper', 'closed')} <= states
    assert len(end_effectors) == 1
    assert end_effectors[0].attrib['parent_link'] == 'tool0'
    assert end_effectors[0].attrib['group'] == 'gripper'
    assert frozenset((
        'robotiq_85_left_finger_tip_link',
        'robotiq_85_left_inner_knuckle_link',
    )) in disabled_pairs
    assert frozenset((
        'robotiq_85_right_finger_tip_link',
        'robotiq_85_right_inner_knuckle_link',
    )) in disabled_pairs


def test_controller_uses_humble_gripper_action_controller():
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'ur_controllers.yaml').read_text(encoding='utf-8')
    )
    manager = document['controller_manager']['ros__parameters']
    params = document['robotiq_gripper_controller']['ros__parameters']
    state_broadcaster = document['joint_state_broadcaster']['ros__parameters']

    assert manager['robotiq_gripper_controller']['type'] == (
        'position_controllers/GripperActionController'
    )
    assert params['joint'] == 'robotiq_85_left_knuckle_joint'
    assert params['allow_stalling'] is True
    assert state_broadcaster['joints'][-1] == 'robotiq_85_left_knuckle_joint'
    assert len(state_broadcaster['joints']) == 7


def test_robot_skills_use_physical_gripper_without_gazebo_pose_teleport():
    skills_root = WORKSPACE_SRC / 'ur3_robot_skills'
    config = yaml.safe_load(
        (skills_root / 'config' / 'robot_skills.yaml').read_text(encoding='utf-8')
    )['robot_skill_server']['ros__parameters']
    source = (skills_root / 'src' / 'robot_skill_server.cpp').read_text(
        encoding='utf-8'
    )

    assert config['end_effector_link'] == 'gripper_tcp'
    assert config['gripper_action'] == '/robotiq_gripper_controller/gripper_cmd'
    assert config['gripper_closed_position'] > config['gripper_open_position']
    assert config['object_tool_axis_approach']['purple_cube'] is True
    assert config['object_grasp_clearances']['purple_cube'] == 0.022
    assert config['object_orientations']['green_cube'] == [
        0.0, 0.98480775, 0.17364818, 0.0,
    ]
    assert {
        'robotiq_85_left_finger_tip_link',
        'robotiq_85_right_finger_tip_link',
    } <= set(config['gripper_touch_links'])
    assert 'control_msgs::action::GripperCommand' in source
    assert '/set_pose' not in source
    assert 'setGazeboGraspJoint' in source
    assert config['gazebo_pose_topic'] == '/world/empty/dynamic_pose/info'
    assert 'ignition::msgs::Pose_V' in source
    assert 'synchronizeGazeboPose(*object)' in source
    assert 'removeWorldObject(object_name)' in source
    assert 'planning_scene_.removeCollisionObjects({object_name})' in source
    assert 'best_distance > 5.5' not in source
    assert 'releaseToolPose(placed_object_pose, object_name)' in source
    assert 'preserved wrist orientation above ' in source
    assert 'replanned retreat with ' in source
    assert source.index('result = openGripper(feedback);', source.index('SkillResult place(')) < (
        source.index(
            'setGazeboGraspJoint(object_name, false)',
            source.index('SkillResult place('),
        )
    )
