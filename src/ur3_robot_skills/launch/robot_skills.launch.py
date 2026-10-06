"""Launch the Milestone 2 gripper scene, skill server, and optional demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ur3_llm_control.scene_config import load_scene


def _create_skill_nodes(context):
    scene_path = context.perform_substitution(LaunchConfiguration('scene_file'))
    scene = load_scene(scene_path)

    scene_parameters = {
        'world_name': scene.world_name,
        'valid_objects': list(scene.objects.keys()),
        'valid_zones': list(scene.zones.keys()),
        'valid_temporary_slots': list(scene.temporary_slots.keys()),
        'use_sim_time': True,
    }
    for name, zone in scene.zones.items():
        scene_parameters[f'zones.{name}.pose'] = list(zone.pose)
        scene_parameters[f'zones.{name}.size'] = list(zone.size)
    for name, slot in scene.temporary_slots.items():
        scene_parameters[f'temporary_slots.{name}.pose'] = list(slot.pose)
        scene_parameters[f'temporary_slots.{name}.size'] = list(slot.size)
    for name, object_spec in scene.objects.items():
        # The skill server temporarily removes a cube before grasp contact.
        # Keep its scene color keyed by ID when the cube is added again.
        scene_parameters[f'object_colors.{name}'] = list(object_spec.color)

    skill_server = Node(
        package='ur3_robot_skills',
        executable='robot_skill_server',
        name='robot_skill_server',
        output='screen',
        parameters=[
            LaunchConfiguration('skill_config_file'),
            PathJoinSubstitution(
                [FindPackageShare('ur_moveit_config'), 'config', 'kinematics.yaml']
            ),
            scene_parameters,
        ],
    )

    demo = Node(
        package='ur3_robot_skills',
        executable='robot_skill_demo',
        name='robot_skill_demo',
        output='screen',
        parameters=[{
            'server_wait_seconds': 120.0,
            'start_delay_seconds': 3.0,
            'object_name': LaunchConfiguration('demo_object'),
            'zone_name': LaunchConfiguration('demo_zone'),
            'use_sim_time': True,
        }],
        condition=IfCondition(LaunchConfiguration('run_demo')),
    )
    return [skill_server, demo]


def generate_launch_description():
    scene_file = LaunchConfiguration('scene_file')

    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur3_perception'), 'launch', 'perception.launch.py']
            )
        ),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'scene_file': scene_file,
            'launch_perception': 'true',
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'ur_type',
                default_value='ur3e',
                choices=['ur3', 'ur3e'],
            ),
            DeclareLaunchArgument(
                'gazebo_gui',
                default_value='true',
                choices=['true', 'false'],
            ),
            DeclareLaunchArgument(
                'launch_rviz',
                default_value='true',
                choices=['true', 'false'],
            ),
            DeclareLaunchArgument(
                'run_demo',
                default_value='true',
                choices=['true', 'false'],
                description=(
                    'Run the independent pick(object) -> place(object, zone) '
                    'physical-gripper check.'
                ),
            ),
            DeclareLaunchArgument(
                'demo_object',
                default_value='red_cube',
                choices=[
                    'red_cube',
                    'yellow_cube',
                    'blue_cube',
                    'green_cube',
                    'purple_cube',
                ],
            ),
            DeclareLaunchArgument(
                'demo_zone',
                default_value='zone_a',
                choices=['zone_a', 'zone_b', 'zone_c'],
            ),
            DeclareLaunchArgument(
                'scene_file',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('ur3_llm_control'),
                        'config',
                        'scene_clear.yaml',
                    ]
                ),
            ),
            DeclareLaunchArgument(
                'skill_config_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('ur3_robot_skills'), 'config', 'robot_skills.yaml']
                ),
            ),
            baseline,
            OpaqueFunction(function=_create_skill_nodes),
        ]
    )
