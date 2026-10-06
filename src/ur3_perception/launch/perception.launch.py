"""Launch simulation, camera bridge and camera-based SceneState perception."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_perception = LaunchConfiguration('launch_perception')
    scene_file = LaunchConfiguration('scene_file')
    config_file = LaunchConfiguration('perception_config_file')
    calibration_file = LaunchConfiguration('camera_calibration_file')

    workcell = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur3_llm_control'), 'launch', 'llm_robot.launch.py']
            )
        ),
        launch_arguments={
            'ur_type': ur_type,
            'gazebo_gui': gazebo_gui,
            'launch_rviz': launch_rviz,
            'scene_file': scene_file,
        }.items(),
    )
    detector = Node(
        package='ur3_perception',
        executable='block_detector',
        name='block_detector',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'scene_file': scene_file,
                'config_file': config_file,
                'calibration_file': calibration_file,
            }
        ],
        condition=IfCondition(launch_perception),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('ur_type', default_value='ur3e'),
            DeclareLaunchArgument('gazebo_gui', default_value='true'),
            DeclareLaunchArgument('launch_rviz', default_value='true'),
            DeclareLaunchArgument('launch_perception', default_value='true'),
            DeclareLaunchArgument(
                'scene_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('ur3_llm_control'), 'config', 'scene_clear.yaml']
                ),
            ),
            DeclareLaunchArgument(
                'perception_config_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('ur3_perception'), 'config', 'perception.yaml']
                ),
            ),
            DeclareLaunchArgument(
                'camera_calibration_file',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('ur3_llm_control'),
                        'config',
                        'camera_calibration.yaml',
                    ]
                ),
            ),
            workcell,
            detector,
        ]
    )
