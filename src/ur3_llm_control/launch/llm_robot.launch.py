"""Launch the UR3e baseline and a YAML-defined Bài 03 scenario."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

from ur3_llm_control.scene_config import box_sdf, gazebo_boxes, load_scene


def _spawn_scene(context):
    scene_file = context.perform_substitution(LaunchConfiguration('scene_file'))
    scene = load_scene(scene_file)
    spawn_nodes = []
    for box in gazebo_boxes(scene):
        spawn_nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{box.name}',
                output='screen',
                arguments=[
                    '-string',
                    box_sdf(box),
                    '-name',
                    box.name,
                    '-allow_renaming',
                    'false',
                    '-x',
                    str(box.pose[0]),
                    '-y',
                    str(box.pose[1]),
                    '-z',
                    str(box.pose[2]),
                    '-R',
                    str(box.pose[3]),
                    '-P',
                    str(box.pose[4]),
                    '-Y',
                    str(box.pose[5]),
                ],
            )
        )
    # ros_gz_sim/create applies its initial_pose over the pose embedded in SDF.
    # Spawn the table first and give Gazebo time to register its collision
    # geometry before releasing the dynamic cubes.
    return [
        TimerAction(period=2.0, actions=[spawn_nodes[0]]),
        TimerAction(period=3.0, actions=spawn_nodes[1:]),
    ]


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    launch_rviz = LaunchConfiguration('launch_rviz')
    scene_file = LaunchConfiguration('scene_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    world_file = LaunchConfiguration('world_file')
    robotiq_resource_path = PathJoinSubstitution(
        [FindPackagePrefix('robotiq_description'), 'share']
    )

    simulation_and_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur_simulation_gz'), 'launch', 'ur_sim_moveit.launch.py']
            )
        ),
        launch_arguments={
            'ur_type': ur_type,
            'gazebo_gui': gazebo_gui,
            'launch_rviz': 'false',
            'launch_servo': 'false',
            'runtime_config_package': 'ur3_workcell_description',
            'controllers_file': 'ur_controllers.yaml',
            'description_package': 'ur3_workcell_description',
            'description_file': 'ur3e_robotiq.urdf.xacro',
            'moveit_config_package': 'ur3_workcell_description',
            'moveit_config_file': 'ur_robotiq.srdf.xacro',
            'world_file': world_file,
        }.items(),
    )
    simulation_group = GroupAction(actions=[simulation_and_moveit], scoped=True)

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_llm_scene',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare('ur3_workcell_description'),
                    'config',
                    'kinematics.yaml',
                ]
            ),
            {'use_sim_time': True},
        ],
        condition=IfCondition(launch_rviz),
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_robotiq_gripper_controller',
        output='screen',
        arguments=[
            'robotiq_gripper_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
    )

    scene_manager = Node(
        package='ur3_llm_control',
        executable='scene_manager',
        name='scene_manager',
        output='screen',
        parameters=[{'scene_file': scene_file, 'use_sim_time': True}],
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='overhead_camera_bridge',
        output='screen',
        arguments=[
            '/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
            (
                '/camera_info@sensor_msgs/msg/CameraInfo'
                '[ignition.msgs.CameraInfo'
            ),
        ],
        # Keep the Gazebo transport topic /camera while exposing the standard
        # ROS image topic expected by RViz, image tools and Milestone 4.
        remappings=[
            ('/camera', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
    )

    # Gazebo scopes the sensor header as ur/camera_link/overhead_camera. Map
    # that data frame to the standard ROS optical frame so every image and
    # CameraInfo message has a complete TF chain back to world.
    camera_sensor_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='overhead_camera_sensor_tf',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'camera_optical_frame',
            '--child-frame-id', 'ur/camera_link/overhead_camera',
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'ur_type',
                default_value='ur3e',
                choices=['ur3', 'ur3e'],
                description='Universal Robots model; the assignment defaults to UR3e.',
            ),
            DeclareLaunchArgument(
                'gazebo_gui',
                default_value='true',
                choices=['true', 'false'],
                description='Start Gazebo with its graphical interface.',
            ),
            DeclareLaunchArgument(
                'launch_rviz',
                default_value='true',
                choices=['true', 'false'],
                description='Start RViz with MotionPlanning and scene markers.',
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
                description='Spawn configuration for objects and target areas.',
            ),
            DeclareLaunchArgument(
                'rviz_config_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('ur3_llm_control'), 'rviz', 'scene.rviz']
                ),
                description='RViz configuration for the Bài 03 scene.',
            ),
            DeclareLaunchArgument(
                'world_file',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('ur3_workcell_description'),
                        'worlds',
                        'camera_empty.sdf',
                    ]
                ),
                description='Gazebo world with the rendering sensor system.',
            ),
            # Gazebo converts package:// mesh URIs to model:// URIs.  The
            # parent of robotiq_description must therefore be on both the
            # Fortress and newer Gazebo resource search paths.
            SetEnvironmentVariable(
                'IGN_GAZEBO_RESOURCE_PATH',
                [
                    robotiq_resource_path,
                    ':',
                    EnvironmentVariable(
                        'IGN_GAZEBO_RESOURCE_PATH', default_value=''
                    ),
                ],
            ),
            SetEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                [
                    robotiq_resource_path,
                    ':',
                    EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
                ],
            ),
            simulation_group,
            gripper_controller_spawner,
            rviz,
            scene_manager,
            camera_bridge,
            camera_sensor_tf,
            OpaqueFunction(function=_spawn_scene),
        ]
    )
