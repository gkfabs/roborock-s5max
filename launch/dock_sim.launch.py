#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_path = get_package_share_path('roborock_s5max')
    world_path = package_path / 'worlds' / 'dock_test.world'
    model_path = package_path / 'urdf' / 'robot.urdf.xacro'
    gazebo_launch = (
        get_package_share_path('ros_gz_sim')
        / 'launch'
        / 'gz_sim.launch.py'
    )
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    scenario = LaunchConfiguration('scenario')
    result_path = LaunchConfiguration('result_path')
    docking_params = str(package_path / 'config' / 'docking.yaml')
    robot_description = ParameterValue(
        Command(['xacro ', str(model_path)]),
        value_type=str,
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_launch)),
        launch_arguments={
            'gz_args': f'-s -r --headless-rendering -v 1 {world_path}',
            'on_exit_shutdown': 'true',
        }.items(),
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_launch)),
        launch_arguments={
            'gz_args': '-g -v 1',
            'on_exit_shutdown': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': str(package_path / 'config' / 'gz_bridge.yaml'),
            'use_sim_time': True,
        }],
    )
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'default',
            '-topic', 'robot_description',
            '-name', 'roborock_s5max',
            '-timeout', '120',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.03',
            '-Y', yaw,
            '-allow_renaming', 'false',
        ],
    )
    adapter = Node(
        package='roborock_s5max',
        executable='dock_sim_adapter.py',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_origin_x': ParameterValue(x_pose, value_type=float),
            'odom_origin_y': ParameterValue(y_pose, value_type=float),
            'odom_origin_yaw': ParameterValue(yaw, value_type=float),
            'scenario': scenario,
        }],
    )
    scan_adapter = Node(
        package='roborock_s5max',
        executable='dock_scan_fault_adapter.py',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'noise_profile': LaunchConfiguration('lidar_noise_profile'),
            'outage_start_sec': ParameterValue(
                LaunchConfiguration('lidar_outage_start_sec'),
                value_type=float),
            'outage_duration_sec': ParameterValue(
                LaunchConfiguration('lidar_outage_duration_sec'),
                value_type=float),
            'outage_state': LaunchConfiguration('lidar_outage_state'),
            'outage_state_delay_sec': ParameterValue(
                LaunchConfiguration('lidar_outage_state_delay_sec'),
                value_type=float),
        }],
    )
    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[
            docking_params,
            {
                'use_sim_time': True,
                'controller.use_collision_detection': False,
                's5max.external_detection_translation_y': 0.0,
            },
        ],
        remappings=[('cmd_vel', '/cmd_vel')],
    )
    pose_estimator = Node(
        package='roborock_s5max',
        executable='dock_pose_estimator.py',
        name='s5max_dock_pose_estimator',
        output='screen',
        parameters=[docking_params, {'use_sim_time': True}],
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['docking_server'],
            'use_sim_time': True,
        }],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=[
            '-d', str(package_path / 'rviz' / 'monitor_robot.rviz'),
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('gui')),
    )
    dock_action = Node(
        package='roborock_s5max',
        executable='dock_robot.py',
        output='screen',
        arguments=[
            '--dock-x', '-0.0365',
            '--dock-y', '0.0',
            '--dock-yaw', '0.0',
            '--dock-frame', 'odom',
            '--skip-staging',
            '--server-timeout', '45',
            '--operation-timeout', '90',
            '--undock-after', LaunchConfiguration('undock_after_docking'),
            '--undock-timeout', '30',
            '--result-path', result_path,
        ],
        parameters=[{'use_sim_time': True}],
    )
    start_docking = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[
                TimerAction(period=8.0, actions=[dock_action]),
            ],
        ))
    stop_after_docking = RegisterEventHandler(
        OnProcessExit(
            target_action=dock_action,
            on_exit=[EmitEvent(event=Shutdown(reason='docking finished'))],
        ))

    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='-0.25'),
        DeclareLaunchArgument('y_pose', default_value='0.00'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('scenario', default_value='centered'),
        DeclareLaunchArgument(
            'undock_after_docking', default_value='false'),
        DeclareLaunchArgument(
            'result_path', default_value='/tmp/s5max-dock-result.json'),
        DeclareLaunchArgument(
            'lidar_noise_profile', default_value='realistic'),
        DeclareLaunchArgument('lidar_outage_start_sec', default_value='0.0'),
        DeclareLaunchArgument('lidar_outage_duration_sec', default_value='0.0'),
        DeclareLaunchArgument('lidar_outage_state', default_value=''),
        DeclareLaunchArgument(
            'lidar_outage_state_delay_sec', default_value='0.25'),
        gazebo_server,
        gazebo_client,
        state_publisher,
        bridge,
        rviz,
        adapter,
        scan_adapter,
        docking_server,
        pose_estimator,
        lifecycle,
        spawn,
        start_docking,
        stop_after_docking,
    ])
