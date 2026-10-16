#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_path = get_package_share_path('roborock_s5max')
    model_path = package_path / 'urdf' / 'robot.urdf.xacro'
    bridge_launch = (
        get_package_share_path('vacuum_ros2_bridge')
        / 'launch'
        / 'bridge.launch.py'
    )

    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command(['xacro ', str(model_path)]),
        value_type=str,
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bridge_launch)),
        launch_arguments={
            'robot_ip': robot_ip,
            'robot_port': robot_port,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'lidar_frame_id': 'base_scan',
            'publish_tf': 'true',
            'undock_before_lidar': 'true',
            'scan_time_offset': '0.0',
            'scan_mask_deg': '',
        }.items(),
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    joint_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    velocity_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[str(package_path / 'config' / 'twist_mux.yaml')],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
    )

    safety_adapter = Node(
        package='roborock_s5max',
        executable='safety_adapter.py',
        name='s5max_safety_adapter',
        output='screen',
        parameters=[{
            'odom_timeout_sec': 0.5,
            'publish_rate_hz': 20.0,
            'cliff_stop_seconds': 0.10,
            'cliff_backup_seconds': 0.60,
            'cliff_backup_speed': -0.05,
            'command_fresh_seconds': 0.30,
            'minimum_forward_speed': 0.01,
        }],
    )

    bumper_adapter = Node(
        package='roborock_s5max',
        executable='bumper_adapter.py',
        name='s5max_bumper_adapter',
        output='screen',
    )

    contact_reflex = Node(
        package='roborock_s5max',
        executable='contact_reflex.py',
        name='s5max_contact_reflex',
        output='screen',
        parameters=[{
            'publish_rate_hz': 20.0,
            'stop_seconds': 0.10,
            'backup_seconds': 0.60,
            'turn_seconds': 0.60,
            'settle_seconds': 0.10,
            'backup_speed': -0.05,
            'turn_speed': 0.45,
            'command_fresh_seconds': 0.30,
            'minimum_forward_speed': 0.01,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.4.55',
            description='Address of the S5 Max that runs SangamIO.',
        ),
        DeclareLaunchArgument(
            'robot_port',
            default_value='5555',
            description='SangamIO TCP port.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use the simulation clock.',
        ),
        bridge,
        velocity_mux,
        safety_adapter,
        bumper_adapter,
        contact_reflex,
        state_publisher,
        joint_publisher,
    ])
