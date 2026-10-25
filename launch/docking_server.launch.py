#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_path = get_package_share_path('roborock_s5max')
    docking_params = str(package_path / 'config' / 'docking.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[
            docking_params,
            {
                'use_sim_time': use_sim_time,
                'controller.use_collision_detection': ParameterValue(
                    LaunchConfiguration('collision_detection'),
                    value_type=bool),
            },
        ],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
    )
    pose_estimator = Node(
        package='roborock_s5max',
        executable='dock_pose_estimator.py',
        name='s5max_dock_pose_estimator',
        output='screen',
        parameters=[
            docking_params,
            {
                'use_sim_time': use_sim_time,
                'scan_topic': LaunchConfiguration('scan_topic'),
            },
        ],
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['docking_server'],
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('collision_detection', default_value='false'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_nav'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        docking_server,
        pose_estimator,
        lifecycle,
    ])
