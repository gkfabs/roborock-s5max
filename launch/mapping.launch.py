#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_path = get_package_share_path('roborock_s5max')
    bringup_launch = package_path / 'launch' / 'bringup.launch.py'
    slam_launch = (
        get_package_share_path('slam_toolbox')
        / 'launch'
        / 'online_async_launch.py'
    )

    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')
    use_sim_time = LaunchConfiguration('use_sim_time')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_launch)),
        launch_arguments={
            'robot_ip': robot_ip,
            'robot_port': robot_port,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(slam_launch)),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': str(
                package_path / 'config' / 'mapper_params_online_async.yaml'
            ),
        }.items(),
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
        bringup,
        slam,
    ])
