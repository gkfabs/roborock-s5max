#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_path = get_package_share_path('roborock_s5max')
    model_path = package_path / 'urdf' / 'robot.urdf.xacro'
    rviz_path = package_path / 'rviz' / 'monitor_robot.rviz'
    bringup_path = package_path / 'launch' / 'bringup.launch.py'

    physical = LaunchConfiguration('physical')
    show_rviz = LaunchConfiguration('show_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = ParameterValue(
        Command(['xacro ', str(model_path)]),
        value_type=str,
    )

    physical_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_path)),
        condition=IfCondition(physical),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'robot_port': LaunchConfiguration('robot_port'),
            'use_sim_time': use_sim_time,
        }.items(),
    )

    model_only_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        condition=UnlessCondition(physical),
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    model_only_joint_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(physical),
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(show_rviz),
        output='screen',
        arguments=['-d', str(rviz_path)],
    )

    stop_when_rviz_closes = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason='RViz closed'))],
        ),
        condition=IfCondition(show_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'physical',
            default_value='true',
            choices=['true', 'false'],
            description='Start the physical SangamIO bridge.',
        ),
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
        DeclareLaunchArgument(
            'show_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Start the RViz window.',
        ),
        physical_bringup,
        model_only_state_publisher,
        model_only_joint_publisher,
        rviz,
        stop_when_rviz_closes,
    ])
