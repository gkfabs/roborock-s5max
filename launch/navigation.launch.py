#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_path = get_package_share_path('roborock_s5max')
    bringup_launch = package_path / 'launch' / 'bringup.launch.py'
    slam_launch = (
        get_package_share_path('slam_toolbox')
        / 'launch'
        / 'localization_launch.py'
    )

    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')
    posegraph_path = LaunchConfiguration('posegraph_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    localization_params = RewrittenYaml(
        source_file=str(
            package_path / 'config' / 'mapper_params_localization.yaml'
        ),
        root_key='',
        param_rewrites={'map_file_name': posegraph_path},
        convert_types=True,
    )
    navigation_params = str(package_path / 'config' / 'navigation.yaml')
    common_parameters = [
        navigation_params,
        {'use_sim_time': use_sim_time},
    ]
    transform_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_launch)),
        launch_arguments={
            'robot_ip': robot_ip,
            'robot_port': robot_port,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(slam_launch)),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': localization_params,
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
        }.items(),
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=common_parameters,
        remappings=transform_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )
    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=common_parameters,
        remappings=transform_remappings,
    )
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=common_parameters,
        remappings=transform_remappings,
    )
    behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=common_parameters,
        remappings=transform_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )
    navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=common_parameters,
        remappings=transform_remappings,
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
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
            'posegraph_path',
            description='Saved SLAM Toolbox pose graph path without extension.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use the simulation clock.',
        ),
        bringup,
        localization,
        controller,
        smoother,
        planner,
        behaviors,
        navigator,
        lifecycle,
    ])
