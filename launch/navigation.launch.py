#!/usr/bin/env python3

import json
import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def make_localization(context, package_path, slam_launch, posegraph_path,
                      start_pose_path, use_sim_time):
    pose_path = context.perform_substitution(start_pose_path)
    graph_path = context.perform_substitution(posegraph_path)
    x = 0.0
    y = 0.0
    yaw = 0.0
    if pose_path and os.path.isfile(pose_path):
        with open(pose_path, encoding='utf-8') as source:
            data = json.load(source)
        stage = data['stage']
        x = float(stage['x'])
        y = float(stage['y'])
        yaw = float(stage['yaw'])
        print(
            'Localization start pose: '
            f'x={x:.3f} y={y:.3f} yaw={yaw:.3f} from {pose_path}')
    elif pose_path:
        print(
            f'Localization start pose file not found: {pose_path}. '
            'Using the pose graph origin.')

    source_path = package_path / 'config' / 'mapper_params_localization.yaml'
    with open(source_path, encoding='utf-8') as source:
        parameters = yaml.safe_load(source)
    slam_parameters = parameters['slam_toolbox']['ros__parameters']
    slam_parameters['map_file_name'] = graph_path
    slam_parameters['map_start_pose'] = [x, y, yaw]
    temporary = tempfile.NamedTemporaryFile(
        mode='w', prefix='s5max-localization-', suffix='.yaml', delete=False)
    yaml.safe_dump(parameters, temporary)
    temporary.close()
    localization_params = temporary.name
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(slam_launch)),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': localization_params,
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
        }.items(),
    )]


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
    start_pose_path = LaunchConfiguration('start_pose_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    navigation_params = str(package_path / 'config' / 'navigation.yaml')
    docking_params = str(package_path / 'config' / 'docking.yaml')
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
    localization = OpaqueFunction(
        function=make_localization,
        args=[
            package_path,
            slam_launch,
            posegraph_path,
            start_pose_path,
            use_sim_time,
        ],
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=common_parameters,
        remappings=transform_remappings + [('cmd_vel', 'cmd_vel_nav_raw')],
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
        remappings=transform_remappings + [('cmd_vel', 'cmd_vel_nav_raw')],
    )
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=common_parameters,
        remappings=transform_remappings,
    )
    navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=common_parameters,
        remappings=transform_remappings,
    )
    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[docking_params, {'use_sim_time': use_sim_time}],
        remappings=(
            transform_remappings
            + [('cmd_vel', 'cmd_vel_nav_raw')]
        ),
    )
    dock_pose_estimator = Node(
        package='roborock_s5max',
        executable='dock_pose_estimator.py',
        name='s5max_dock_pose_estimator',
        output='screen',
        parameters=[docking_params, {'use_sim_time': use_sim_time}],
    )
    localization_pose_adapter = Node(
        package='roborock_s5max',
        executable='localization_pose_adapter.py',
        name='s5max_localization_pose_adapter',
        output='screen',
    )
    navigation_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': ParameterValue(autostart, value_type=bool),
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'collision_monitor',
            ],
        }],
    )
    docking_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{
            'autostart': ParameterValue(autostart, value_type=bool),
            'node_names': ['docking_server'],
            'use_sim_time': use_sim_time,
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
            'start_pose_path',
            default_value='',
            description='Dock pose JSON used as the localization start pose.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use the simulation clock.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            choices=['true', 'false'],
            description='Activate the navigation and docking servers at launch.',
        ),
        bringup,
        localization,
        controller,
        smoother,
        planner,
        behaviors,
        navigator,
        docking_server,
        dock_pose_estimator,
        localization_pose_adapter,
        collision_monitor,
        navigation_lifecycle,
        docking_lifecycle,
    ])
