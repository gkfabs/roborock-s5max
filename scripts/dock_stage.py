#!/usr/bin/env python3

import argparse
import json
import math
import os
import sys
import time
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def angle_difference(left, right):
    return math.remainder(left - right, 2.0 * math.pi)


def planar_distance(left, right):
    return math.hypot(left[0] - right[0], left[1] - right[1])


def approach_pose(stage, distance):
    return (
        stage[0] - distance * math.cos(stage[2]),
        stage[1] - distance * math.sin(stage[2]),
        stage[2],
    )


def pose_from_estimate(message):
    if message.header.frame_id != 'map':
        raise ValueError('The final pose estimate must use the map frame.')
    pose = message.pose.pose
    return (
        float(pose.position.x),
        float(pose.position.y),
        yaw_from_quaternion(pose.orientation),
    )


def stable_pose(samples, maximum_position_spread=0.03,
                maximum_yaw_spread=math.radians(5.0)):
    if len(samples) < 8:
        return None
    mean_x = sum(sample[0] for sample in samples) / len(samples)
    mean_y = sum(sample[1] for sample in samples) / len(samples)
    sin_yaw = sum(math.sin(sample[2]) for sample in samples)
    cos_yaw = sum(math.cos(sample[2]) for sample in samples)
    mean_yaw = math.atan2(sin_yaw, cos_yaw)
    position_spread = max(
        math.hypot(sample[0] - mean_x, sample[1] - mean_y)
        for sample in samples
    )
    yaw_spread = max(
        abs(angle_difference(sample[2], mean_yaw))
        for sample in samples
    )
    if position_spread > maximum_position_spread:
        return None
    if yaw_spread > maximum_yaw_spread:
        return None
    return mean_x, mean_y, mean_yaw


def write_stage(path, map_name, pose):
    data = {
        'format': 1,
        'map': map_name,
        'frame': 'map',
        'stage': {
            'x': pose[0],
            'y': pose[1],
            'yaw': pose[2],
        },
    }
    temporary_path = f'{path}.tmp'
    with open(temporary_path, 'w', encoding='utf-8') as output:
        json.dump(data, output, indent=2)
        output.write('\n')
    os.replace(temporary_path, path)


def read_stage(path, map_name):
    with open(path, encoding='utf-8') as source:
        data = json.load(source)
    if data.get('format') != 1:
        raise ValueError('Unsupported dock staging file format.')
    if data.get('map') != map_name:
        raise ValueError('The dock staging pose belongs to a different map.')
    if data.get('frame') != 'map':
        raise ValueError('The dock staging pose does not use the map frame.')
    stage = data.get('stage', {})
    return float(stage['x']), float(stage['y']), float(stage['yaw'])


class StageRecorder(Node):
    def __init__(self):
        super().__init__('s5max_dock_stage_recorder')
        self.samples = deque(maxlen=20)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def sample(self):
        try:
            transform = self.buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
            )
        except TransformException:
            return
        pose = transform.transform
        self.samples.append((
            float(pose.translation.x),
            float(pose.translation.y),
            yaw_from_quaternion(pose.rotation),
        ))


class EstimateRecorder(Node):
    def __init__(self):
        super().__init__('s5max_dock_estimate_recorder')
        self.pose = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.on_estimate,
            10,
        )

    def on_estimate(self, message):
        try:
            self.pose = pose_from_estimate(message)
        except ValueError as error:
            self.get_logger().error(str(error))
            return


class StageNavigator(Node):
    def __init__(self, pose):
        super().__init__('s5max_dock_stage_navigator')
        self.pose = pose
        self.odom = None
        self.distance_remaining = None
        self.commands = {
            'raw': None,
            'filtered': None,
            'output': None,
        }
        self.diagnostics_active = False
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.cancel_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal',
        )
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(
            Twist,
            '/cmd_vel_nav_raw',
            self.command_callback('raw'),
            10,
        )
        self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.command_callback('filtered'),
            10,
        )
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.command_callback('output'),
            10,
        )
        self.create_timer(1.0, self.report_diagnostics)

    def on_odom(self, message):
        self.odom = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )

    def command_callback(self, name):
        def callback(message):
            self.commands[name] = (
                float(message.linear.x),
                float(message.angular.z),
            )
        return callback

    def on_feedback(self, message):
        self.distance_remaining = float(message.feedback.distance_remaining)

    @staticmethod
    def format_pair(value):
        if value is None:
            return 'unavailable'
        return '(%.3f,%.3f)' % value

    def report_diagnostics(self):
        if not self.diagnostics_active:
            return
        remaining = 'unavailable'
        if self.distance_remaining is not None:
            remaining = '%.3f' % self.distance_remaining
        self.get_logger().info(
            'Dock transit: remaining=%s odom=%s raw=%s filtered=%s output=%s' % (
                remaining,
                self.format_pair(self.odom),
                self.format_pair(self.commands['raw']),
                self.format_pair(self.commands['filtered']),
                self.format_pair(self.commands['output']),
            ))

    def distance_to_stage(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self.buffer.lookup_transform(
                    'map',
                    'base_footprint',
                    rclpy.time.Time(),
                )
            except TransformException:
                continue
            current = (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
            )
            return planar_distance(current, self.pose)
        return None

    def cancel_active_navigation(self, timeout, settle_time=1.0):
        if not self.cancel_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                'The Nav2 cancellation service is unavailable.')
            return False

        cancel_future = self.cancel_client.call_async(CancelGoal.Request())
        rclpy.spin_until_future_complete(
            self, cancel_future, timeout_sec=timeout)
        if not cancel_future.done():
            self.get_logger().error(
                'Nav2 did not process the cancellation request in time.')
            return False

        response = cancel_future.result()
        if response is None or response.return_code != CancelGoal.Response.ERROR_NONE:
            self.get_logger().error(
                'Nav2 rejected the cancellation request.')
            return False

        if response.goals_canceling:
            self.get_logger().info(
                'Canceled the active navigation goal before docking.')
        deadline = time.monotonic() + settle_time
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def navigate(self, server_timeout, travel_timeout, near_distance,
                 approach_distance):
        if not self.cancel_active_navigation(server_timeout):
            return False

        distance = self.distance_to_stage(server_timeout)
        if distance is not None and distance <= near_distance:
            self.get_logger().info(
                'The robot is %.3f m from the dock staging pose. '
                'Skipping Nav2 and starting beacon docking.' % distance)
            return True

        if not self.client.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error('The Nav2 action server is unavailable.')
            return False

        target = approach_pose(self.pose, approach_distance)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target[0]
        goal.pose.pose.position.y = target[1]
        qx, qy, qz, qw = quaternion_from_yaw(target[2])
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.diagnostics_active = True
        send_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.on_feedback,
        )
        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=server_timeout)
        if not send_future.done():
            self.get_logger().error('Nav2 did not accept the staging goal in time.')
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the dock staging goal.')
            return False

        self.get_logger().info(
            'Nav2 accepted the dock approach goal at '
            'x=%.3f y=%.3f, %.2f m before the staging pose.' % (
                target[0], target[1], approach_distance))
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=travel_timeout)
        if not result_future.done():
            self.get_logger().error('Travel to the dock staging pose timed out.')
            goal_handle.cancel_goal_async()
            return False
        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'Nav2 ended the dock staging goal with status {result.status}.')
            return False
        self.get_logger().info('The robot reached the dock approach pose.')
        return True


def record(path, map_name, timeout):
    node = StageRecorder()
    deadline = time.monotonic() + timeout
    pose = None
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        node.sample()
        pose = stable_pose(node.samples)
        if pose is not None:
            break
    node.destroy_node()
    if pose is None:
        print('ERROR: A stable localized pose was not available.', file=sys.stderr)
        return 1
    write_stage(path, map_name, pose)
    print(
        'Saved dock staging pose: '
        f'x={pose[0]:.3f} y={pose[1]:.3f} yaw={pose[2]:.3f}')
    return 0


def watch_estimates(path, map_name):
    node = EstimateRecorder()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.pose is None:
            continue
        write_stage(path, map_name, node.pose)
        print(
            'Updated dock staging pose from RViz: '
            f'x={node.pose[0]:.3f} y={node.pose[1]:.3f} '
            f'yaw={node.pose[2]:.3f}',
            flush=True,
        )
        node.pose = None
    node.destroy_node()
    return 0


def navigate(path, map_name, server_timeout, travel_timeout, near_distance,
             approach_distance):
    try:
        pose = read_stage(path, map_name)
    except (OSError, KeyError, TypeError, ValueError) as error:
        print(f'ERROR: Cannot read the dock staging pose: {error}', file=sys.stderr)
        return 1
    node = StageNavigator(pose)
    success = node.navigate(
        server_timeout,
        travel_timeout,
        near_distance,
        approach_distance,
    )
    node.destroy_node()
    return 0 if success else 1


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'mode', choices=('record', 'watch-estimates', 'navigate'))
    parser.add_argument('--path', required=True)
    parser.add_argument('--map-name', required=True)
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--travel-timeout', type=float, default=300.0)
    parser.add_argument('--near-distance', type=float, default=0.50)
    parser.add_argument('--approach-distance', type=float, default=0.45)
    options = parser.parse_args(args)

    rclpy.init()
    try:
        if options.mode == 'record':
            status = record(options.path, options.map_name, options.timeout)
        elif options.mode == 'watch-estimates':
            status = watch_estimates(options.path, options.map_name)
        else:
            status = navigate(
                options.path,
                options.map_name,
                options.timeout,
                options.travel_timeout,
                options.near_distance,
                options.approach_distance,
            )
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(status)


if __name__ == '__main__':
    main()
