#!/usr/bin/env python3

import argparse
import json
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import DockRobot, UndockRobot
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


FEEDBACK_STATES = {
    DockRobot.Feedback.NAV_TO_STAGING_POSE: 'NAV_TO_STAGING_POSE',
    DockRobot.Feedback.INITIAL_PERCEPTION: 'INITIAL_PERCEPTION',
    DockRobot.Feedback.CONTROLLING: 'CONTROLLING',
    DockRobot.Feedback.WAIT_FOR_CHARGE: 'WAIT_FOR_CHARGE',
    DockRobot.Feedback.RETRY: 'RETRY',
}


def publishes_feedback_state(state, navigate_to_staging):
    return not (
        state == 'NAV_TO_STAGING_POSE' and not bool(navigate_to_staging))


def quaternion_from_yaw(yaw):
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


def dock_pose_from_stage(stage, offset):
    x, y, yaw = stage
    return (
        x + float(offset) * math.cos(yaw),
        y + float(offset) * math.sin(yaw),
        yaw,
    )


def read_dock_pose(path, map_name, offset):
    with open(path, encoding='utf-8') as source:
        data = json.load(source)
    if data.get('map') != map_name:
        raise ValueError('The dock pose belongs to a different map.')
    if data.get('frame') != 'map':
        raise ValueError('The dock pose does not use the map frame.')
    if 'dock' in data:
        pose = data['dock']
        return float(pose['x']), float(pose['y']), float(pose['yaw'])
    stage = data.get('stage', {})
    return dock_pose_from_stage((
        float(stage['x']),
        float(stage['y']),
        float(stage['yaw']),
    ), offset)


def parse_bool(value):
    normalized = str(value).strip().lower()
    if normalized == 'true':
        return True
    if normalized == 'false':
        return False
    raise ValueError(f'Invalid Boolean value: {value}')


class DockRobotClient(Node):
    def __init__(self, pose, frame, navigate_to_staging, staging_timeout):
        super().__init__('s5max_dock_robot_client')
        self.pose = pose
        self.frame = frame
        self.navigate_to_staging = navigate_to_staging
        self.staging_timeout = staging_timeout
        self.client = ActionClient(self, DockRobot, '/dock_robot')
        self.undock_client = ActionClient(self, UndockRobot, '/undock_robot')
        self.lifecycle_client = self.create_client(
            GetState, '/docking_server/get_state')
        self.result = None
        self.goal_handle = None
        self.last_feedback_state = None
        self.last_retry_count = 0
        self.detected_pose = None
        inhibit_qos = QoSProfile(depth=1)
        inhibit_qos.reliability = ReliabilityPolicy.RELIABLE
        inhibit_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.inhibit_publisher = self.create_publisher(
            Bool, '/contact_reflex/inhibit', inhibit_qos)
        self.state_publisher = self.create_publisher(
            String, '/dock_robot/state', inhibit_qos)
        self.create_subscription(
            PoseStamped,
            '/detected_dock_pose',
            self.detection,
            10,
        )
        self.publish_state('WAITING')

    def publish_state(self, state):
        self.state_publisher.publish(String(data=str(state)))

    def inhibit_contact_reflex(self, inhibited):
        self.inhibit_publisher.publish(Bool(data=bool(inhibited)))

    def detection(self, message):
        if message.header.frame_id:
            self.detected_pose = message

    def feedback(self, wrapped):
        feedback = wrapped.feedback
        state = FEEDBACK_STATES.get(feedback.state, f'UNKNOWN_{feedback.state}')
        if feedback.num_retries > self.last_retry_count:
            self.last_retry_count = feedback.num_retries
            self.publish_state('RETRY')
        if state != self.last_feedback_state:
            self.get_logger().info(
                f'Docking state: {state}, retries={feedback.num_retries}')
            self.last_feedback_state = state
            if publishes_feedback_state(state, self.navigate_to_staging):
                self.publish_state(state)

    def wait_for_active_server(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if not self.lifecycle_client.wait_for_service(
                    timeout_sec=min(0.5, remaining)):
                continue
            future = self.lifecycle_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=min(1.0, remaining))
            if not future.done() or future.result() is None:
                continue
            state = future.result().current_state
            if state.id == State.PRIMARY_STATE_ACTIVE:
                return True
            self.get_logger().info(
                f'Waiting for the docking server. Current state: {state.label}.')
            time.sleep(0.2)
        self.get_logger().error(
            'The docking server did not enter the active lifecycle state.')
        return False

    def start(self, server_timeout):
        self.get_logger().info('Waiting for the active docking server.')
        if not self.wait_for_active_server(server_timeout):
            return False
        self.get_logger().info('The docking server is active.')
        self.get_logger().info('Waiting for the Nav2 DockRobot action.')
        if not self.client.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error('The Nav2 DockRobot action is unavailable.')
            return False
        self.get_logger().info('The Nav2 DockRobot action is available.')
        if self.navigate_to_staging:
            self.get_logger().info(
                'Sending the dock goal. OpenNav will navigate to staging '
                'before initial dock detection.')
        else:
            self.publish_state('INITIAL_PERCEPTION')
            self.get_logger().info('Waiting for a detected dock pose.')
            detection_deadline = time.monotonic() + server_timeout
            while (
                    rclpy.ok()
                    and self.detected_pose is None
                    and time.monotonic() < detection_deadline):
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.detected_pose is None:
                self.get_logger().error(
                    'A valid detected dock pose was not available before docking.')
                return False
            self.get_logger().info(
                'A detected dock pose is available in frame %s.'
                % self.detected_pose.header.frame_id)
        goal = DockRobot.Goal()
        goal.use_dock_id = False
        goal.dock_type = 's5max'
        goal.navigate_to_staging_pose = self.navigate_to_staging
        goal.max_staging_time = float(self.staging_timeout)
        goal.dock_pose = PoseStamped()
        goal.dock_pose.header.frame_id = self.frame
        goal.dock_pose.header.stamp = self.get_clock().now().to_msg()
        goal.dock_pose.pose.position.x = self.pose[0]
        goal.dock_pose.pose.position.y = self.pose[1]
        qx, qy, qz, qw = quaternion_from_yaw(self.pose[2])
        goal.dock_pose.pose.orientation.x = qx
        goal.dock_pose.pose.orientation.y = qy
        goal.dock_pose.pose.orientation.z = qz
        goal.dock_pose.pose.orientation.w = qw

        future = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        rclpy.spin_until_future_complete(self, future, timeout_sec=server_timeout)
        if not future.done() or future.result() is None:
            self.get_logger().error('The DockRobot goal request timed out.')
            return False
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('The DockRobot goal was rejected.')
            return False
        self.get_logger().info(
            'DockRobot accepted target x=%.3f y=%.3f yaw=%.3f.' % self.pose)
        return True

    def wait(self, operation_timeout):
        future = self.goal_handle.get_result_async()
        deadline = time.monotonic() + operation_timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            self.get_logger().error('The DockRobot action reached its client time limit.')
            self.goal_handle.cancel_goal_async()
            return False
        wrapped = future.result()
        result = wrapped.result
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            self.get_logger().error(
                'DockRobot failed: status=%d code=%d retries=%d message=%s' % (
                    wrapped.status,
                    result.error_code,
                    result.num_retries,
                    result.error_msg,
                ))
            return False
        self.get_logger().info(
            f'DockRobot succeeded after {result.num_retries} retries.')
        return True

    def undock(self, server_timeout, operation_timeout):
        self.get_logger().info('Waiting for the Nav2 UndockRobot action.')
        if not self.undock_client.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error('The Nav2 UndockRobot action is unavailable.')
            return False
        goal = UndockRobot.Goal()
        goal.dock_type = 's5max'
        goal.max_undocking_time = float(operation_timeout)
        future = self.undock_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=server_timeout)
        if not future.done() or future.result() is None:
            self.get_logger().error('The UndockRobot goal request timed out.')
            return False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('The UndockRobot goal was rejected.')
            return False
        self.get_logger().info('UndockRobot accepted the s5max dock type.')
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + operation_timeout
        while (
                rclpy.ok()
                and not result_future.done()
                and time.monotonic() < deadline):
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result_future.done():
            self.get_logger().error(
                'The UndockRobot action reached its client time limit.')
            goal_handle.cancel_goal_async()
            return False
        wrapped = result_future.result()
        result = wrapped.result
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            self.get_logger().error(
                'UndockRobot failed: status=%d code=%d' % (
                    wrapped.status,
                    result.error_code,
                ))
            return False
        self.get_logger().info('UndockRobot succeeded.')
        return True


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--path')
    parser.add_argument('--map-name')
    parser.add_argument('--dock-offset', type=float, default=0.215)
    parser.add_argument('--dock-x', type=float)
    parser.add_argument('--dock-y', type=float)
    parser.add_argument('--dock-yaw', type=float)
    parser.add_argument('--dock-frame', default='map')
    parser.add_argument('--skip-staging', action='store_true')
    parser.add_argument('--server-timeout', type=float, default=30.0)
    parser.add_argument('--staging-timeout', type=float, default=300.0)
    parser.add_argument('--operation-timeout', type=float, default=420.0)
    parser.add_argument('--undock-after', default='false')
    parser.add_argument('--undock-timeout', type=float, default=30.0)
    parser.add_argument('--result-path')
    options, _ = parser.parse_known_args(args)

    try:
        direct = (options.dock_x, options.dock_y, options.dock_yaw)
        if all(value is not None for value in direct):
            pose = direct
        elif options.path and options.map_name:
            pose = read_dock_pose(
                options.path, options.map_name, options.dock_offset)
        else:
            raise ValueError(
                'Use --path with --map-name or provide all direct dock coordinates.')
    except (OSError, KeyError, TypeError, ValueError) as error:
        print(f'ERROR: Cannot read the dock pose: {error}', file=sys.stderr)
        raise SystemExit(1) from error

    rclpy.init()
    node = DockRobotClient(
        pose,
        frame=options.dock_frame,
        navigate_to_staging=not options.skip_staging,
        staging_timeout=options.staging_timeout,
    )
    dock_success = False
    undock_success = None
    success = False
    try:
        node.inhibit_contact_reflex(True)
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.start(options.server_timeout):
            dock_success = node.wait(options.operation_timeout)
            success = dock_success
            if dock_success and parse_bool(options.undock_after):
                undock_success = node.undock(
                    options.server_timeout,
                    options.undock_timeout,
                )
                success = undock_success
    except (KeyboardInterrupt, ExternalShutdownException):
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
    finally:
        if rclpy.ok():
            node.inhibit_contact_reflex(False)
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if options.result_path:
        with open(options.result_path, 'w', encoding='utf-8') as target:
            json.dump({
                'success': success,
                'dock_success': dock_success,
                'undock_success': undock_success,
                'last_state': node.last_feedback_state,
            }, target)
            target.write('\n')
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
