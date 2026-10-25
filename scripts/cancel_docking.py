#!/usr/bin/env python3

import argparse
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


ACTIVE_STATUSES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}


def goal_id(goal_info):
    return bytes(goal_info.goal_id.uuid)


def target_goals_are_stopped(status_list, target_ids):
    active_ids = {
        goal_id(entry.goal_info)
        for entry in status_list
        if entry.status in ACTIVE_STATUSES
    }
    return not active_ids.intersection(target_ids)


class DockingCancellation(Node):
    def __init__(self):
        super().__init__('s5max_docking_cancellation')
        self.stop_client = self.create_client(Trigger, '/stop_motion')
        self.cancel_clients = {
            '/dock_robot': self.create_client(
                CancelGoal, '/dock_robot/_action/cancel_goal'),
            '/undock_robot': self.create_client(
                CancelGoal, '/undock_robot/_action/cancel_goal'),
        }
        self.statuses = {
            '/dock_robot': [],
            '/undock_robot': [],
        }
        self.status_updates = {
            '/dock_robot': 0,
            '/undock_robot': 0,
        }
        self.create_subscription(
            GoalStatusArray,
            '/dock_robot/_action/status',
            lambda message: self._on_status('/dock_robot', message),
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            '/undock_robot/_action/status',
            lambda message: self._on_status('/undock_robot', message),
            10,
        )
        self.safety_publisher = self.create_publisher(
            Twist, '/cmd_vel_safety', 10)

    def _on_status(self, action_name, message):
        self.statuses[action_name] = list(message.status_list)
        self.status_updates[action_name] += 1

    def _spin_with_zero(self, future, deadline):
        while (
                rclpy.ok()
                and not future.done()
                and time.monotonic() < deadline):
            self.safety_publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def latch_operator_stop(self, timeout):
        deadline = time.monotonic() + timeout
        while (
                rclpy.ok()
                and time.monotonic() < deadline
                and not self.stop_client.wait_for_service(timeout_sec=0.1)):
            self.safety_publisher.publish(Twist())
        if not self.stop_client.service_is_ready():
            self.get_logger().error('The operator safety stop is unavailable.')
            return False
        future = self.stop_client.call_async(Trigger.Request())
        if not self._spin_with_zero(future, deadline):
            self.get_logger().error('The operator safety stop timed out.')
            return False
        response = future.result()
        if response is None or not response.success:
            message = response.message if response is not None else 'no response'
            self.get_logger().error(
                f'The operator safety stop failed: {message}')
            return False
        self.get_logger().info(response.message)
        return True

    def cancel_action(self, action_name, timeout):
        client = self.cancel_clients[action_name]
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(
                f'{action_name} is unavailable. No goal needs cancellation.')
            return True

        deadline = time.monotonic() + timeout
        request = CancelGoal.Request()
        update_before_request = self.status_updates[action_name]
        future = client.call_async(request)
        if not self._spin_with_zero(future, deadline):
            self.get_logger().error(
                f'The cancellation request for {action_name} timed out.')
            return False
        response = future.result()
        if response is None or response.return_code != 0:
            return_code = response.return_code if response is not None else -1
            self.get_logger().error(
                f'The cancellation request for {action_name} failed with '
                f'code {return_code}.')
            return False

        target_ids = {goal_id(item) for item in response.goals_canceling}
        if not target_ids:
            self.get_logger().info(
                f'{action_name} has no active goal.')
            return True

        while rclpy.ok() and time.monotonic() < deadline:
            self.safety_publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                    self.status_updates[action_name] > update_before_request
                    and target_goals_are_stopped(
                        self.statuses[action_name], target_ids)):
                self.get_logger().info(
                    f'{action_name} confirmed cancellation.')
                return True
        self.get_logger().error(
            f'{action_name} did not stop before the cancellation timeout.')
        return False

    def hold_zero_velocity(self, duration):
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.safety_publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--zero-hold', type=float, default=0.5)
    options, _ = parser.parse_known_args(args)

    rclpy.init()
    node = DockingCancellation()
    success = False
    try:
        stop_active = node.latch_operator_stop(options.timeout)
        dock_stopped = node.cancel_action('/dock_robot', options.timeout)
        undock_stopped = node.cancel_action('/undock_robot', options.timeout)
        node.hold_zero_velocity(max(0.0, options.zero_hold))
        success = stop_active and dock_stopped and undock_stopped
    except (KeyboardInterrupt, ExternalShutdownException):
        node.hold_zero_velocity(max(0.0, options.zero_hold))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
