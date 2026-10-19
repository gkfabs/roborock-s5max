#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from vacuum_ros2_bridge.msg import VacuumStatus
from vacuum_ros2_bridge.srv import SetActuator


def steering_command(bearing, linear_speed, gain, maximum_angular_speed):
    angular = float(bearing) * float(gain)
    angular = max(-maximum_angular_speed, min(maximum_angular_speed, angular))
    return float(linear_speed), angular


def planar_distance(start_x, start_y, current_x, current_y):
    return math.hypot(current_x - start_x, current_y - start_y)


class DockFinalApproach(Node):
    def __init__(self):
        super().__init__('s5max_dock_final_approach')
        self.declare_parameter('linear_speed', 0.035)
        self.declare_parameter('steering_gain', 1.5)
        self.declare_parameter('maximum_angular_speed', 0.4)
        self.declare_parameter('maximum_distance', 0.45)
        self.declare_parameter('approach_timeout_sec', 15.0)
        self.declare_parameter('data_timeout_sec', 0.6)
        self.declare_parameter('charging_timeout_sec', 3.0)
        self.declare_parameter('charging_stable_sec', 0.3)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.steering_gain = float(self.get_parameter('steering_gain').value)
        self.maximum_angular_speed = float(
            self.get_parameter('maximum_angular_speed').value)
        self.maximum_distance = float(
            self.get_parameter('maximum_distance').value)
        self.approach_timeout = float(
            self.get_parameter('approach_timeout_sec').value)
        self.data_timeout = float(
            self.get_parameter('data_timeout_sec').value)
        self.charging_timeout = float(
            self.get_parameter('charging_timeout_sec').value)
        self.charging_stable = float(
            self.get_parameter('charging_stable_sec').value)

        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel_clean', 10)
        self.create_subscription(
            Bool, '/dock_drive/beacon_visible', self._on_visible, 10)
        self.create_subscription(
            Float32, '/dock_detector/beacon_bearing', self._on_bearing, 10)
        self.create_subscription(
            VacuumStatus, '/vacuum_status', self._on_status, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.charger_client = self.create_client(SetActuator, '/set_actuator')
        self.timer = self.create_timer(0.05, self._tick)

        now = time.monotonic()
        self.startup_deadline = now + 5.0
        self.approach_deadline = None
        self.charging_deadline = None
        self.visible = False
        self.visible_time = None
        self.bearing = None
        self.bearing_time = None
        self.status = None
        self.status_time = None
        self.position = None
        self.odom_time = None
        self.start_position = None
        self.charger_future = None
        self.charging_since = None
        self.state = 'WAITING'
        self.done = False
        self.success = False

    def _on_visible(self, msg):
        self.visible = bool(msg.data)
        self.visible_time = time.monotonic()

    def _on_bearing(self, msg):
        self.bearing = float(msg.data)
        self.bearing_time = time.monotonic()

    def _on_status(self, msg):
        self.status = msg
        self.status_time = time.monotonic()

    def _on_odom(self, msg):
        self.position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        self.odom_time = time.monotonic()

    def _publish_stop(self):
        self.velocity_pub.publish(Twist())

    def _fail(self, message):
        if self.done:
            return
        self._publish_stop()
        self.get_logger().error(message)
        self.done = True

    def _finish(self):
        self._publish_stop()
        self.get_logger().info('Charging confirmed. The final dock approach succeeded.')
        self.success = True
        self.done = True

    def _fresh(self, timestamp, now):
        return timestamp is not None and now - timestamp <= self.data_timeout

    def _tick(self):
        if self.done:
            return
        now = time.monotonic()

        if self.state == 'WAITING':
            ready = (
                self._fresh(self.visible_time, now)
                and self._fresh(self.bearing_time, now)
                and self._fresh(self.status_time, now)
                and self._fresh(self.odom_time, now)
                and self.visible
                and self.bearing is not None
                and self.status is not None
                and not self.status.is_docked
                and self.position is not None
            )
            if ready:
                self.start_position = self.position
                self.approach_deadline = now + self.approach_timeout
                self.state = 'APPROACHING'
                self.get_logger().info('Starting the bounded final dock approach.')
                return
            if now >= self.startup_deadline:
                self._fail('Fresh dock beacon, odometry, and undocked status are required.')
            return

        if self.state == 'APPROACHING':
            if self.status is not None and self.status.is_docked:
                self._publish_stop()
                if not self.charger_client.wait_for_service(timeout_sec=0.1):
                    self._fail('The charger service is unavailable.')
                    return
                request = SetActuator.Request()
                request.component = 'charger'
                request.speed = 1
                self.charger_future = self.charger_client.call_async(request)
                self.charging_deadline = now + self.charging_timeout
                self.state = 'ENABLING_CHARGER'
                return

            if now >= self.approach_deadline:
                self._fail('The final dock approach reached its time limit.')
                return
            if not self._fresh(self.odom_time, now):
                self._fail('Odometry became stale during the final dock approach.')
                return
            if not self._fresh(self.visible_time, now) or not self.visible:
                self._fail('The dock beacon disappeared during the final approach.')
                return
            if not self._fresh(self.bearing_time, now):
                self._fail('The dock beacon bearing became stale.')
                return

            distance = planar_distance(*self.start_position, *self.position)
            if distance >= self.maximum_distance:
                self._fail('The final dock approach reached its distance limit.')
                return

            linear, angular = steering_command(
                self.bearing,
                self.linear_speed,
                self.steering_gain,
                self.maximum_angular_speed,
            )
            command = Twist()
            command.linear.x = linear
            command.angular.z = angular
            self.velocity_pub.publish(command)
            return

        if self.state == 'ENABLING_CHARGER':
            if self._charging_is_stable(now):
                return
            if self.charger_future is not None and self.charger_future.done():
                try:
                    response = self.charger_future.result()
                except Exception as error:
                    self._fail(f'The charger request failed: {error}')
                    return
                if not response.success:
                    self._fail(f'The charger request failed: {response.message}')
                    return
                self.state = 'WAITING_FOR_CHARGE'
            if now >= self.charging_deadline:
                self._fail('Charging did not start before the time limit.')
            return

        if self.state == 'WAITING_FOR_CHARGE':
            if self._charging_is_stable(now):
                return
            if now >= self.charging_deadline:
                self._fail('Charging did not start before the time limit.')

    def _charging_is_stable(self, now):
        if self.status is None or not self.status.is_charging:
            self.charging_since = None
            return False
        if self.charging_since is None:
            self.charging_since = now
            return False
        if now - self.charging_since < self.charging_stable:
            return False
        self._finish()
        return True


def main(args=None):
    rclpy.init(args=args)
    node = DockFinalApproach()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node._fail('The final dock approach was interrupted.')
    finally:
        node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if node.success else 1)


if __name__ == '__main__':
    main()
