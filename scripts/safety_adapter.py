#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from vacuum_ros2_bridge.msg import Bumper, Cliff


class SafetyAdapter(Node):
    def __init__(self):
        super().__init__('s5max_safety_adapter')
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.odom_timeout = float(
            self.get_parameter('odom_timeout_sec').value
        )
        publish_rate = float(self.get_parameter('publish_rate_hz').value)

        self.stop_publisher = self.create_publisher(
            Twist, '/cmd_vel_safety', 10
        )
        self.status_publisher = self.create_publisher(
            Bool, '/safety_stop', 10
        )
        self.create_subscription(
            Bumper, '/bumper', self.on_bumper, qos_profile_sensor_data
        )
        self.create_subscription(
            Cliff, '/cliff', self.on_cliff, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, '/odom', self.on_odom, qos_profile_sensor_data
        )
        self.create_service(Trigger, '/reset_safety', self.on_reset)
        self.timer = self.create_timer(1.0 / publish_rate, self.on_timer)

        self.last_odom = None
        self.bumper_active = False
        self.cliff_active = False
        self.latched_reason = None
        self.last_reported_reason = object()

    def on_bumper(self, message):
        self.bumper_active = message.left or message.right
        if self.bumper_active:
            self.latched_reason = 'bumper contact'

    def on_cliff(self, message):
        self.cliff_active = any((
            message.left_side,
            message.left_front,
            message.right_front,
            message.right_side,
        ))
        if self.cliff_active:
            self.latched_reason = 'cliff detected'

    def on_odom(self, _message):
        self.last_odom = time.monotonic()

    def current_reason(self):
        if self.latched_reason is not None:
            return self.latched_reason
        if self.last_odom is None:
            return 'waiting for odometry'
        if time.monotonic() - self.last_odom > self.odom_timeout:
            self.latched_reason = 'odometry stale'
            return self.latched_reason
        return None

    def on_timer(self):
        reason = self.current_reason()
        blocked = reason is not None
        self.status_publisher.publish(Bool(data=blocked))
        if blocked:
            self.stop_publisher.publish(Twist())
        if reason != self.last_reported_reason:
            if reason is None:
                self.get_logger().info('Safety stop released')
            else:
                self.get_logger().warning(f'Safety stop active: {reason}')
            self.last_reported_reason = reason

    def on_reset(self, _request, response):
        if self.bumper_active:
            response.success = False
            response.message = 'The bumper remains active.'
            return response
        if self.cliff_active:
            response.success = False
            response.message = 'A cliff sensor remains active.'
            return response
        if self.last_odom is None:
            response.success = False
            response.message = 'Odometry is not available.'
            return response
        if time.monotonic() - self.last_odom > self.odom_timeout:
            response.success = False
            response.message = 'Odometry remains stale.'
            return response

        self.latched_reason = None
        response.success = True
        response.message = 'Safety stop released.'
        return response


def main():
    rclpy.init()
    node = SafetyAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
