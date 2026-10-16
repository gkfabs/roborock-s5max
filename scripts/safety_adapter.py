#!/usr/bin/env python3

import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from vacuum_ros2_bridge.msg import Cliff


IDLE = 'idle'
STOP = 'stop'
BACKUP = 'backup'
LOCKOUT = 'lockout'


@dataclass(frozen=True)
class RecoveryCommand:
    state: str
    linear_x: float = 0.0

    @property
    def active(self):
        return self.state != IDLE


class CliffRecovery:
    def __init__(self, stop_seconds, backup_seconds, backup_speed):
        self.stop_seconds = stop_seconds
        self.backup_seconds = backup_seconds
        self.backup_speed = backup_speed
        self.sensor_active = False
        self.consumed = False
        self.started_at = None

    def update(self, active, now, allow_start):
        rising = active and not self.sensor_active
        self.sensor_active = active
        if not active:
            self.consumed = False
            self.started_at = None
        elif rising and allow_start and not self.consumed:
            self.consumed = True
            self.started_at = now

    def command(self, now):
        if self.started_at is None:
            return RecoveryCommand(IDLE)
        elapsed = now - self.started_at
        if elapsed < self.stop_seconds:
            return RecoveryCommand(STOP)
        if elapsed < self.stop_seconds + self.backup_seconds:
            return RecoveryCommand(BACKUP, self.backup_speed)
        return RecoveryCommand(LOCKOUT)


class SafetyAdapter(Node):
    def __init__(self):
        super().__init__('s5max_safety_adapter')
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('cliff_stop_seconds', 0.10)
        self.declare_parameter('cliff_backup_seconds', 0.60)
        self.declare_parameter('cliff_backup_speed', -0.05)
        self.declare_parameter('command_fresh_seconds', 0.30)
        self.declare_parameter('minimum_forward_speed', 0.01)
        self.odom_timeout = float(
            self.get_parameter('odom_timeout_sec').value
        )
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.command_fresh_seconds = float(
            self.get_parameter('command_fresh_seconds').value
        )
        self.minimum_forward_speed = float(
            self.get_parameter('minimum_forward_speed').value
        )
        self.cliff_recovery = CliffRecovery(
            stop_seconds=float(
                self.get_parameter('cliff_stop_seconds').value
            ),
            backup_seconds=float(
                self.get_parameter('cliff_backup_seconds').value
            ),
            backup_speed=float(
                self.get_parameter('cliff_backup_speed').value
            ),
        )

        self.stop_publisher = self.create_publisher(
            Twist, '/cmd_vel_safety', 10
        )
        self.status_publisher = self.create_publisher(
            Bool, '/safety_stop', 10
        )
        self.create_subscription(
            Cliff, '/cliff', self.on_cliff, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, '/odom', self.on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            Twist, '/cmd_vel', self.on_drive_command, 10
        )
        self.create_service(Trigger, '/reset_safety', self.on_reset)
        self.timer = self.create_timer(1.0 / publish_rate, self.on_timer)

        self.last_odom = None
        self.last_forward_command = None
        self.cliff_active = False
        self.latched_reason = None
        self.last_reported_reason = object()

    def on_cliff(self, message):
        now = time.monotonic()
        was_active = self.cliff_active
        active = any((
            message.left_side,
            message.left_front,
            message.right_front,
            message.right_side,
        ))
        allow_start = (
            self.last_forward_command is not None
            and now - self.last_forward_command
            <= self.command_fresh_seconds
        )
        self.cliff_recovery.update(active, now, allow_start)
        self.cliff_active = active
        if self.cliff_active:
            self.latched_reason = 'cliff detected'
        elif was_active and self.latched_reason == 'cliff detected':
            self.latched_reason = None

    def on_odom(self, _message):
        self.last_odom = time.monotonic()

    def on_drive_command(self, message):
        if message.linear.x > self.minimum_forward_speed:
            self.last_forward_command = time.monotonic()

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
            command = self.cliff_recovery.command(time.monotonic())
            output = Twist()
            if (
                command.state == BACKUP
                and self.cliff_active
                and self.last_odom is not None
                and time.monotonic() - self.last_odom <= self.odom_timeout
            ):
                output.linear.x = command.linear_x
            self.stop_publisher.publish(output)
        if reason != self.last_reported_reason:
            if reason is None:
                self.get_logger().info('Safety stop released')
            else:
                self.get_logger().warning(f'Safety stop active: {reason}')
            self.last_reported_reason = reason

    def on_reset(self, _request, response):
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
        self.cliff_recovery.update(False, time.monotonic(), False)
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
