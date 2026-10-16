#!/usr/bin/env python3

from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, String


IDLE = 'IDLE'
STOP = 'STOP'
BACKUP = 'BACKUP'
TURN = 'TURN'
SETTLE = 'SETTLE'
LOCKOUT = 'LOCKOUT'


def state_qos():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


@dataclass(frozen=True)
class RecoveryCommand:
    active: bool
    linear_x: float
    angular_z: float
    state: str


class ContactRecovery:
    def __init__(
        self,
        stop_seconds=0.10,
        backup_seconds=0.60,
        turn_seconds=0.60,
        settle_seconds=0.10,
        backup_speed=-0.05,
        turn_speed=0.45,
    ):
        self.stop_seconds = float(stop_seconds)
        self.backup_seconds = float(backup_seconds)
        self.turn_seconds = float(turn_seconds)
        self.settle_seconds = float(settle_seconds)
        self.backup_speed = -abs(float(backup_speed))
        self.turn_speed = abs(float(turn_speed))
        self.left = False
        self.right = False
        self.state = IDLE
        self.started_at = None
        self.turn_direction = 1.0
        self.next_head_on_direction = 1.0

    def update_contacts(self, left, right, now, allow_start=True):
        self.left = bool(left)
        self.right = bool(right)
        if (
            self.state == IDLE
            and allow_start
            and (self.left or self.right)
        ):
            self._start(now)
        elif self.state == LOCKOUT and not self.left and not self.right:
            self.state = IDLE
            self.started_at = None

    def command(self, now):
        if self.state == IDLE:
            return RecoveryCommand(False, 0.0, 0.0, IDLE)
        if self.state == LOCKOUT:
            return RecoveryCommand(True, 0.0, 0.0, LOCKOUT)

        elapsed = max(0.0, float(now) - self.started_at)
        backup_end = self.stop_seconds + self.backup_seconds
        turn_end = backup_end + self.turn_seconds
        settle_end = turn_end + self.settle_seconds

        if elapsed < self.stop_seconds:
            self.state = STOP
            return RecoveryCommand(True, 0.0, 0.0, STOP)
        if elapsed < backup_end:
            self.state = BACKUP
            return RecoveryCommand(
                True, self.backup_speed, 0.0, BACKUP
            )
        if elapsed < turn_end:
            self.state = TURN
            return RecoveryCommand(
                True, 0.0, self.turn_direction * self.turn_speed, TURN
            )
        if elapsed < settle_end:
            self.state = SETTLE
            return RecoveryCommand(True, 0.0, 0.0, SETTLE)

        if self.left or self.right:
            self.state = LOCKOUT
            return RecoveryCommand(True, 0.0, 0.0, LOCKOUT)

        self.state = IDLE
        self.started_at = None
        return RecoveryCommand(False, 0.0, 0.0, IDLE)

    def _start(self, now):
        self.started_at = float(now)
        self.state = STOP
        if self.left and not self.right:
            self.turn_direction = -1.0
        elif self.right and not self.left:
            self.turn_direction = 1.0
        else:
            self.turn_direction = self.next_head_on_direction
            self.next_head_on_direction *= -1.0


class ContactReflex(Node):
    def __init__(self):
        super().__init__('s5max_contact_reflex')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('stop_seconds', 0.10)
        self.declare_parameter('backup_seconds', 0.60)
        self.declare_parameter('turn_seconds', 0.60)
        self.declare_parameter('settle_seconds', 0.10)
        self.declare_parameter('backup_speed', -0.05)
        self.declare_parameter('turn_speed', 0.45)
        self.declare_parameter('command_fresh_seconds', 0.30)
        self.declare_parameter('minimum_forward_speed', 0.01)

        self.recovery = ContactRecovery(
            stop_seconds=self._parameter('stop_seconds'),
            backup_seconds=self._parameter('backup_seconds'),
            turn_seconds=self._parameter('turn_seconds'),
            settle_seconds=self._parameter('settle_seconds'),
            backup_speed=self._parameter('backup_speed'),
            turn_speed=self._parameter('turn_speed'),
        )
        self.command_publisher = self.create_publisher(
            Twist, '/cmd_vel_contact', 10
        )
        self.active_publisher = self.create_publisher(
            Bool, '/contact_reflex/active', state_qos()
        )
        self.state_publisher = self.create_publisher(
            String, '/contact_reflex/state', state_qos()
        )
        self.create_subscription(
            Bool, '/bumper/left', self.on_left, state_qos()
        )
        self.create_subscription(
            Bool, '/bumper/right', self.on_right, state_qos()
        )
        self.create_subscription(
            Twist, '/cmd_vel', self.on_drive_command, 10
        )

        self.left = False
        self.right = False
        self.last_forward_command = None
        self.command_fresh_seconds = self._parameter(
            'command_fresh_seconds'
        )
        self.minimum_forward_speed = self._parameter(
            'minimum_forward_speed'
        )
        self.last_state = None
        self.was_active = False
        rate = max(1.0, self._parameter('publish_rate_hz'))
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

    def _parameter(self, name):
        return float(self.get_parameter(name).value)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_left(self, message):
        self.left = bool(message.data)
        now = self._now()
        self.recovery.update_contacts(
            self.left, self.right, now, self._forward_is_active(now)
        )

    def on_right(self, message):
        self.right = bool(message.data)
        now = self._now()
        self.recovery.update_contacts(
            self.left, self.right, now, self._forward_is_active(now)
        )

    def on_drive_command(self, message):
        if message.linear.x > self.minimum_forward_speed:
            now = self._now()
            self.last_forward_command = now
            self.recovery.update_contacts(
                self.left, self.right, now, allow_start=True
            )

    def _forward_is_active(self, now):
        return (
            self.last_forward_command is not None
            and now - self.last_forward_command
            <= self.command_fresh_seconds
        )

    def on_timer(self):
        command = self.recovery.command(self._now())
        if command.active or self.was_active:
            message = Twist()
            message.linear.x = command.linear_x
            message.angular.z = command.angular_z
            self.command_publisher.publish(message)
        self.active_publisher.publish(Bool(data=command.active))
        if command.state != self.last_state:
            self.state_publisher.publish(String(data=command.state))
            if command.state == STOP:
                self.get_logger().warning(
                    'Bumper contact detected. Starting bounded recovery.'
                )
            elif command.state == IDLE and self.last_state is not None:
                self.get_logger().info('Bumper recovery complete.')
            elif command.state == LOCKOUT:
                self.get_logger().warning(
                    'Bumper remains pressed. Holding zero velocity.'
                )
            self.last_state = command.state
        self.was_active = command.active


def main():
    rclpy.init()
    node = ContactReflex()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.command_publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
