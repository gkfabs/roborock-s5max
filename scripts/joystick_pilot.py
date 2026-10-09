#!/usr/bin/env python3
import time
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from vacuum_ros2_bridge.msg import ActuatorCommand


class JoystickPilot(Node):
    LINEAR_SCALE = 0.20
    ANGULAR_SCALE = 1.20
    AXIS_DEADZONE = 0.08
    ENABLE_BUTTON = 3
    TOGGLES = {
        0: ("vacuum", 38),
        1: ("main_brush", 71),
        2: ("side_brush", 30),
    }

    def __init__(self):
        super().__init__("s5max_joystick_pilot")
        self.drive = self.create_publisher(Twist, "/cmd_vel", 10)
        self.actuator = self.create_publisher(
            ActuatorCommand, "/actuator_cmd", 10
        )
        self.subscription = self.create_subscription(
            Joy, "/joy", self.on_joy, qos_profile_sensor_data
        )
        self.timer = self.create_timer(0.05, self.refresh_drive)
        self.axes = [0.0, 0.0]
        self.buttons = None
        self.last_joy = 0.0
        self.last_drive_log = 0.0
        self.actuator_state = {name: False for name, _ in self.TOGGLES.values()}
        self.get_logger().info(
            "Hold button 3 to drive or toggle motors. "
            "Buttons: 0 vacuum, 1 main brush, 2 side brush."
        )

    def publish_drive(self, linear=0.0, angular=0.0):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.drive.publish(message)

    def publish_actuator(self, component, speed):
        message = ActuatorCommand()
        message.component = component
        message.speed = speed
        self.actuator.publish(message)

    @classmethod
    def analog_axis(cls, value):
        value = max(-1.0, min(1.0, float(value)))
        magnitude = abs(value)
        if magnitude <= cls.AXIS_DEADZONE:
            return 0.0
        normalized = (magnitude - cls.AXIS_DEADZONE) / (
            1.0 - cls.AXIS_DEADZONE
        )
        return math.copysign(normalized, value)

    def on_joy(self, message):
        now = time.monotonic()
        self.last_joy = now
        self.axes = list(message.axes)
        current = list(message.buttons)
        if self.buttons is None:
            self.buttons = current
            return

        enabled = current[self.ENABLE_BUTTON] == 1
        if enabled:
            for button, (component, speed) in self.TOGGLES.items():
                if current[button] == 1 and self.buttons[button] == 0:
                    active = not self.actuator_state[component]
                    self.actuator_state[component] = active
                    self.publish_actuator(component, speed if active else 0)
                    state = "on" if active else "off"
                    self.get_logger().info(f"{component} {state}")
        self.buttons = current

    def refresh_drive(self):
        if self.buttons is None or time.monotonic() - self.last_joy > 0.25:
            self.publish_drive()
            return
        if self.buttons[self.ENABLE_BUTTON] == 0:
            self.publish_drive()
            return
        linear_axis = self.analog_axis(self.axes[1])
        angular_axis = self.analog_axis(self.axes[0])
        linear = linear_axis * self.LINEAR_SCALE
        angular = angular_axis * self.ANGULAR_SCALE
        self.publish_drive(linear, angular)
        now = time.monotonic()
        if now - self.last_drive_log >= 0.5:
            self.get_logger().info(
                f"analog axes=({self.axes[0]:.3f},{self.axes[1]:.3f}) "
                f"cmd_vel=({linear:.3f} m/s,{angular:.3f} rad/s)"
            )
            self.last_drive_log = now

    def stop_all(self):
        for _ in range(5):
            self.publish_drive()
            for component in self.actuator_state:
                self.publish_actuator(component, 0)
            rclpy.spin_once(self, timeout_sec=0.05)


rclpy.init()
node = JoystickPilot()
try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    if rclpy.ok():
        node.stop_all()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
