#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool
from vacuum_ros2_bridge.msg import ActuatorCommand


def state_qos():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class CleaningActuatorManager(Node):
    def __init__(self):
        super().__init__('s5max_cleaning_actuators')
        self.declare_parameter('vacuum_speed', 38)
        self.declare_parameter('main_brush_speed', 71)
        self.declare_parameter('side_brush_speed', 30)

        self.speeds = {
            'vacuum': self._speed('vacuum_speed'),
            'main_brush': self._speed('main_brush_speed'),
            'side_brush': self._speed('side_brush_speed'),
        }
        self.publisher = self.create_publisher(
            ActuatorCommand, '/actuator_cmd', 10
        )
        self.create_subscription(
            Bool,
            '/coverage_planner/cleaning_active',
            self.on_cleaning_active,
            state_qos(),
        )
        self.active = False

    def _speed(self, name):
        return max(0, min(100, int(self.get_parameter(name).value)))

    def _publish(self, component, speed):
        message = ActuatorCommand()
        message.component = component
        message.speed = speed
        self.publisher.publish(message)

    def on_cleaning_active(self, message):
        active = bool(message.data)
        if active == self.active:
            return
        self.active = active
        if active:
            for component, speed in self.speeds.items():
                self._publish(component, speed)
            self.get_logger().info('Cleaning actuators started.')
        else:
            self.stop()
            self.get_logger().info('Cleaning actuators stopped.')

    def stop(self):
        for component in ('vacuum', 'main_brush', 'side_brush', 'water_pump'):
            self._publish(component, 0)


def main():
    rclpy.init()
    node = CleaningActuatorManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
