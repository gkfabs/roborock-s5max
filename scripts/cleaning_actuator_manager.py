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
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from vacuum_ros2_bridge.msg import ActuatorCommand


def state_qos():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class CompletionLatch:
    def __init__(self):
        self.seen_active = False
        self.completed = False
        self.success = False
        self.message = ''

    def update(self, active):
        if active:
            self.seen_active = True
        return self.completed

    def set_result(self, result):
        self.message = str(result)
        self.success = self.message == 'succeeded'
        self.completed = self.success or self.message.startswith('failed:')
        return self.completed


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
        self.create_subscription(
            String,
            '/coverage_planner/result',
            self.on_result,
            state_qos(),
        )
        self.create_service(
            Trigger,
            '~/start',
            self.on_start,
        )
        self.active = False
        self.completion = CompletionLatch()

    def _speed(self, name):
        return max(0, min(100, int(self.get_parameter(name).value)))

    def _publish(self, component, speed):
        message = ActuatorCommand()
        message.component = component
        message.speed = speed
        self.publisher.publish(message)

    def on_cleaning_active(self, message):
        active = bool(message.data)
        self.completion.update(active)
        if active == self.active:
            return
        if active:
            self.start()
        else:
            self.stop()
            self.get_logger().info('Cleaning actuators stopped.')

    def on_result(self, message):
        if not self.completion.set_result(message.data):
            return
        self.stop()
        if self.completion.success:
            self.get_logger().info('Coverage completed successfully.')
        else:
            self.get_logger().error(message.data)

    def on_start(self, _request, response):
        self.start()
        response.success = True
        response.message = 'Cleaning actuators started.'
        return response

    def start(self):
        if self.active:
            return
        self.active = True
        for component, speed in self.speeds.items():
            self._publish(component, speed)
        self.get_logger().info('Cleaning actuators started.')

    def stop(self):
        self.active = False
        for component in ('vacuum', 'main_brush', 'side_brush', 'water_pump'):
            self._publish(component, 0)


def main():
    rclpy.init()
    node = CleaningActuatorManager()
    successful = False
    try:
        while rclpy.ok() and not node.completion.completed:
            rclpy.spin_once(node, timeout_sec=0.2)
        successful = node.completion.success
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not successful:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
