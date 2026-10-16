#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool
from vacuum_ros2_bridge.msg import Bumper


def state_qos():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class BumperAdapter(Node):
    def __init__(self):
        super().__init__('s5max_bumper_adapter')
        self.left_publisher = self.create_publisher(
            Bool, '/bumper/left', state_qos()
        )
        self.right_publisher = self.create_publisher(
            Bool, '/bumper/right', state_qos()
        )
        self.create_subscription(
            Bumper, '/bumper', self.on_bumper, qos_profile_sensor_data
        )
        self.last_state = None

    def on_bumper(self, message):
        state = (bool(message.left), bool(message.right))
        self.left_publisher.publish(Bool(data=state[0]))
        self.right_publisher.publish(Bool(data=state[1]))
        if state != self.last_state:
            self.get_logger().info(
                f'Bumper state changed: left={state[0]} right={state[1]}'
            )
            self.last_state = state


def main():
    rclpy.init()
    node = BumperAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
