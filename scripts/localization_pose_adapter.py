#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


def latched_qos():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class LocalizationPoseAdapter(Node):
    def __init__(self):
        super().__init__('s5max_localization_pose_adapter')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', latched_qos()
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.on_pose, 10
        )

    def on_pose(self, message):
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = LocalizationPoseAdapter()
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
