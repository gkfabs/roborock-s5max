#!/usr/bin/env python3

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from vacuum_ros2_bridge.msg import DockIr


BEACON_BITS = 0x1F
DEFAULT_SECTOR_ANGLE_RAD = math.atan2(0.30, 0.60)


def decode_beacon(left_mask, right_mask, sector_angle_rad):
    """Return beacon visibility and its coarse bearing in robot axes."""
    left_active = bool(left_mask & BEACON_BITS)
    right_active = bool(right_mask & BEACON_BITS)
    if not left_active and not right_active:
        return False, None
    if left_active and right_active:
        return True, 0.0
    if left_active:
        return True, sector_angle_rad
    return True, -sector_angle_rad


class DockBeaconAdapter(Node):
    """Convert S5 Max dock receiver masks to OOMWOO beacon hints."""

    def __init__(self):
        super().__init__('s5max_dock_beacon_adapter')
        self.declare_parameter('sector_angle_rad', DEFAULT_SECTOR_ANGLE_RAD)
        self.sector_angle_rad = float(
            self.get_parameter('sector_angle_rad').value)
        self.visible_pub = self.create_publisher(
            Bool, '/dock_drive/beacon_visible', 10)
        self.bearing_pub = self.create_publisher(
            Float32, '/dock_detector/beacon_bearing', 10)
        self.subscription = self.create_subscription(
            DockIr, '/dock_ir', self._on_dock_ir, 10)

    def _on_dock_ir(self, msg):
        visible, bearing = decode_beacon(
            msg.left_mask, msg.right_mask, self.sector_angle_rad)
        self.visible_pub.publish(Bool(data=visible))
        if bearing is not None:
            self.bearing_pub.publish(Float32(data=float(bearing)))


def main(args=None):
    rclpy.init(args=args)
    node = DockBeaconAdapter()
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
