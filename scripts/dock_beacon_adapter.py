#!/usr/bin/env python3

import math
from typing import NamedTuple

import rclpy
from oomwoo_msgs.msg import DockSignal
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from vacuum_ros2_bridge.msg import DockIr


BEACON_BITS = 0x1F
FAR_LEFT_BIT = 0x01
FAR_RIGHT_BIT = 0x02
CLOSE_LEFT_BIT = 0x04
CLOSE_RIGHT_BIT = 0x08
CENTER_BIT = 0x10
DEFAULT_SECTOR_ANGLE_RAD = math.atan2(0.30, 0.60)


class DockSignalObservation(NamedTuple):
    left_mask: int
    right_mask: int
    visible: bool
    bearing: float | None
    confidence: float
    far_left: bool
    far_right: bool
    close_left: bool
    close_right: bool
    center: bool


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


def decode_signal(left_mask, right_mask, sector_angle_rad):
    """Decode the complete S5 Max dock signal into a shared observation."""
    left_mask = int(left_mask) & BEACON_BITS
    right_mask = int(right_mask) & BEACON_BITS
    combined = left_mask | right_mask
    visible, bearing = decode_beacon(
        left_mask, right_mask, sector_angle_rad)
    active_receivers = int(bool(left_mask)) + int(bool(right_mask))
    return DockSignalObservation(
        left_mask=left_mask,
        right_mask=right_mask,
        visible=visible,
        bearing=bearing,
        confidence=active_receivers / 2.0,
        far_left=bool(combined & FAR_LEFT_BIT),
        far_right=bool(combined & FAR_RIGHT_BIT),
        close_left=bool(combined & CLOSE_LEFT_BIT),
        close_right=bool(combined & CLOSE_RIGHT_BIT),
        center=bool(combined & CENTER_BIT),
    )


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
        self.signal_pub = self.create_publisher(
            DockSignal, '/dock_signal', 10)
        self.subscription = self.create_subscription(
            DockIr, '/dock_ir', self._on_dock_ir, 10)

    def _on_dock_ir(self, msg):
        signal = decode_signal(
            msg.left_mask, msg.right_mask, self.sector_angle_rad)
        self.visible_pub.publish(Bool(data=signal.visible))
        if signal.bearing is not None:
            self.bearing_pub.publish(Float32(data=float(signal.bearing)))

        output = DockSignal()
        output.header = msg.header
        output.visible = signal.visible
        output.bearing_rad = (
            float(signal.bearing) if signal.bearing is not None else math.nan)
        output.confidence = signal.confidence
        output.left_mask = signal.left_mask
        output.right_mask = signal.right_mask
        output.far_left = signal.far_left
        output.far_right = signal.far_right
        output.close_left = signal.close_left
        output.close_right = signal.close_right
        output.center = signal.center
        self.signal_pub.publish(output)


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
