#!/usr/bin/env python3

import copy
import math
import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def lidar_noise_parameters(profile, linear_speed, angular_speed):
    motion = min(
        1.0,
        abs(float(linear_speed)) / 0.06
        + abs(float(angular_speed)) / 0.40,
    )
    if profile == 'clean':
        return 0.0, 0.0, 0.0, 0.0, 0.0
    if profile == 'realistic':
        return (
            0.004 + 0.012 * motion,
            0.002 + 0.008 * motion,
            0.003 + 0.015 * motion,
            0.002 + 0.010 * motion,
            0.010 + 0.040 * motion,
        )
    if profile == 'stress':
        return (
            0.012 + 0.030 * motion,
            0.006 + 0.020 * motion,
            0.030 + 0.080 * motion,
            0.020 + 0.050 * motion,
            0.050 + 0.150 * motion,
        )
    raise ValueError(f'Unknown LiDAR noise profile: {profile}')


def apply_lidar_noise(
        ranges, range_min, range_max, profile, linear_speed, angular_speed,
        seed, scan_index):
    noise, bias_noise, missing, outlier, scan_loss = lidar_noise_parameters(
        profile, linear_speed, angular_speed)
    if profile == 'clean':
        return list(ranges)
    generator = random.Random(int(seed) + int(scan_index) * 104729)
    if generator.random() < scan_loss:
        return None
    scan_bias = generator.gauss(0.0, bias_noise)
    output = []
    for raw_range in ranges:
        distance = float(raw_range)
        if not math.isfinite(distance):
            output.append(distance)
            continue
        if generator.random() < missing:
            output.append(math.inf)
            continue
        value = distance + scan_bias + generator.gauss(0.0, noise)
        if generator.random() < outlier:
            value += generator.uniform(-0.20, 0.20)
        output.append(max(float(range_min), min(float(range_max), value)))
    return output


def apply_front_sector_outage(
        ranges, angle_min, angle_increment, active, sector_angle=1.0):
    output = list(ranges)
    if not active:
        return output
    for index in range(len(output)):
        raw_angle = float(angle_min) + index * float(angle_increment)
        angle = math.atan2(math.sin(raw_angle), math.cos(raw_angle))
        if abs(angle) <= abs(float(sector_angle)):
            output[index] = math.inf
    return output


def outage_is_active(now, started_at, duration):
    return (
        started_at is not None
        and float(now) >= float(started_at)
        and float(now) < float(started_at) + float(duration)
    )


class DockScanFaultAdapter(Node):
    def __init__(self):
        super().__init__('s5max_dock_scan_fault_adapter')
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('noise_profile', 'realistic')
        self.declare_parameter('noise_seed', 5237)
        self.declare_parameter('outage_start_sec', 0.0)
        self.declare_parameter('outage_duration_sec', 0.0)
        self.declare_parameter('outage_state', '')
        self.declare_parameter('outage_state_delay_sec', 0.25)
        self.command = Twist()
        self.first_scan_at = None
        self.state_outage_at = None
        self.state_outage_triggered = False
        self.scan_index = 0
        self.profile = str(self.get_parameter('noise_profile').value)
        lidar_noise_parameters(self.profile, 0.0, 0.0)
        self.publisher = self.create_publisher(
            LaserScan, str(self.get_parameter('output_topic').value),
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, str(self.get_parameter('input_topic').value),
            self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self._on_command, 10)
        self.create_subscription(
            String, '/dock_robot/state', self._on_state, 10)

    def _seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_command(self, message):
        self.command = message

    def _on_state(self, message):
        target = str(self.get_parameter('outage_state').value).upper()
        if not target or self.state_outage_triggered:
            return
        if str(message.data).upper() != target:
            return
        self.state_outage_triggered = True
        self.state_outage_at = (
            self._seconds()
            + float(self.get_parameter('outage_state_delay_sec').value))
        self.get_logger().info(
            'The LiDAR geometry outage will start during %s.' % target)

    def _on_scan(self, message):
        now = self._seconds()
        if self.first_scan_at is None:
            self.first_scan_at = now
        self.scan_index += 1
        ranges = apply_lidar_noise(
            message.ranges,
            message.range_min,
            message.range_max,
            self.profile,
            self.command.linear.x,
            self.command.angular.z,
            int(self.get_parameter('noise_seed').value),
            self.scan_index,
        )
        if ranges is None:
            return
        duration = float(self.get_parameter('outage_duration_sec').value)
        timed_start = float(self.get_parameter('outage_start_sec').value)
        timed_active = (
            duration > 0.0
            and not str(self.get_parameter('outage_state').value)
            and timed_start <= now - self.first_scan_at < timed_start + duration)
        state_active = outage_is_active(
            now, self.state_outage_at, duration)
        ranges = apply_front_sector_outage(
            ranges, message.angle_min, message.angle_increment,
            timed_active or state_active)
        output = copy.deepcopy(message)
        output.ranges = ranges
        output.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = DockScanFaultAdapter()
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
