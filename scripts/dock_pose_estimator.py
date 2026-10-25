#!/usr/bin/env python3

import math
import statistics
import time
from collections import deque
from typing import NamedTuple

import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from nav2_msgs.action import DockRobot
from oomwoo_msgs.msg import DockSignal
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class DockGeometry(NamedTuple):
    normal_bearing: float
    wall_distance: float
    dock_lateral: float
    wall_points: int
    dock_points: int


DETECTION_STATES = frozenset(('INITIAL_PERCEPTION', 'CONTROLLING'))
FEEDBACK_STATES = {
    DockRobot.Feedback.NAV_TO_STAGING_POSE: 'NAV_TO_STAGING_POSE',
    DockRobot.Feedback.INITIAL_PERCEPTION: 'INITIAL_PERCEPTION',
    DockRobot.Feedback.CONTROLLING: 'CONTROLLING',
    DockRobot.Feedback.WAIT_FOR_CHARGE: 'WAIT_FOR_CHARGE',
    DockRobot.Feedback.RETRY: 'RETRY',
}


def detection_is_enabled(state):
    return str(state) in DETECTION_STATES


def feedback_state_name(state):
    return FEEDBACK_STATES.get(int(state), f'UNKNOWN_{int(state)}')


def action_status_is_active(status_values):
    return any(status in (
        GoalStatus.STATUS_ACCEPTED,
        GoalStatus.STATUS_EXECUTING,
        GoalStatus.STATUS_CANCELING,
    ) for status in status_values)


def action_status_is_terminal(status_values):
    return any(status in (
        GoalStatus.STATUS_SUCCEEDED,
        GoalStatus.STATUS_CANCELED,
        GoalStatus.STATUS_ABORTED,
    ) for status in status_values)


def wrap_angle(angle):
    return math.remainder(float(angle), 2.0 * math.pi)


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z),
    )


def quaternion_from_yaw(yaw):
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


def percentile(values, fraction):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    index = int(round((len(ordered) - 1) * float(fraction)))
    return ordered[max(0, min(len(ordered) - 1, index))]


def select_dock_group(groups, beacon_bearing, side_deadband=0.015):
    def center(values):
        return (values[0] + values[-1]) / 2.0

    if beacon_bearing is None or abs(float(beacon_bearing)) < 0.10:
        return min(groups, key=lambda values: abs(center(values)))

    direction = 1.0 if float(beacon_bearing) > 0.0 else -1.0
    matching = [
        values for values in groups
        if direction * center(values) >= float(side_deadband)
    ]
    if matching:
        return min(matching, key=lambda values: abs(center(values)))
    return min(groups, key=lambda values: abs(center(values)))


def estimate_dock_geometry(
        ranges, angle_min, angle_increment,
        minimum_range=0.15, maximum_range=1.20,
        sector_angle=1.00, minimum_protrusion=0.012,
        maximum_protrusion=0.080, beacon_bearing=None):
    points = []
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        angle = wrap_angle(float(angle_min) + index * float(angle_increment))
        if not math.isfinite(distance):
            continue
        if distance < minimum_range or distance > maximum_range:
            continue
        if abs(angle) > sector_angle:
            continue
        points.append((distance * math.cos(angle), distance * math.sin(angle)))
    if len(points) < 12:
        return None

    best = None
    for angle_index in range(-80, 81):
        normal_angle = angle_index * 0.01
        normal_x = math.cos(normal_angle)
        normal_y = math.sin(normal_angle)
        bins = {}
        for point in points:
            distance = point[0] * normal_x + point[1] * normal_y
            if minimum_range <= distance <= maximum_range:
                index = int(round(distance / 0.01))
                bins[index] = bins.get(index, 0) + 1
        if bins:
            index, count = max(bins.items(), key=lambda item: item[1])
            if best is None or count > best[0]:
                best = count, normal_angle, index * 0.01
    if best is None or best[0] < 6:
        return None

    _, coarse_angle, coarse_distance = best
    coarse_x = math.cos(coarse_angle)
    coarse_y = math.sin(coarse_angle)
    wall_points = [
        point for point in points
        if abs(
            point[0] * coarse_x + point[1] * coarse_y - coarse_distance
        ) <= 0.012
    ]
    if len(wall_points) < 6:
        return None

    mean_x = statistics.fmean(point[0] for point in wall_points)
    mean_y = statistics.fmean(point[1] for point in wall_points)
    xx = sum((point[0] - mean_x) ** 2 for point in wall_points)
    yy = sum((point[1] - mean_y) ** 2 for point in wall_points)
    xy = sum(
        (point[0] - mean_x) * (point[1] - mean_y)
        for point in wall_points)
    tangent_angle = 0.5 * math.atan2(2.0 * xy, xx - yy)
    normal_x = -math.sin(tangent_angle)
    normal_y = math.cos(tangent_angle)
    if normal_x * mean_x + normal_y * mean_y < 0.0:
        normal_x = -normal_x
        normal_y = -normal_y
    tangent_x = -normal_y
    tangent_y = normal_x

    wall_distance = percentile([
        point[0] * normal_x + point[1] * normal_y
        for point in wall_points
    ], 0.50)
    dock_laterals = []
    for point in points:
        normal_distance = point[0] * normal_x + point[1] * normal_y
        protrusion = wall_distance - normal_distance
        if minimum_protrusion <= protrusion <= maximum_protrusion:
            dock_laterals.append(point[0] * tangent_x + point[1] * tangent_y)
    if len(dock_laterals) < 3:
        return None

    dock_laterals.sort()
    groups = [[dock_laterals[0]]]
    for lateral in dock_laterals[1:]:
        if lateral - groups[-1][-1] <= 0.035:
            groups[-1].append(lateral)
        else:
            groups.append([lateral])
    groups = [
        group for group in groups
        if len(group) >= 3 and 0.035 <= group[-1] - group[0] <= 0.25
    ]
    if not groups:
        return None
    group = select_dock_group(groups, beacon_bearing)
    return DockGeometry(
        normal_bearing=math.atan2(normal_y, normal_x),
        wall_distance=float(wall_distance),
        dock_lateral=(group[0] + group[-1]) / 2.0,
        wall_points=len(wall_points),
        dock_points=len(group),
    )


def target_in_robot(
        geometry, dock_protrusion, contact_offset,
        scan_translation_x=0.0, scan_translation_y=0.0,
        scan_rotation_yaw=0.0):
    normal = float(geometry.wall_distance) - float(dock_protrusion) - float(contact_offset)
    normal_x = math.cos(geometry.normal_bearing)
    normal_y = math.sin(geometry.normal_bearing)
    tangent_x = -normal_y
    tangent_y = normal_x
    target_in_scan = (
        normal * normal_x + geometry.dock_lateral * tangent_x,
        normal * normal_y + geometry.dock_lateral * tangent_y,
        geometry.normal_bearing,
    )
    return transform_pose(
        (
            float(scan_translation_x),
            float(scan_translation_y),
            float(scan_rotation_yaw),
        ),
        target_in_scan,
    )


def transform_pose(robot_pose, relative_pose):
    x, y, yaw = robot_pose
    relative_x, relative_y, relative_yaw = relative_pose
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        x + relative_x * cosine - relative_y * sine,
        y + relative_x * sine + relative_y * cosine,
        wrap_angle(yaw + relative_yaw),
    )


def mean_pose(poses):
    return (
        statistics.fmean(pose[0] for pose in poses),
        statistics.fmean(pose[1] for pose in poses),
        math.atan2(
            sum(math.sin(pose[2]) for pose in poses),
            sum(math.cos(pose[2]) for pose in poses),
        ),
    )


def pose_distance(first, second):
    return math.hypot(first[0] - second[0], first[1] - second[1])


class DockPoseEstimator(Node):
    def __init__(self):
        super().__init__('s5max_dock_pose_estimator')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pose_topic', '/detected_dock_pose')
        self.declare_parameter('dock_protrusion_m', 0.040)
        self.declare_parameter('contact_offset_m', 0.1765)
        self.declare_parameter('scan_translation_x_m', 0.060)
        self.declare_parameter('scan_translation_y_m', 0.0)
        self.declare_parameter('scan_rotation_yaw_rad', 0.0)
        self.declare_parameter('beacon_timeout_sec', 0.35)
        self.declare_parameter('odometry_timeout_sec', 0.25)
        self.declare_parameter('hold_timeout_sec', 10.0)
        self.declare_parameter('minimum_samples', 3)
        self.declare_parameter('sample_window', 5)
        self.declare_parameter('maximum_position_spread_m', 0.10)
        self.declare_parameter('maximum_yaw_spread_rad', 0.20)
        self.declare_parameter('maximum_pose_jump_m', 0.18)
        self.declare_parameter('maximum_beacon_error_rad', 0.75)
        self.declare_parameter('final_pose_lock_distance_m', 0.45)

        self.odom_pose = None
        self.odom_time = None
        self.signal = None
        self.signal_time = None
        self.samples = deque(maxlen=int(self.get_parameter('sample_window').value))
        self.accepted_pose = None
        self.accepted_at = None
        self.last_reason = None
        self.last_pose_log_at = None
        self.dock_state = 'WAITING'

        self.publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter('pose_topic').value), 10)
        self.create_subscription(
            Odometry, '/odom', self._on_odometry, qos_profile_sensor_data)
        self.create_subscription(
            DockSignal, '/dock_signal', self._on_signal, qos_profile_sensor_data)
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.inhibit_publisher = self.create_publisher(
            Bool, '/contact_reflex/inhibit', state_qos)
        self.create_subscription(
            String, '/dock_robot/state', self._on_dock_state, state_qos)
        self.create_subscription(
            DockRobot.Impl.FeedbackMessage,
            '/dock_robot/_action/feedback',
            self._on_action_feedback,
            10,
        )
        self.create_subscription(
            DockRobot.Impl.GoalStatusMessage,
            '/dock_robot/_action/status',
            self._on_action_status,
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_timer(0.10, self._publish_held_pose)

    def _seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _parameter(self, name):
        return float(self.get_parameter(name).value)

    def _reject(self, reason):
        if reason != self.last_reason:
            self.get_logger().info(f'Dock pose rejected: {reason}')
            self.last_reason = reason

    def _on_odometry(self, message):
        pose = message.pose.pose
        self.odom_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )
        self.odom_time = self._seconds()

    def _on_signal(self, message):
        self.signal = message
        self.signal_time = self._seconds()

    def _on_dock_state(self, message):
        self._set_dock_state(str(message.data))

    def _on_action_feedback(self, message):
        self._set_dock_state(feedback_state_name(message.feedback.state))

    def _on_action_status(self, message):
        status_values = [status.status for status in message.status_list]
        active = action_status_is_active(status_values)
        self.inhibit_publisher.publish(Bool(data=active))
        terminal = action_status_is_terminal(status_values)
        if terminal and not active:
            self._set_dock_state('WAITING')

    def _set_dock_state(self, next_state):
        if next_state == self.dock_state:
            return
        was_enabled = detection_is_enabled(self.dock_state)
        self.dock_state = next_state
        if next_state == 'WAITING':
            self.samples.clear()
            self.accepted_pose = None
            self.accepted_at = None
            self.last_reason = None
            return
        if was_enabled != detection_is_enabled(next_state):
            self.samples.clear()
            self.last_reason = None
            if detection_is_enabled(next_state) and self.accepted_pose is not None:
                self.accepted_at = self._seconds()

    def _on_scan(self, message):
        now = self._seconds()
        if not detection_is_enabled(self.dock_state):
            return
        if self.odom_pose is None or self.odom_time is None:
            self._reject('odometry is unavailable')
            return
        if now - self.odom_time > self._parameter('odometry_timeout_sec'):
            self._reject('odometry is stale')
            return
        if self.signal is None or self.signal_time is None:
            self._reject('the dock beacon is unavailable')
            return
        if now - self.signal_time > self._parameter('beacon_timeout_sec'):
            self._reject('the dock beacon is stale')
            return
        if not self.signal.visible or not math.isfinite(self.signal.bearing_rad):
            self._reject('the dock beacon is not visible')
            return

        if (
                self.accepted_pose is not None
                and pose_distance(self.odom_pose, self.accepted_pose)
                <= self._parameter('final_pose_lock_distance_m')):
            self.accepted_at = now
            self.last_reason = None
            self._publish(self.accepted_pose)
            return

        geometry = estimate_dock_geometry(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            beacon_bearing=self.signal.bearing_rad,
        )
        if geometry is None:
            self._reject('LiDAR dock geometry is unavailable')
            return
        relative = target_in_robot(
            geometry,
            self._parameter('dock_protrusion_m'),
            self._parameter('contact_offset_m'),
            self._parameter('scan_translation_x_m'),
            self._parameter('scan_translation_y_m'),
            self._parameter('scan_rotation_yaw_rad'),
        )
        target_bearing = math.atan2(relative[1], relative[0])
        beacon_error = abs(wrap_angle(target_bearing - self.signal.bearing_rad))
        if beacon_error > self._parameter('maximum_beacon_error_rad'):
            self._reject('LiDAR geometry conflicts with the dock beacon')
            return

        observation = transform_pose(self.odom_pose, relative)
        if self.accepted_pose is not None:
            jump = math.hypot(
                observation[0] - self.accepted_pose[0],
                observation[1] - self.accepted_pose[1],
            )
            if jump > self._parameter('maximum_pose_jump_m'):
                self.samples.clear()
                self._reject('the dock pose jumped outside its limit')
                return
        self.samples.append(observation)
        minimum_samples = int(self.get_parameter('minimum_samples').value)
        if len(self.samples) < minimum_samples:
            self._reject('the dock pose needs more supporting samples')
            return

        candidate = mean_pose(self.samples)
        position_spread = max(
            math.hypot(pose[0] - candidate[0], pose[1] - candidate[1])
            for pose in self.samples)
        yaw_spread = max(
            abs(wrap_angle(pose[2] - candidate[2])) for pose in self.samples)
        if position_spread > self._parameter('maximum_position_spread_m'):
            self._reject('the dock position is unstable')
            return
        if yaw_spread > self._parameter('maximum_yaw_spread_rad'):
            self._reject('the dock direction is unstable')
            return

        self.accepted_pose = candidate
        self.accepted_at = now
        self.last_reason = None
        if self.last_pose_log_at is None or now - self.last_pose_log_at >= 1.0:
            self.get_logger().info(
                'Dock pose accepted: target=(%.3f,%.3f,%.3f) '
                'geometry=(normal=%.3f wall=%.3f lateral=%.3f) '
                'points=(%d,%d) beacon=%.3f' % (
                    candidate[0],
                    candidate[1],
                    candidate[2],
                    geometry.normal_bearing,
                    geometry.wall_distance,
                    geometry.dock_lateral,
                    geometry.wall_points,
                    geometry.dock_points,
                    self.signal.bearing_rad,
                ))
            self.last_pose_log_at = now
        self._publish(candidate)

    def _publish(self, pose):
        message = PoseStamped()
        message.header.frame_id = 'odom'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = pose[0]
        message.pose.position.y = pose[1]
        qx, qy, qz, qw = quaternion_from_yaw(pose[2])
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        self.publisher.publish(message)

    def _publish_held_pose(self):
        if not detection_is_enabled(self.dock_state):
            return
        if self.accepted_pose is None or self.accepted_at is None:
            return
        if self._seconds() - self.accepted_at > self._parameter('hold_timeout_sec'):
            self.accepted_pose = None
            self.accepted_at = None
            self.samples.clear()
            return
        self._publish(self.accepted_pose)


def main(args=None):
    rclpy.init(args=args)
    node = DockPoseEstimator()
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
