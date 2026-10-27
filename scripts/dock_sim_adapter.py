#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from std_msgs.msg import String
from vacuum_ros2_bridge.msg import VacuumStatus
from vacuum_ros2_bridge.srv import SetActuator


def wrap_angle(angle):
    return math.remainder(float(angle), 2.0 * math.pi)


def yaw_from_odometry(message):
    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y),
        1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z),
    )


def receiver_sees_beacon(
        x, y, yaw, beacon_x, beacon_y,
        receiver_x, receiver_y, receiver_yaw,
        half_fov, maximum_range):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    sensor_x = x + receiver_x * cosine - receiver_y * sine
    sensor_y = y + receiver_x * sine + receiver_y * cosine
    dx = float(beacon_x) - sensor_x
    dy = float(beacon_y) - sensor_y
    bearing = wrap_angle(
        math.atan2(dy, dx) - float(yaw) - float(receiver_yaw))
    return math.hypot(dx, dy) <= maximum_range and abs(bearing) <= half_fov


def simulated_signal(
        x, y, yaw, beacon_x, beacon_y,
        front_receiver_x=0.174, front_receiver_y=0.0,
        front_receiver_yaw=0.0, front_receiver_half_fov=0.26,
        right_receiver_x=0.124, right_receiver_y=-0.124,
        right_receiver_yaw=-math.pi / 4.0,
        right_receiver_half_fov=1.75, receiver_range=1.20):
    dx = float(beacon_x) - float(x)
    dy = float(beacon_y) - float(y)
    distance = math.hypot(dx, dy)
    sight_bearing = wrap_angle(math.atan2(dy, dx) - float(yaw))
    visible = distance <= receiver_range and abs(sight_bearing) <= 0.9
    front_visible = visible and receiver_sees_beacon(
        x, y, yaw, beacon_x, beacon_y,
        front_receiver_x, front_receiver_y, front_receiver_yaw,
        front_receiver_half_fov, receiver_range)
    right_visible = visible and receiver_sees_beacon(
        x, y, yaw, beacon_x, beacon_y,
        right_receiver_x, right_receiver_y, right_receiver_yaw,
        right_receiver_half_fov, receiver_range)
    if not front_visible and not right_visible:
        return 0, 0, None
    forward = dx
    lateral = dy
    heading_error = wrap_angle(-float(yaw))
    guidance = wrap_angle(
        heading_error + math.atan2(lateral, max(0.40, forward)))
    sector_angle = math.atan2(0.30, 0.60)
    if guidance > 0.05:
        return (
            0x1A if front_visible else 0,
            0x18 if right_visible else 0,
            sector_angle,
        )
    if guidance < -0.05:
        return (
            0x15 if front_visible else 0,
            0x15 if right_visible else 0,
            -sector_angle,
        )
    return (
        0x15 if front_visible else 0,
        0x18 if right_visible else 0,
        0.0,
    )


def simulated_contact(
        x, y, yaw, dock_center_x, lateral_tolerance,
        longitudinal_tolerance, yaw_tolerance):
    return (
        abs(float(x) - float(dock_center_x)) <= longitudinal_tolerance
        and abs(float(y)) <= lateral_tolerance
        and abs(wrap_angle(yaw)) <= yaw_tolerance
    )


def retain_simulated_contact(latched, contact, x, y, yaw, dock_center_x):
    if contact:
        return True
    if not latched:
        return False
    return (
        abs(float(x) - float(dock_center_x)) <= 0.060
        and abs(float(y)) <= 0.050
        and abs(wrap_angle(yaw)) <= 0.20
    )


def scenario_contact(scenario, retry_seen, dock_state, bumper, contact):
    if scenario != 'missed-contact':
        return contact
    if not retry_seen:
        return False
    return contact


def world_pose(x, y, yaw, origin_x, origin_y, origin_yaw):
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    return (
        origin_x + x * cosine - y * sine,
        origin_y + x * sine + y * cosine,
        wrap_angle(origin_yaw + yaw),
    )


class DockSimulationAdapter(Node):
    def __init__(self):
        super().__init__('s5max_dock_sim_adapter')
        self.declare_parameter('beacon_x', 0.123)
        self.declare_parameter('beacon_y', 0.0)
        self.declare_parameter('front_receiver_x', 0.174)
        self.declare_parameter('front_receiver_y', 0.0)
        self.declare_parameter('front_receiver_yaw', 0.0)
        self.declare_parameter('front_receiver_half_fov', 0.26)
        self.declare_parameter('right_receiver_x', 0.124)
        self.declare_parameter('right_receiver_y', -0.124)
        self.declare_parameter('right_receiver_yaw', -math.pi / 4.0)
        self.declare_parameter('right_receiver_half_fov', 1.75)
        self.declare_parameter('receiver_range', 1.20)
        self.declare_parameter('dock_center_x', -0.060)
        self.declare_parameter('body_radius', 0.1765)
        self.declare_parameter('dock_face_x', 0.124)
        self.declare_parameter('contact_lateral_tolerance', 0.015)
        self.declare_parameter('contact_longitudinal_tolerance', 0.015)
        self.declare_parameter('contact_yaw_tolerance', 0.05)
        self.declare_parameter('odom_origin_x', 0.0)
        self.declare_parameter('odom_origin_y', 0.0)
        self.declare_parameter('odom_origin_yaw', 0.0)
        self.declare_parameter('scenario', 'centered')
        self.declare_parameter('beacon_loss_duration_sec', 1.5)
        self.declare_parameter('charging_delay_sec', 3.0)
        self.pose = None
        self.charger_enabled = False
        self.charger_enabled_at = None
        self.dock_state = 'WAITING'
        self.beacon_loss_started_at = None
        self.retry_seen = False
        self.docked_latched = False
        self.last_bumper = False
        self.last_contact = False

        self.left_bumper_pub = self.create_publisher(
            Bool, '/bumper/left', 10)
        self.right_bumper_pub = self.create_publisher(
            Bool, '/bumper/right', 10)
        self.status_pub = self.create_publisher(
            VacuumStatus, '/vacuum_status', 10)
        self.command_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            String, '/dock_robot/state', self._on_dock_state, 10)
        self.create_service(SetActuator, '/set_actuator', self._set_actuator)
        self.create_timer(0.05, self._publish)

    def _parameter(self, name):
        return float(self.get_parameter(name).value)

    def _on_odom(self, message):
        self.pose = world_pose(
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            yaw_from_odometry(message),
            self._parameter('odom_origin_x'),
            self._parameter('odom_origin_y'),
            self._parameter('odom_origin_yaw'),
        )

    def _set_actuator(self, request, response):
        if request.component == 'charger':
            self.charger_enabled = request.speed > 0
            self.charger_enabled_at = (
                self._seconds() if self.charger_enabled else None)
        response.success = True
        response.message = 'simulation accepted the actuator command'
        return response

    def _seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _scenario(self):
        return str(self.get_parameter('scenario').value)

    def _on_dock_state(self, message):
        self.dock_state = str(message.data)
        if self.pose is None:
            self.get_logger().info(
                f'Dock simulation state: {self.dock_state} pose=unavailable')
        else:
            self.get_logger().info(
                'Dock simulation state: %s pose=(%.3f,%.3f,%.3f)' % (
                    self.dock_state,
                    self.pose[0],
                    self.pose[1],
                    self.pose[2],
                ))
        if self.dock_state == 'RETRY':
            self.retry_seen = True
        if (
                self._scenario() == 'beacon-loss'
                and self.dock_state == 'CONTROLLING'
                and self.beacon_loss_started_at is None):
            self.beacon_loss_started_at = self._seconds()
            self.get_logger().info(
                'The beacon-loss scenario started its temporary outage.')

    def _publish(self):
        if self.pose is None:
            self.command_pub.publish(Twist())
            return
        x, y, yaw = self.pose
        now = self.get_clock().now().to_msg()
        front_mask, right_mask, _ = simulated_signal(
            x,
            y,
            yaw,
            self._parameter('beacon_x'),
            self._parameter('beacon_y'),
            self._parameter('front_receiver_x'),
            self._parameter('front_receiver_y'),
            self._parameter('front_receiver_yaw'),
            self._parameter('front_receiver_half_fov'),
            self._parameter('right_receiver_x'),
            self._parameter('right_receiver_y'),
            self._parameter('right_receiver_yaw'),
            self._parameter('right_receiver_half_fov'),
            self._parameter('receiver_range'),
        )
        if (
                self.beacon_loss_started_at is not None
                and self._seconds() - self.beacon_loss_started_at
                < self._parameter('beacon_loss_duration_sec')):
            front_mask, right_mask = 0, 0

        bumper = (
            x + self._parameter('body_radius')
            >= self._parameter('dock_face_x') - 0.004
            and abs(y) <= 0.25
        )
        self.left_bumper_pub.publish(Bool(data=bumper))
        self.right_bumper_pub.publish(Bool(data=bumper))
        if bumper != self.last_bumper:
            self.get_logger().info(
                'Simulated bumper changed to %s at pose=(%.3f,%.3f,%.3f).' % (
                    bumper, x, y, yaw))
            self.last_bumper = bumper

        contact = simulated_contact(
            x,
            y,
            yaw,
            self._parameter('dock_center_x'),
            self._parameter('contact_lateral_tolerance'),
            self._parameter('contact_longitudinal_tolerance'),
            self._parameter('contact_yaw_tolerance'),
        )
        contact = scenario_contact(
            self._scenario(),
            self.retry_seen,
            self.dock_state,
            bumper,
            contact,
        )
        if contact != self.last_contact:
            self.get_logger().info(
                'Simulated charging contact changed to %s at '
                'pose=(%.3f,%.3f,%.3f).' % (contact, x, y, yaw))
            self.last_contact = contact
        self.docked_latched = retain_simulated_contact(
            self.docked_latched,
            contact,
            x,
            y,
            yaw,
            self._parameter('dock_center_x'),
        )
        docked = self.docked_latched
        status = VacuumStatus()
        status.header.stamp = now
        status.header.frame_id = 'base_footprint'
        status.battery_voltage = 14.4
        status.battery_percent = 75
        status.is_docked = docked
        status.dock_ir_front_mask = front_mask
        status.dock_ir_right_mask = right_mask
        charging_delay = (
            self._parameter('charging_delay_sec')
            if self._scenario() == 'delayed-charging' else 0.0)
        charger_ready = (
            self.charger_enabled_at is not None
            and self._seconds() - self.charger_enabled_at >= charging_delay)
        status.is_charging = docked and self.charger_enabled and charger_ready
        status.dustbox_attached = True
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = DockSimulationAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
