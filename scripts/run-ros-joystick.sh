#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

cleanup()
{
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
    for component in vacuum main_brush side_brush; do
        ros2 topic pub --once /actuator_cmd vacuum_ros2_bridge/msg/ActuatorCommand \
            "{component: \"$component\", speed: 0}" >/dev/null 2>&1 || true
    done
    kill -TERM "$pilot_pid" "$joy_pid" "$bridge_pid" 2>/dev/null || true
    wait "$pilot_pid" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

ros2 run vacuum_ros2_bridge bridge_node.py --ros-args -p robot_ip:="$ROBOT_IP" &
bridge_pid=$!
sleep 3
ros2 run joy joy_node --ros-args \
    -p device_id:=0 \
    -p deadzone:=0.0 \
    -p autorepeat_rate:=20.0 &
joy_pid=$!
python3 /tmp/joystick_pilot.py &
pilot_pid=$!
wait "$pilot_pid"
