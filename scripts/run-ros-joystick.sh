#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

set -u

launch_pid=""
joy_pid=""
pilot_pid=""

stop_process()
{
    local pid="$1"
    if [ -z "$pid" ] || ! kill -0 "$pid" 2>/dev/null; then
        return
    fi

    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 50); do
        if ! kill -0 "$pid" 2>/dev/null; then
            wait "$pid" 2>/dev/null || true
            return
        fi
        sleep 0.1
    done

    kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
        if ! kill -0 "$pid" 2>/dev/null; then
            wait "$pid" 2>/dev/null || true
            return
        fi
        sleep 0.1
    done

    kill -KILL "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
}

cleanup()
{
    trap - EXIT INT TERM

    stop_process "$pilot_pid"
    stop_process "$joy_pid"

    timeout 3 ros2 topic pub --once /cmd_vel_manual geometry_msgs/msg/Twist '{}' \
        >/dev/null 2>&1 || true
    timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' \
        >/dev/null 2>&1 || true
    for component in vacuum main_brush side_brush water_pump; do
        timeout 3 ros2 topic pub --once /actuator_cmd \
            vacuum_ros2_bridge/msg/ActuatorCommand \
            "{component: \"$component\", speed: 0}" >/dev/null 2>&1 || true
    done
    timeout 5 ros2 service call /set_lidar vacuum_ros2_bridge/srv/SetLidar \
        '{enable: false}' >/dev/null 2>&1 || true

    stop_process "$launch_pid"
}

trap cleanup EXIT INT TERM

ros2 launch roborock_s5max monitor_robot.launch.py \
    physical:=true robot_ip:="$ROBOT_IP" &
launch_pid=$!

for _ in $(seq 1 30); do
    if ros2 service type /set_lidar 2>/dev/null | \
            grep -q '^vacuum_ros2_bridge/srv/SetLidar$'; then
        break
    fi
    if ! kill -0 "$launch_pid" 2>/dev/null; then
        wait "$launch_pid"
    fi
    sleep 1
done

if ! ros2 service type /set_lidar 2>/dev/null | \
        grep -q '^vacuum_ros2_bridge/srv/SetLidar$'; then
    echo "ERROR: The ROS bridge did not provide the LiDAR service." >&2
    exit 1
fi

lidar_response="$(
    timeout 8 ros2 service call /set_lidar vacuum_ros2_bridge/srv/SetLidar \
        '{enable: true}' 2>&1
)" || lidar_status=$?
echo "$lidar_response"
if [ "${lidar_status:-0}" -ne 0 ] && [ "${lidar_status:-0}" -ne 124 ]; then
    echo "ERROR: The LiDAR command failed." >&2
    exit 1
fi

echo "Waiting for LiDAR and odometry data..."
if ! timeout 15 ros2 topic echo --once /scan >/dev/null 2>&1; then
    echo "ERROR: Automatic undocking or LiDAR startup did not produce a scan." >&2
    exit 1
fi
if ! timeout 5 ros2 topic echo --once /odom >/dev/null 2>&1; then
    echo "ERROR: No odometry data arrived." >&2
    exit 1
fi

ros2 run joy joy_node --ros-args \
    -p device_id:=0 \
    -p deadzone:=0.0 \
    -p autorepeat_rate:=20.0 &
joy_pid=$!
python3 /tmp/joystick_pilot.py &
pilot_pid=$!

echo "The demonstration is ready. Close RViz or press Ctrl-C to stop."
wait -n "$launch_pid" "$joy_pid" "$pilot_pid"
