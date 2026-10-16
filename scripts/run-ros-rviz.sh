#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

set -u

launch_pid=""

cleanup()
{
    trap - EXIT INT TERM
    timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' \
        >/dev/null 2>&1 || true
    timeout 5 ros2 service call /set_lidar vacuum_ros2_bridge/srv/SetLidar \
        '{enable: false}' >/dev/null 2>&1 || true
    if [ -n "$launch_pid" ]; then
        kill -INT "$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
    fi
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

echo "Waiting for the first LiDAR scan..."
if ! timeout 15 ros2 topic echo --once /scan >/dev/null 2>&1; then
    echo "ERROR: Automatic undocking or LiDAR startup did not produce a scan." >&2
    exit 1
fi
echo "LiDAR is active. RViz is receiving /scan data."

wait "$launch_pid"
