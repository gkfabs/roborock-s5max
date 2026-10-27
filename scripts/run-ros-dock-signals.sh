#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

set -u

launch_pid=""
docking_pid=""
UNDOCK="${UNDOCK:-0}"
REDOCK="${REDOCK:-0}"

cleanup()
{
    trap - EXIT INT TERM
    if [ -n "$docking_pid" ]; then
        kill -INT "$docking_pid" 2>/dev/null || true
        wait "$docking_pid" 2>/dev/null || true
    fi
    if [ "$UNDOCK" = 1 ]; then
        timeout 5 ros2 service call /set_lidar vacuum_ros2_bridge/srv/SetLidar \
            '{enable: false}' >/dev/null 2>&1 || true
    fi
    if [ -n "$launch_pid" ]; then
        kill -INT "$launch_pid" 2>/dev/null || true
        for _ in $(seq 1 50); do
            if ! kill -0 "$launch_pid" 2>/dev/null; then
                wait "$launch_pid" 2>/dev/null || true
                return
            fi
            sleep 0.1
        done
        kill -TERM "$launch_pid" 2>/dev/null || true
        for _ in $(seq 1 50); do
            if ! kill -0 "$launch_pid" 2>/dev/null; then
                wait "$launch_pid" 2>/dev/null || true
                return
            fi
            sleep 0.1
        done
        kill -KILL "$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

ros2 launch roborock_s5max monitor_robot.launch.py \
    physical:=true show_rviz:=false robot_ip:="$ROBOT_IP" &
launch_pid=$!

for _ in $(seq 1 30); do
    if ros2 topic type /vacuum_status 2>/dev/null | \
            grep -q '^vacuum_ros2_bridge/msg/VacuumStatus$'; then
        break
    fi
    if ! kill -0 "$launch_pid" 2>/dev/null; then
        wait "$launch_pid"
    fi
    sleep 1
done

if ! ros2 topic type /vacuum_status 2>/dev/null | \
        grep -q '^vacuum_ros2_bridge/msg/VacuumStatus$'; then
    echo "ERROR: The ROS bridge did not provide the vacuum status topic." >&2
    exit 1
fi

if [ "$UNDOCK" = 1 ]; then
    for _ in $(seq 1 30); do
        if ros2 service type /set_lidar 2>/dev/null | \
                grep -q '^vacuum_ros2_bridge/srv/SetLidar$'; then
            break
        fi
        sleep 1
    done
    if ! ros2 service type /set_lidar 2>/dev/null | \
            grep -q '^vacuum_ros2_bridge/srv/SetLidar$'; then
        echo "ERROR: The ROS bridge did not provide the LiDAR service." >&2
        exit 1
    fi
    lidar_response="$(
        timeout 10 ros2 service call /set_lidar vacuum_ros2_bridge/srv/SetLidar \
            '{enable: true}' 2>&1
    )" || lidar_status=$?
    echo "$lidar_response"
    if [ "${lidar_status:-0}" -ne 0 ]; then
        echo "ERROR: The automatic undock or LiDAR command failed." >&2
        exit 1
    fi
    if ! timeout 15 ros2 topic echo --once /scan >/dev/null 2>&1; then
        echo "ERROR: The automatic undock did not produce a LiDAR scan." >&2
        exit 1
    fi
    sleep 1
fi

echo "Dock infrared data is active."
echo "The dock_ir mask values are hexadecimal receiver histories."
echo "A nonzero value means that the receiver detected the dock beacon."

if ! timeout 15 ros2 topic echo --once /vacuum_status; then
    echo "ERROR: SangamIO did not publish a dock infrared sample." >&2
    exit 1
fi

if [ "$REDOCK" = 1 ]; then
    echo "Starting the Nav2 dock action."
    ros2 launch roborock_s5max docking_server.launch.py &
    docking_pid=$!
    ros2 run roborock_s5max dock_robot.py \
        --dock-x 0.0 --dock-y 0.0 --dock-yaw 0.0 \
        --dock-frame odom --skip-staging \
        --server-timeout 20 --operation-timeout 45
fi
