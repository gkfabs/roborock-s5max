#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

set -u

launch_pid=""
rviz_pid=""
safety_stop_pid=""
estimate_recorder_pid=""
docked_success=false
pending_path="/maps/.${MAP_NAME}.dock.json.pending"
stage_path="/maps/${MAP_NAME}.dock.json"

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
    stop_process "$estimate_recorder_pid"
    rm -f "$pending_path"
    if [ "$DOCK_MODE" = calibrate ] && [ "$docked_success" != true ]; then
        echo "Calibration stopped. Trying the bounded recovery redock." >&2
        timeout 20 ros2 run roborock_s5max dock_final_approach.py || true
    fi
    timeout 8 ros2 topic pub -r 20 /cmd_vel_safety \
        geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 &
    safety_stop_pid=$!
    timeout 5 ros2 service call /set_lidar vacuum_ros2_bridge/srv/SetLidar \
        '{enable: false}' >/dev/null 2>&1 || true
    stop_process "$rviz_pid"
    stop_process "$launch_pid"
    wait "$safety_stop_pid" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

ros2 launch roborock_s5max navigation.launch.py \
    robot_ip:="$ROBOT_IP" \
    posegraph_path:="/maps/${MAP_NAME}_serial" \
    start_pose_path:="$stage_path" &
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
    timeout 10 ros2 service call /set_lidar vacuum_ros2_bridge/srv/SetLidar \
        '{enable: true}' 2>&1
)" || lidar_status=$?
echo "$lidar_response"
if [ "${lidar_status:-0}" -ne 0 ]; then
    echo "ERROR: The automatic undock or LiDAR command failed." >&2
    exit 1
fi

echo "Waiting for LiDAR, localization, and Nav2."
if ! timeout 15 ros2 topic echo --once /scan >/dev/null 2>&1; then
    echo "No LiDAR scan arrived. Restarting the LiDAR once." >&2
    timeout 5 ros2 service call /set_lidar vacuum_ros2_bridge/srv/SetLidar \
        '{enable: false}' >/dev/null 2>&1 || true
    sleep 1
    timeout 10 ros2 service call /set_lidar vacuum_ros2_bridge/srv/SetLidar \
        '{enable: true}' >/dev/null
    if ! timeout 15 ros2 topic echo --once /scan >/dev/null 2>&1; then
        echo "ERROR: No LiDAR scan arrived after the retry." >&2
        exit 1
    fi
fi
if ! timeout 30 ros2 topic echo --once /map >/dev/null 2>&1; then
    echo "ERROR: SLAM Toolbox did not publish the saved map." >&2
    exit 1
fi
if ! timeout 15 ros2 topic echo --once /amcl_pose >/dev/null 2>&1; then
    echo "ERROR: No localized robot pose is available." >&2
    exit 1
fi

nav2_ready=false
for _ in $(seq 1 30); do
    if ros2 lifecycle get /controller_server 2>/dev/null | grep -q 'active'; then
        nav2_ready=true
        break
    fi
    sleep 1
done
if [ "$nav2_ready" != true ]; then
    echo "ERROR: Nav2 did not become active." >&2
    exit 1
fi

rviz2 -d /ws/install/roborock_s5max/share/roborock_s5max/rviz/navigation.rviz &
rviz_pid=$!
echo "Make sure that the LiDAR scan aligns with the saved map."
echo "If the pose is wrong, use 2D Pose Estimate."

if [ "$DOCK_MODE" = calibrate ]; then
    ros2 run roborock_s5max dock_stage.py watch-estimates \
        --path "$pending_path" --map-name "$MAP_NAME" &
    estimate_recorder_pid=$!
    if ! read -r -p "Type 'record' to save this dock staging pose: " confirmation; then
        echo "ERROR: No dock calibration confirmation was received." >&2
        exit 1
    fi
    if [ "$confirmation" != record ]; then
        echo "The dock calibration was canceled."
        exit 0
    fi
    stop_process "$estimate_recorder_pid"
    estimate_recorder_pid=""
    if [ ! -s "$pending_path" ]; then
        echo "ERROR: No RViz pose estimate was recorded." >&2
        echo "Use 2D Pose Estimate before you type 'record'." >&2
        exit 1
    fi
else
    if ! read -r -p "Type 'dock' to start the return: " confirmation; then
        echo "ERROR: No docking confirmation was received." >&2
        exit 1
    fi
    if [ "$confirmation" != dock ]; then
        echo "The return to dock was canceled."
        exit 0
    fi
    ros2 run roborock_s5max dock_stage.py navigate \
        --path "$stage_path" --map-name "$MAP_NAME" \
        --timeout 30 --travel-timeout 300 --near-distance 0.30 \
        --approach-distance 0.20
    sleep 1
fi

echo "Starting the beacon-guided final dock approach."
ros2 run roborock_s5max dock_final_approach.py
docked_success=true

if [ "$DOCK_MODE" = calibrate ]; then
    mv "$pending_path" "$stage_path"
    echo "Saved dock staging pose: $stage_path"
fi

echo "The robot is docked and charging."
