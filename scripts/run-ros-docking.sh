#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

set -u

launch_pid=""
rviz_pid=""
safety_stop_pid=""
estimate_recorder_pid=""
pending_path="/maps/.${MAP_NAME}.dock.json.pending"
stage_path="/maps/${MAP_NAME}.dock.json"
dock_database_path=""

set_lidar()
{
    local enabled="$1"
    local response
    local status=0

    response="$(
        timeout 30 ros2 service call /set_lidar \
            vacuum_ros2_bridge/srv/SetLidar "{enable: $enabled}" 2>&1
    )" || status=$?
    printf '%s\n' "$response"
    if [ "$status" -ne 0 ]; then
        return "$status"
    fi
    printf '%s\n' "$response" | grep -Eq \
        'success[=:][[:space:]]*(True|true)'
}

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

if [ "$DOCK_MODE" = return ]; then
    dock_database_path="/tmp/${MAP_NAME}.dock-database.yaml"
    ros2 run roborock_s5max generate_dock_database.py \
        --input "$stage_path" \
        --map-name "$MAP_NAME" \
        --output "$dock_database_path"
fi

ros2 launch roborock_s5max navigation.launch.py \
    robot_ip:="$ROBOT_IP" \
    posegraph_path:="/maps/${MAP_NAME}_serial" \
    start_pose_path:="$stage_path" \
    dock_database_path:="$dock_database_path" \
    autostart:=false &
launch_pid=$!

rviz2 -d /ws/install/roborock_s5max/share/roborock_s5max/rviz/navigation.rviz &
rviz_pid=$!

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

echo "Starting the guarded undock and LiDAR sequence."
if ! set_lidar true; then
    echo "ERROR: The automatic undock or LiDAR command failed." >&2
    exit 1
fi

echo "Waiting for LiDAR and localization."
if ! timeout 15 ros2 topic echo --once /scan >/dev/null 2>&1; then
    echo "No LiDAR scan arrived. Restarting the LiDAR once." >&2
    timeout 5 ros2 service call /set_lidar vacuum_ros2_bridge/srv/SetLidar \
        '{enable: false}' >/dev/null 2>&1 || true
    sleep 1
    if ! set_lidar true; then
        echo "ERROR: The LiDAR restart command failed." >&2
        exit 1
    fi
    if ! timeout 15 ros2 topic echo --once /scan >/dev/null 2>&1; then
        echo "ERROR: No LiDAR scan arrived after the retry." >&2
        exit 1
    fi
fi
if ! timeout 10 ros2 topic echo --once /odom >/dev/null 2>&1; then
    echo "ERROR: No odometry data arrived." >&2
    exit 1
fi
if ! timeout 30 ros2 topic echo --once /map >/dev/null 2>&1; then
    echo "ERROR: SLAM Toolbox did not publish the saved map." >&2
    exit 1
fi
if ! timeout 15 ros2 topic echo --once /amcl_pose >/dev/null 2>&1; then
    echo "ERROR: No localized robot pose is available." >&2
    exit 1
fi

echo "Make sure that the LiDAR scan aligns with the saved map."
echo "If the pose is wrong, use 2D Pose Estimate."

echo "Starting Nav2 after localization became ready."
echo "This script owns lifecycle startup. Do not use RViz Startup."
if ! ros2 run roborock_s5max lifecycle_start.py \
        --manager lifecycle_manager_navigation \
        --node controller_server \
        --node smoother_server \
        --node planner_server \
        --node behavior_server \
        --node bt_navigator \
        --node collision_monitor \
        --timeout 30; then
    echo "ERROR: Nav2 did not enter the active lifecycle state." >&2
    exit 1
fi
if ! ros2 run roborock_s5max lifecycle_start.py \
        --manager lifecycle_manager_docking \
        --node docking_server \
        --timeout 30; then
    echo "ERROR: The docking server did not enter the active lifecycle state." >&2
    exit 1
fi
echo "The navigation and docking servers are active."

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
fi

if [ "$DOCK_MODE" = calibrate ]; then
    docking_path="$pending_path"
else
    docking_path="$stage_path"
fi

echo "Starting the Nav2 dock action."
ros2 run roborock_s5max dock_robot.py \
    --path "$docking_path" --map-name "$MAP_NAME" \
    --server-timeout 30 --staging-timeout 300 --operation-timeout 420
if [ "$DOCK_MODE" = calibrate ]; then
    mv "$pending_path" "$stage_path"
    echo "Saved dock staging pose: $stage_path"
fi

echo "The robot is docked and charging."
