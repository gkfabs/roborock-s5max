#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

set -u

launch_pid=""
joy_pid=""
pilot_pid=""
rviz_pid=""
autosave_pid=""
mapping_ready=false

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

save_map()
{
    if [ "$mapping_ready" != true ]; then
        return
    fi
    for attempt in 1 2 3; do
        if bash /tmp/save-map.sh; then
            return 0
        fi
        echo "Final map save attempt $attempt failed." >&2
        sleep 1
    done

    echo "Using the newest complete recovery snapshot." >&2
    MAP_DIRECTORY=/maps bash /tmp/promote-recovery-map.sh
}

remove_recovery_slot()
{
    local recovery_name="$1"
    rm -f \
        "/maps/${recovery_name}.yaml" \
        "/maps/${recovery_name}.pgm" \
        "/maps/${recovery_name}.png" \
        "/maps/${recovery_name}_serial.posegraph" \
        "/maps/${recovery_name}_serial.data"
}

save_recovery_loop()
{
    local slot=a
    local recovery_name

    while true; do
        recovery_name=".${MAP_NAME}.recovery-${slot}"
        remove_recovery_slot "$recovery_name"
        rm -f "/tmp/s5max-map-recovery-${slot}-complete"

        if SAVE_MAP_NAME="$recovery_name" \
                SAVE_COMPLETE_FILE="/tmp/s5max-map-recovery-${slot}-complete" \
                bash /tmp/save-map.sh; then
            printf '%s\n' "$recovery_name" \
                >"/maps/.${MAP_NAME}.recovery-latest.new"
            mv "/maps/.${MAP_NAME}.recovery-latest.new" \
                "/maps/.${MAP_NAME}.recovery-latest"
            echo "Recovery snapshot updated: $recovery_name"
            if [ "$slot" = a ]; then
                slot=b
            else
                slot=a
            fi
        else
            echo "WARNING: The recovery snapshot failed." >&2
        fi
        sleep 30
    done
}

cleanup()
{
    trap - EXIT INT TERM

    stop_process "$pilot_pid"
    stop_process "$joy_pid"
    stop_process "$autosave_pid"

    timeout 3 ros2 topic pub --once /cmd_vel_manual geometry_msgs/msg/Twist '{}' \
        >/dev/null 2>&1 || true
    timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' \
        >/dev/null 2>&1 || true

    if ! save_map; then
        echo "ERROR: The final map and recovery snapshot both failed." >&2
    fi

    for component in vacuum main_brush side_brush water_pump; do
        timeout 3 ros2 topic pub --once /actuator_cmd \
            vacuum_ros2_bridge/msg/ActuatorCommand \
            "{component: \"$component\", speed: 0}" >/dev/null 2>&1 || true
    done
    timeout 5 ros2 service call /set_lidar vacuum_ros2_bridge/srv/SetLidar \
        '{enable: false}' >/dev/null 2>&1 || true

    stop_process "$rviz_pid"
    stop_process "$launch_pid"
}

stop_on_signal()
{
    exit 0
}

trap cleanup EXIT
trap stop_on_signal INT TERM

ros2 launch roborock_s5max mapping.launch.py robot_ip:="$ROBOT_IP" &
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

echo "Waiting for LiDAR, odometry, and SLAM data..."
if ! timeout 15 ros2 topic echo --once /scan >/dev/null 2>&1; then
    echo "ERROR: No LiDAR scan arrived. Physically undock the robot and retry." >&2
    exit 1
fi
if ! timeout 5 ros2 topic echo --once /odom >/dev/null 2>&1; then
    echo "ERROR: No odometry data arrived." >&2
    exit 1
fi
if ! timeout 20 ros2 topic echo --once /map >/dev/null 2>&1; then
    echo "ERROR: SLAM Toolbox did not publish a map." >&2
    exit 1
fi
mapping_ready=true
save_recovery_loop &
autosave_pid=$!

ros2 run joy joy_node --ros-args \
    -p device_id:=0 \
    -p deadzone:=0.0 \
    -p autorepeat_rate:=20.0 &
joy_pid=$!
python3 /tmp/joystick_pilot.py &
pilot_pid=$!
rviz2 -d /ws/install/roborock_s5max/share/roborock_s5max/rviz/navigation.rviz &
rviz_pid=$!

echo "Mapping is ready. Drive slowly and revisit the starting area."
echo "Close RViz or press Ctrl-C to save the map and restore Valetudo."
session_status=0
wait -n "$launch_pid" "$joy_pid" "$pilot_pid" "$rviz_pid" || session_status=$?
if [ "$session_status" -ne 0 ]; then
    echo "A mapping process stopped with status $session_status. Saving the map." >&2
fi
exit "$session_status"
