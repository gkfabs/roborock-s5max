#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

set -u

launch_pid=""
joy_pid=""
pilot_pid=""
rviz_pid=""
safety_stop_pid=""

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

    timeout 8 ros2 topic pub -r 20 /cmd_vel_safety \
        geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 &
    safety_stop_pid=$!

    for component in vacuum main_brush side_brush water_pump; do
        timeout 3 ros2 topic pub --once /actuator_cmd \
            vacuum_ros2_bridge/msg/ActuatorCommand \
            "{component: \"$component\", speed: 0}" >/dev/null 2>&1 || true
    done
    timeout 5 ros2 service call /set_lidar vacuum_ros2_bridge/srv/SetLidar \
        '{enable: false}' >/dev/null 2>&1 || true

    stop_process "$rviz_pid"
    stop_process "$launch_pid"
    wait "$safety_stop_pid" 2>/dev/null || true
}

stop_on_signal()
{
    exit 0
}

trap cleanup EXIT
trap stop_on_signal INT TERM

ros2 launch roborock_s5max navigation.launch.py \
    robot_ip:="$ROBOT_IP" \
    posegraph_path:="/maps/${MAP_NAME}_serial" &
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

echo "Waiting for LiDAR, odometry, localization, and Nav2..."
if ! timeout 15 ros2 topic echo --once /scan >/dev/null 2>&1; then
    echo "ERROR: No LiDAR scan arrived. Physically undock the robot and retry." >&2
    exit 1
fi
if ! timeout 5 ros2 topic echo --once /odom >/dev/null 2>&1; then
    echo "ERROR: No odometry data arrived." >&2
    exit 1
fi
if ! timeout 30 ros2 topic echo --once /map >/dev/null 2>&1; then
    echo "ERROR: SLAM Toolbox did not publish the saved map." >&2
    exit 1
fi

nav2_ready=false
for _ in $(seq 1 30); do
    if ros2 lifecycle get /controller_server 2>/dev/null | \
            grep -q 'active'; then
        nav2_ready=true
        break
    fi
    if ! kill -0 "$launch_pid" 2>/dev/null; then
        wait "$launch_pid"
    fi
    sleep 1
done
if [ "$nav2_ready" != true ]; then
    echo "ERROR: Nav2 did not become active." >&2
    exit 1
fi

if [ "$ENABLE_JOYSTICK" = true ]; then
    ros2 run joy joy_node --ros-args \
        -p device_id:=0 \
        -p deadzone:=0.0 \
        -p autorepeat_rate:=20.0 &
    joy_pid=$!
    python3 /tmp/joystick_pilot.py &
    pilot_pid=$!
    echo "Hold joystick button 3 for manual override."
fi

rviz2 -d /ws/install/roborock_s5max/share/roborock_s5max/rviz/navigation.rviz &
rviz_pid=$!

echo "Navigation is ready. No movement command was sent."
echo "Make sure that the LiDAR scan aligns with the saved map."
echo "If the pose is wrong, use 2D Pose Estimate before setting a goal."
echo "Use Nav2 Goal only after localization is correct."
echo "Close RViz or press Ctrl-C to stop and restore Valetudo."

session_status=0
wait -n "$launch_pid" "$rviz_pid" || session_status=$?
if [ "$session_status" -ne 0 ]; then
    echo "A navigation process stopped with status $session_status." >&2
fi
exit "$session_status"
