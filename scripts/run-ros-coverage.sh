#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

set -u

launch_pid=""
planner_pid=""
actuator_pid=""
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
    wait "$pid" 2>/dev/null || true
}

cleanup()
{
    trap - EXIT INT TERM

    stop_process "$planner_pid"
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

    stop_process "$actuator_pid"
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
    start_pose_path:="/maps/${MAP_NAME}.dock.json" &
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

timeout 8 ros2 service call /set_lidar vacuum_ros2_bridge/srv/SetLidar \
    '{enable: true}' || true

echo "Waiting for LiDAR, odometry, and localization..."
if ! timeout 15 ros2 topic echo --once /scan >/dev/null 2>&1; then
    echo "ERROR: Automatic undocking or LiDAR startup did not produce a scan." >&2
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

rviz2 -d /ws/install/roborock_s5max/share/roborock_s5max/rviz/navigation.rviz &
rviz_pid=$!

if [ "$ENABLE_JOYSTICK" = true ]; then
    ros2 run joy joy_node --ros-args \
        -p device_id:=0 -p deadzone:=0.0 -p autorepeat_rate:=20.0 &
    joy_pid=$!
    python3 /tmp/joystick_pilot.py &
    pilot_pid=$!
    echo "Hold joystick button 3 for manual override."
fi

echo "Use 2D Pose Estimate if the robot does not align with the saved map."
echo "Inspect the LiDAR alignment before autonomous movement starts."
if ! read -r -p "Type 'clean' to start coverage: " confirmation; then
    echo "ERROR: No coverage confirmation was received." >&2
    exit 1
fi
if [ "$confirmation" != "clean" ]; then
    echo "The coverage run was canceled."
    exit 0
fi
if ! timeout 10 ros2 topic echo --once /amcl_pose >/dev/null 2>&1; then
    echo "ERROR: No localized robot pose is available." >&2
    exit 1
fi

ros2 run roborock_s5max cleaning_actuator_manager.py --ros-args \
    --params-file /ws/install/roborock_s5max/share/roborock_s5max/config/coverage.yaml &
actuator_pid=$!

if ! timeout 10 ros2 service call /s5max_cleaning_actuators/start \
        std_srvs/srv/Trigger '{}' | grep -q 'success=True'; then
    echo "ERROR: The cleaning actuators did not start." >&2
    exit 1
fi

ros2 run oomwoo_coverage coverage_planner --ros-args \
    --params-file /ws/install/roborock_s5max/share/roborock_s5max/config/coverage.yaml \
    -r cmd_vel:=/cmd_vel_clean \
    -r map:=/map \
    -r navigate_to_pose:=/navigate_to_pose &
planner_pid=$!

echo "Coverage started. Contact recovery can override and release the route."
echo "Successful coverage will return the robot to its mapped dock."
echo "Close RViz or press Ctrl-C to stop without an automatic dock return."

completed_pid=""
session_status=0
wait -n -p completed_pid \
    "$launch_pid" "$planner_pid" "$actuator_pid" "$rviz_pid" || \
    session_status=$?
if [ "$completed_pid" != "$actuator_pid" ]; then
    echo "Coverage did not report successful completion. Dock return is disabled." >&2
    if [ "$session_status" -eq 0 ]; then
        session_status=1
    fi
elif [ "$session_status" -ne 0 ]; then
    echo "A coverage process stopped with status $session_status." >&2
else
    echo "Coverage completed. Stopping cleaning before the dock return."
    stop_process "$planner_pid"
    planner_pid=""
    stop_process "$pilot_pid"
    pilot_pid=""
    stop_process "$joy_pid"
    joy_pid=""
    stop_process "$actuator_pid"
    actuator_pid=""

    echo "Starting the Nav2 dock action."
    ros2 run roborock_s5max dock_robot.py \
        --path "/maps/${MAP_NAME}.dock.json" --map-name "$MAP_NAME" \
        --server-timeout 30 --staging-timeout 300 --operation-timeout 420
    echo "Coverage finished. The robot is docked and charging."
fi
exit "$session_status"
