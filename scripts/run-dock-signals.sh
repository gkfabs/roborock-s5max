#!/usr/bin/env bash
set -euo pipefail

ROBOT="${ROBOT:-root@192.168.4.55}"
ROBOT_IP="${ROBOT_IP:-192.168.4.55}"
ROS_IMAGE="${ROS_IMAGE:-oomwoo_one:latest}"
ROS_WORKSPACE="${ROS_WORKSPACE:-/tmp/s5_ros_ws}"
BRIDGE_REPO="${BRIDGE_REPO:?BRIDGE_REPO is required}"
ROBOT_DESCRIPTION_REPO="${ROBOT_DESCRIPTION_REPO:?ROBOT_DESCRIPTION_REPO is required}"
DEMO_SECONDS="${DEMO_SECONDS:-1800}"
UNDOCK="${UNDOCK:-0}"
REDOCK="${REDOCK:-0}"
REDOCK_MAX_DISTANCE="${REDOCK_MAX_DISTANCE:-0.20}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER=s5max-dock-signals

restore_valetudo()
{
    docker stop --timeout 3 "$CONTAINER" >/dev/null 2>&1 || true
    ROBOT="$ROBOT" "$SCRIPT_DIR/robot-mode.sh" valetudo || true
}

if [ "$REDOCK" = 1 ]; then
    echo "This test uses a bounded automatic undock and final dock approach."
    echo "Keep at least 35 cm of clear floor behind the robot."
elif [ "$UNDOCK" = 1 ]; then
    echo "This test uses the bounded automatic undock and starts the LiDAR."
    echo "Keep at least 35 cm of clear floor behind the robot."
else
    echo "This test does not start the wheels, cleaning motors, or LiDAR."
fi
echo "Keep the robot still while you record each dock position."

trap restore_valetudo EXIT INT TERM

ROBOT="$ROBOT" DEMO_SECONDS="$DEMO_SECONDS" \
    "$SCRIPT_DIR/robot-mode.sh" sangamio

echo "Starting the passive dock infrared monitor."
echo "The monitor will read one sample and restore Valetudo."

docker run --rm --name "$CONTAINER" --network host \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -e ROBOT_IP="$ROBOT_IP" \
    -e UNDOCK="$UNDOCK" \
    -e REDOCK="$REDOCK" \
    -e REDOCK_MAX_DISTANCE="$REDOCK_MAX_DISTANCE" \
    -v "$BRIDGE_REPO:/ws/src/vacuum_ros2_bridge:ro" \
    -v "$ROBOT_DESCRIPTION_REPO:/ws/src/roborock_s5max:ro" \
    -v "$ROS_WORKSPACE/build:/ws/build" \
    -v "$ROS_WORKSPACE/install:/ws/install:ro" \
    -v "$ROS_WORKSPACE/log:/ws/log" \
    -v "$SCRIPT_DIR/run-ros-dock-signals.sh:/tmp/run-ros-dock-signals.sh:ro" \
    -w /ws "$ROS_IMAGE" bash /tmp/run-ros-dock-signals.sh
