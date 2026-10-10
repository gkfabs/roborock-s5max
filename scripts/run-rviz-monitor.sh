#!/usr/bin/env bash
set -euo pipefail

ROBOT="${ROBOT:-root@192.168.4.55}"
ROBOT_IP="${ROBOT_IP:-192.168.4.55}"
ROS_IMAGE="${ROS_IMAGE:-oomwoo_one:latest}"
ROS_WORKSPACE="${ROS_WORKSPACE:-/tmp/s5_ros_ws}"
BRIDGE_REPO="${BRIDGE_REPO:?BRIDGE_REPO is required}"
ROBOT_DESCRIPTION_REPO="${ROBOT_DESCRIPTION_REPO:?ROBOT_DESCRIPTION_REPO is required}"
DEMO_SECONDS="${DEMO_SECONDS:-1800}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER=s5max-rviz-monitor

restore_valetudo()
{
    docker stop --timeout 3 "$CONTAINER" >/dev/null 2>&1 || true
    ROBOT="$ROBOT" "$SCRIPT_DIR/robot-mode.sh" valetudo || true
}

trap restore_valetudo EXIT INT TERM

ROBOT="$ROBOT" DEMO_SECONDS="$DEMO_SECONDS" \
    "$SCRIPT_DIR/robot-mode.sh" sangamio

echo "Starting physical RViz monitoring. No movement command will be sent."
echo "The robot must be physically undocked so the LiDAR can start."
echo "Close RViz or press Ctrl-C to stop the LiDAR and restore Valetudo."

docker run --rm --name "$CONTAINER" --network host \
    --user "$(id -u):$(id -g)" \
    -e DISPLAY="${DISPLAY:-}" \
    -e HOME=/tmp \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e ROBOT_IP="$ROBOT_IP" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$BRIDGE_REPO:/ws/src/vacuum_ros2_bridge:ro" \
    -v "$ROBOT_DESCRIPTION_REPO:/ws/src/roborock_s5max:ro" \
    -v "$ROS_WORKSPACE/build:/ws/build" \
    -v "$ROS_WORKSPACE/install:/ws/install:ro" \
    -v "$ROS_WORKSPACE/log:/ws/log" \
    -v "$SCRIPT_DIR/run-ros-rviz.sh:/tmp/run-ros-rviz.sh:ro" \
    -w /ws "$ROS_IMAGE" bash /tmp/run-ros-rviz.sh
