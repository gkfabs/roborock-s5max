#!/usr/bin/env bash
set -euo pipefail

ROBOT="${ROBOT:-root@192.168.4.55}"
ROBOT_IP="${ROBOT_IP:-192.168.4.55}"
ROS_IMAGE="${ROS_IMAGE:-roborock_s5max_ros:latest}"
ROS_WORKSPACE="${ROS_WORKSPACE:-/tmp/s5_ros_ws}"
BRIDGE_REPO="${BRIDGE_REPO:?BRIDGE_REPO is required}"
ROBOT_DESCRIPTION_REPO="${ROBOT_DESCRIPTION_REPO:?ROBOT_DESCRIPTION_REPO is required}"
DEMO_SECONDS="${DEMO_SECONDS:-1800}"
MAP_DIR="${MAP_DIR:?MAP_DIR is required}"
NAME="${NAME:-}"
DOCK_MODE="${DOCK_MODE:?DOCK_MODE is required}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER=s5max-docking

restore_valetudo()
{
    docker stop --timeout 30 "$CONTAINER" >/dev/null 2>&1 || true
    ROBOT="$ROBOT" "$SCRIPT_DIR/robot-mode.sh" valetudo || true
}

if [ -z "$NAME" ]; then
    echo "ERROR: A map name is required." >&2
    exit 1
fi
if [[ ! "$NAME" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
    echo "ERROR: NAME can contain letters, numbers, periods, underscores, and hyphens." >&2
    exit 1
fi
if [ ! -f "$MAP_DIR/$NAME.yaml" ]; then
    echo "ERROR: The occupancy map does not exist: $MAP_DIR/$NAME.yaml" >&2
    exit 1
fi
if [ ! -f "$MAP_DIR/${NAME}_serial.posegraph" ]; then
    echo "ERROR: The pose graph does not exist: $MAP_DIR/${NAME}_serial.posegraph" >&2
    exit 1
fi
if [ "$DOCK_MODE" = return ] && [ ! -f "$MAP_DIR/$NAME.dock.json" ]; then
    echo "ERROR: The dock staging pose does not exist: $MAP_DIR/$NAME.dock.json" >&2
    echo "Run make dock-calibrate NAME=$NAME while the robot is docked." >&2
    exit 1
fi
if [ "$DOCK_MODE" = calibrate ] && [ -f "$MAP_DIR/$NAME.dock.json" ]; then
    echo "ERROR: The dock staging pose already exists: $MAP_DIR/$NAME.dock.json" >&2
    exit 1
fi
if [ -z "${DISPLAY:-}" ] || [ ! -d /tmp/.X11-unix ]; then
    echo "ERROR: An X11 display is required for localization review." >&2
    exit 1
fi

if [ "$DOCK_MODE" = calibrate ]; then
    echo "The robot must start docked on map $NAME."
    echo "The command will undock, save the staging pose, and redock."
else
    echo "The command will navigate to the saved dock staging pose and redock."
fi

trap restore_valetudo EXIT INT TERM

ROBOT="$ROBOT" DEMO_SECONDS="$DEMO_SECONDS" \
    "$SCRIPT_DIR/robot-mode.sh" sangamio

docker run --rm -i --name "$CONTAINER" --network host \
    --user "$(id -u):$(id -g)" \
    -e DISPLAY="${DISPLAY:-}" \
    -e HOME=/tmp \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e ROBOT_IP="$ROBOT_IP" \
    -e MAP_NAME="$NAME" \
    -e DOCK_MODE="$DOCK_MODE" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$BRIDGE_REPO:/ws/src/vacuum_ros2_bridge:ro" \
    -v "$ROBOT_DESCRIPTION_REPO:/ws/src/roborock_s5max:ro" \
    -v "$ROS_WORKSPACE/build:/ws/build" \
    -v "$ROS_WORKSPACE/install:/ws/install:ro" \
    -v "$ROS_WORKSPACE/log:/ws/log" \
    -v "$MAP_DIR:/maps:rw" \
    -v "$SCRIPT_DIR/run-ros-docking.sh:/tmp/run-ros-docking.sh:ro" \
    -w /ws "$ROS_IMAGE" bash /tmp/run-ros-docking.sh
