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
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER=s5max-navigation

find_joystick()
{
    for event_link in /dev/input/by-id/*-event-joystick; do
        [ -e "$event_link" ] || continue
        js_link="${event_link%-event-joystick}-joystick"
        if [ -e "$js_link" ]; then
            JOY_EVENT="$event_link"
            JOY_JS="$js_link"
            return 0
        fi
    done

    for name_file in /sys/class/input/input*/name; do
        [ -e "$name_file" ] || continue
        if grep -Eqi 'Microntek|USB[[:space:]]+Joystick' "$name_file"; then
            input_dir="${name_file%/name}"
            event_name="$(find "$input_dir" -maxdepth 1 -name 'event*' -printf '%f\n' | head -n 1)"
            js_name="$(find "$input_dir" -maxdepth 1 -name 'js*' -printf '%f\n' | head -n 1)"
            if [ -n "$event_name" ] && [ -n "$js_name" ]; then
                JOY_EVENT="/dev/input/$event_name"
                JOY_JS="/dev/input/$js_name"
                return 0
            fi
        fi
    done
    return 1
}

restore_valetudo()
{
    docker stop --timeout 30 "$CONTAINER" >/dev/null 2>&1 || true
    ROBOT="$ROBOT" "$SCRIPT_DIR/robot-mode.sh" valetudo || true
}

if [ -z "$NAME" ]; then
    echo "ERROR: A map name is required. Use make navigate NAME=apartment." >&2
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
if [ -z "${DISPLAY:-}" ] || [ ! -d /tmp/.X11-unix ]; then
    echo "ERROR: An X11 display is required for RViz." >&2
    exit 1
fi

ENABLE_JOYSTICK=false
DEVICE_ARGS=()
GROUP_ARGS=()
if find_joystick; then
    ENABLE_JOYSTICK=true
    joy_event_real="$(readlink -f "$JOY_EVENT")"
    joy_js_real="$(readlink -f "$JOY_JS")"
    joy_event_gid="$(stat -Lc %g "$JOY_EVENT")"
    joy_js_gid="$(stat -Lc %g "$JOY_JS")"
    DEVICE_ARGS=(
        --device "$joy_event_real:$joy_event_real"
        --device "$joy_js_real:$joy_js_real"
    )
    GROUP_ARGS=(--group-add "$joy_event_gid")
    if [ "$joy_js_gid" != "$joy_event_gid" ]; then
        GROUP_ARGS+=(--group-add "$joy_js_gid")
    fi
    echo "Joystick found: $JOY_EVENT and $JOY_JS"
else
    echo "No joystick found. RViz navigation remains available."
fi

echo "Map: $MAP_DIR/$NAME.yaml"
echo "Place the robot undocked at the map starting position."
echo "Keep the floor clear around the robot."
read -r -p "Type 'floor-clear' to continue: " confirmation
if [ "$confirmation" != "floor-clear" ]; then
    echo "The navigation session was canceled."
    exit 1
fi

trap restore_valetudo EXIT INT TERM

ROBOT="$ROBOT" DEMO_SECONDS="$DEMO_SECONDS" \
    "$SCRIPT_DIR/robot-mode.sh" sangamio

docker run --rm --name "$CONTAINER" --network host \
        --user "$(id -u):$(id -g)" \
        "${GROUP_ARGS[@]}" \
        "${DEVICE_ARGS[@]}" \
        -e DISPLAY="${DISPLAY:-}" \
        -e HOME=/tmp \
        -e QT_X11_NO_MITSHM=1 \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        -e ROBOT_IP="$ROBOT_IP" \
        -e MAP_NAME="$NAME" \
        -e ENABLE_JOYSTICK="$ENABLE_JOYSTICK" \
        -v /run/udev:/run/udev:ro \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$BRIDGE_REPO:/ws/src/vacuum_ros2_bridge:ro" \
        -v "$ROBOT_DESCRIPTION_REPO:/ws/src/roborock_s5max:ro" \
        -v "$ROS_WORKSPACE/build:/ws/build" \
        -v "$ROS_WORKSPACE/install:/ws/install:ro" \
        -v "$ROS_WORKSPACE/log:/ws/log" \
        -v "$MAP_DIR:/maps:ro" \
        -v "$SCRIPT_DIR/joystick_pilot.py:/tmp/joystick_pilot.py:ro" \
        -v "$SCRIPT_DIR/run-ros-navigation.sh:/tmp/run-ros-navigation.sh:ro" \
        -w /ws "$ROS_IMAGE" bash /tmp/run-ros-navigation.sh
