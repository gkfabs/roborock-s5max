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
CONTAINER=s5max-joystick-demo
JOY_WAIT_SECONDS="${JOY_WAIT_SECONDS:-15}"

find_joystick_once()
{
    if [ -n "${JOY_EVENT:-}" ] && [ -n "${JOY_JS:-}" ] && \
            [ -e "$JOY_EVENT" ] && [ -e "$JOY_JS" ]; then
        return 0
    fi

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

find_joystick()
{
    end_time=$((SECONDS + JOY_WAIT_SECONDS))
    while ! find_joystick_once; do
        if [ "$SECONDS" -ge "$end_time" ]; then
            echo "ERROR: USB joystick was not found after ${JOY_WAIT_SECONDS}s" >&2
            return 1
        fi
        echo "Waiting for the USB joystick..." >&2
        sleep 1
    done
}

restore_valetudo()
{
    docker stop --timeout 3 "$CONTAINER" >/dev/null 2>&1 || true
    ROBOT="$ROBOT" "$SCRIPT_DIR/robot-mode.sh" valetudo || true
}

find_joystick
echo "Joystick found: $JOY_EVENT and $JOY_JS"
echo "The robot must be physically undocked on a clear, level floor."
read -r -p "Type 'floor-clear' to continue: " confirmation
if [ "$confirmation" != "floor-clear" ]; then
    echo "The demonstration was canceled."
    exit 1
fi

trap restore_valetudo EXIT INT TERM

ROBOT="$ROBOT" DEMO_SECONDS="$DEMO_SECONDS" \
    "$SCRIPT_DIR/robot-mode.sh" sangamio

echo "Joystick controls:"
echo "  Hold button 3: enable movement and motor toggles"
echo "  Axis 1: forward and reverse, up to 0.20 m/s"
echo "  Axis 0: left and right, up to 1.20 rad/s"
echo "  Button 0: toggle vacuum fan at 38%"
echo "  Button 1: toggle main brush at 71%"
echo "  Button 2: toggle side brush at 30%"
echo "Release button 3 to stop movement."
echo "Close RViz or press Ctrl-C to stop all actuators and restore Valetudo."

JOY_EVENT_GID="$(stat -Lc %g "$JOY_EVENT")"
JOY_JS_GID="$(stat -Lc %g "$JOY_JS")"
JOY_EVENT_REAL="$(readlink -f "$JOY_EVENT")"
JOY_JS_REAL="$(readlink -f "$JOY_JS")"
GROUP_ARGS=(--group-add "$JOY_EVENT_GID")
if [ "$JOY_JS_GID" != "$JOY_EVENT_GID" ]; then
    GROUP_ARGS+=(--group-add "$JOY_JS_GID")
fi

docker run --rm --name "$CONTAINER" --network host \
    --user "$(id -u):$(id -g)" \
    "${GROUP_ARGS[@]}" \
    --device "$JOY_EVENT_REAL:$JOY_EVENT_REAL" \
    --device "$JOY_JS_REAL:$JOY_JS_REAL" \
    -e DISPLAY="${DISPLAY:-}" \
    -e HOME=/tmp \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e ROBOT_IP="$ROBOT_IP" \
    -v /run/udev:/run/udev:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$BRIDGE_REPO:/ws/src/vacuum_ros2_bridge:ro" \
    -v "$ROBOT_DESCRIPTION_REPO:/ws/src/roborock_s5max:ro" \
    -v "$ROS_WORKSPACE/build:/ws/build" \
    -v "$ROS_WORKSPACE/install:/ws/install:ro" \
    -v "$ROS_WORKSPACE/log:/ws/log" \
    -v "$SCRIPT_DIR/joystick_pilot.py:/tmp/joystick_pilot.py:ro" \
    -v "$SCRIPT_DIR/run-ros-joystick.sh:/tmp/run-ros-joystick.sh:ro" \
    -w /ws "$ROS_IMAGE" bash /tmp/run-ros-joystick.sh
