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
CONTAINER=s5max-mapping
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
    if [ "$(docker inspect --format '{{.State.Running}}' "$CONTAINER" \
            2>/dev/null || true)" = "true" ]; then
        docker exec "$CONTAINER" bash /tmp/save-map.sh || true
    fi
    docker stop --timeout 30 "$CONTAINER" >/dev/null 2>&1 || true
    ROBOT="$ROBOT" "$SCRIPT_DIR/robot-mode.sh" valetudo || true
}

if [ -z "$NAME" ]; then
    echo "ERROR: A map name is required. Use make map NAME=apartment." >&2
    exit 1
fi
if [[ ! "$NAME" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
    echo "ERROR: NAME can contain letters, numbers, periods, underscores, and hyphens." >&2
    exit 1
fi
if [ -z "${DISPLAY:-}" ] || [ ! -d /tmp/.X11-unix ]; then
    echo "ERROR: An X11 display is required for RViz." >&2
    exit 1
fi

mkdir -p "$MAP_DIR"
for output in \
        "$MAP_DIR/$NAME.yaml" \
        "$MAP_DIR/$NAME.pgm" \
        "$MAP_DIR/$NAME.png" \
        "$MAP_DIR/${NAME}_serial.posegraph" \
        "$MAP_DIR/${NAME}_serial.data"; do
    if [ -e "$output" ]; then
        echo "ERROR: Map output already exists: $output" >&2
        exit 1
    fi
done

find_joystick
echo "Joystick found: $JOY_EVENT and $JOY_JS"
echo "Map output: $MAP_DIR/$NAME"
echo "The robot must be physically undocked on a clear, level floor."
read -r -p "Type 'floor-clear' to continue: " confirmation
if [ "$confirmation" != "floor-clear" ]; then
    echo "The mapping session was canceled."
    exit 1
fi

trap restore_valetudo EXIT INT TERM

ROBOT="$ROBOT" DEMO_SECONDS="$DEMO_SECONDS" \
    "$SCRIPT_DIR/robot-mode.sh" sangamio

echo "Hold joystick button 3 to move the robot."
echo "Use slow movement and gentle turns during mapping."
echo "Close RViz or press Ctrl-C to save the map."

JOY_EVENT_GID="$(stat -Lc %g "$JOY_EVENT")"
JOY_JS_GID="$(stat -Lc %g "$JOY_JS")"
JOY_EVENT_REAL="$(readlink -f "$JOY_EVENT")"
JOY_JS_REAL="$(readlink -f "$JOY_JS")"
GROUP_ARGS=(--group-add "$JOY_EVENT_GID")
if [ "$JOY_JS_GID" != "$JOY_EVENT_GID" ]; then
    GROUP_ARGS+=(--group-add "$JOY_JS_GID")
fi

docker_status=0
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
        -e MAP_NAME="$NAME" \
        -v /run/udev:/run/udev:ro \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$BRIDGE_REPO:/ws/src/vacuum_ros2_bridge:ro" \
        -v "$ROBOT_DESCRIPTION_REPO:/ws/src/roborock_s5max:ro" \
        -v "$ROS_WORKSPACE/build:/ws/build" \
        -v "$ROS_WORKSPACE/install:/ws/install:ro" \
        -v "$ROS_WORKSPACE/log:/ws/log" \
        -v "$MAP_DIR:/maps" \
        -v "$SCRIPT_DIR/joystick_pilot.py:/tmp/joystick_pilot.py:ro" \
        -v "$SCRIPT_DIR/save-map.sh:/tmp/save-map.sh:ro" \
        -v "$SCRIPT_DIR/promote-recovery-map.sh:/tmp/promote-recovery-map.sh:ro" \
        -v "$SCRIPT_DIR/run-ros-mapping.sh:/tmp/run-ros-mapping.sh:ro" \
        -w /ws "$ROS_IMAGE" bash /tmp/run-ros-mapping.sh || docker_status=$?

if [ ! -f "$MAP_DIR/$NAME.yaml" ] || \
        [ ! -f "$MAP_DIR/${NAME}_serial.posegraph" ]; then
    MAP_DIRECTORY="$MAP_DIR" MAP_NAME="$NAME" \
        "$SCRIPT_DIR/promote-recovery-map.sh" || true
fi
if [ ! -f "$MAP_DIR/$NAME.yaml" ]; then
    echo "ERROR: The mapping session did not create $MAP_DIR/$NAME.yaml." >&2
    exit 1
fi
if [ ! -f "$MAP_DIR/${NAME}_serial.posegraph" ]; then
    echo "ERROR: The mapping session did not create the SLAM pose graph." >&2
    exit 1
fi

for slot in a b; do
    recovery_name=".${NAME}.recovery-${slot}"
    rm -f \
        "$MAP_DIR/${recovery_name}.yaml" \
        "$MAP_DIR/${recovery_name}.pgm" \
        "$MAP_DIR/${recovery_name}.png" \
        "$MAP_DIR/${recovery_name}_serial.posegraph" \
        "$MAP_DIR/${recovery_name}_serial.data"
done
rm -f \
    "$MAP_DIR/.${NAME}.recovery-latest" \
    "$MAP_DIR/.${NAME}.recovery-latest.new"

if [ "$docker_status" -ne 0 ]; then
    echo "The mapping container stopped with status $docker_status after saving." >&2
fi

echo "Saved map: $MAP_DIR/$NAME.yaml"
echo "Saved pose graph: $MAP_DIR/${NAME}_serial.posegraph"
