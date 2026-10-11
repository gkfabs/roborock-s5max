#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

set -u

LOCK_DIR=/tmp/s5max-map-save.lock
SAVE_MAP_NAME="${SAVE_MAP_NAME:-$MAP_NAME}"
COMPLETE_FILE="${SAVE_COMPLETE_FILE:-/tmp/s5max-map-save-complete}"

if [ -f "$COMPLETE_FILE" ]; then
    echo "The map is already saved."
    exit 0
fi

if ! mkdir "$LOCK_DIR" 2>/dev/null; then
    echo "Another map save is active. Waiting for it to finish..."
    for _ in $(seq 1 30); do
        if [ -f "$COMPLETE_FILE" ]; then
            echo "The map save completed."
            exit 0
        fi
        sleep 1
    done
    echo "ERROR: The active map save did not finish." >&2
    exit 1
fi

remove_lock()
{
    rmdir "$LOCK_DIR" 2>/dev/null || true
}
trap remove_lock EXIT

if ! timeout 5 ros2 topic echo --once /map >/dev/null 2>&1; then
    echo "ERROR: No map data is available." >&2
    exit 1
fi

echo "Saving the occupancy map as /maps/${SAVE_MAP_NAME}..."
timeout 20 ros2 run nav2_map_server map_saver_cli -f "/maps/${SAVE_MAP_NAME}"

echo "Saving the SLAM pose graph as /maps/${SAVE_MAP_NAME}_serial..."
serialize_response="$(
    timeout 20 ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph \
        "{filename: '/maps/${SAVE_MAP_NAME}_serial'}" 2>&1
)" || serialize_status=$?
echo "$serialize_response"
if [ "${serialize_status:-0}" -ne 0 ] && \
        [ "${serialize_status:-0}" -ne 124 ]; then
    echo "ERROR: The SLAM pose graph request failed." >&2
    exit 1
fi

if [ ! -f "/maps/${SAVE_MAP_NAME}.yaml" ]; then
    echo "ERROR: The occupancy map file is missing." >&2
    exit 1
fi
if [ ! -f "/maps/${SAVE_MAP_NAME}_serial.posegraph" ]; then
    echo "ERROR: The SLAM pose graph file is missing." >&2
    exit 1
fi

touch "$COMPLETE_FILE"
echo "The map and pose graph are saved."
