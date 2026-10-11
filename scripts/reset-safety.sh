#!/usr/bin/env bash
set -euo pipefail

container=""
for candidate in s5max-joystick-demo s5max-rviz-monitor; do
    if [ "$(docker inspect --format '{{.State.Running}}' "$candidate" 2>/dev/null || true)" = "true" ]; then
        container="$candidate"
        break
    fi
done

if [ -z "$container" ]; then
    echo "ERROR: No active S5 Max ROS session exists. Run make demo or make rviz first." >&2
    exit 1
fi

echo "Requesting a safety reset from $container..."
response="$(
    docker exec "$container" bash -lc '
        source /opt/ros/jazzy/setup.bash
        source /ws/install/setup.bash
        timeout 10 ros2 service call /reset_safety std_srvs/srv/Trigger "{}"
    '
)"

echo "$response"
if ! grep -q 'success=True' <<<"$response"; then
    echo "ERROR: The safety adapter rejected the reset." >&2
    exit 1
fi

echo "The physical safety stop is released."
