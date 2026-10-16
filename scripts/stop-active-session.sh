#!/usr/bin/env bash
set -euo pipefail

ROS_IMAGE="${ROS_IMAGE:-oomwoo_one:latest}"
ROS_WORKSPACE="${ROS_WORKSPACE:-/tmp/s5_ros_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BRIDGE_REPO="${BRIDGE_REPO:-$REPOSITORY_DIR/../vacuum_ros2_bridge-review}"
ROBOT_DESCRIPTION_REPO="${ROBOT_DESCRIPTION_REPO:-$REPOSITORY_DIR}"

if [ ! -f "$ROS_WORKSPACE/install/setup.bash" ]; then
    echo "ERROR: The ROS workspace is not built. Run make teleop or make ros-build first." >&2
    exit 1
fi

echo "Sending immediate wheel and cleaning actuator stop commands."
docker run --rm --network host \
    -v "$BRIDGE_REPO:/ws/src/vacuum_ros2_bridge:ro" \
    -v "$ROBOT_DESCRIPTION_REPO:/ws/src/roborock_s5max:ro" \
    -v "$ROS_WORKSPACE/build:/ws/build:ro" \
    -v "$ROS_WORKSPACE/install:/ws/install:ro" \
    -w /ws "$ROS_IMAGE" bash -lc '
        source /opt/ros/jazzy/setup.bash
        source /ws/install/setup.bash
        timeout 3 ros2 topic pub --times 5 --rate 20 \
            /cmd_vel_manual geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
        timeout 3 ros2 topic pub --once \
            /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
        for component in vacuum main_brush side_brush water_pump; do
            timeout 3 ros2 topic pub --once /actuator_cmd \
                vacuum_ros2_bridge/msg/ActuatorCommand \
                "{component: \"$component\", speed: 0}" >/dev/null 2>&1 || true
        done
    '
echo "Stop commands were submitted. The active ROS session remains open."
