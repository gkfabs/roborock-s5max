#!/usr/bin/env bash
set -euo pipefail

ROS_IMAGE="${ROS_IMAGE:?ROS_IMAGE is required}"
ROS_WORKSPACE="${ROS_WORKSPACE:?ROS_WORKSPACE is required}"
BRIDGE_REPO="${BRIDGE_REPO:?BRIDGE_REPO is required}"
ROBOT_DESCRIPTION_REPO="${ROBOT_DESCRIPTION_REPO:?ROBOT_DESCRIPTION_REPO is required}"
SIM_GUI="${SIM_GUI:-1}"
SIM_SCENARIO="${SIM_SCENARIO:-centered}"
SIM_LIDAR_PROFILE="${SIM_LIDAR_PROFILE:-realistic}"
SIM_LIDAR_GEOMETRY_OUTAGE_START="${SIM_LIDAR_GEOMETRY_OUTAGE_START:-0.0}"
SIM_LIDAR_GEOMETRY_OUTAGE_DURATION="${SIM_LIDAR_GEOMETRY_OUTAGE_DURATION:-0.0}"
SIM_LIDAR_GEOMETRY_OUTAGE_STATE="${SIM_LIDAR_GEOMETRY_OUTAGE_STATE:-}"
SIM_LIDAR_GEOMETRY_OUTAGE_STATE_DELAY="${SIM_LIDAR_GEOMETRY_OUTAGE_STATE_DELAY:-0.25}"
lidar_noise_profile="$SIM_LIDAR_PROFILE"

x_pose=-0.25
y_pose=0.0
yaw=0.0
case "$SIM_SCENARIO" in
    centered|beacon-loss|missed-contact|delayed-charging)
        ;;
    dock-undock-cycle)
        lidar_noise_profile=clean
        ;;
    lateral-offset)
        x_pose=-0.40
        y_pose=0.07
        ;;
    heading-error)
        yaw=0.15
        ;;
    *)
        echo "ERROR: Unknown dock simulation scenario: $SIM_SCENARIO" >&2
        exit 2
        ;;
esac

echo "Dock simulation start: scenario=$SIM_SCENARIO x=$x_pose y=$y_pose yaw=$yaw"

mkdir -p "$ROS_WORKSPACE/log"
result_name="s5max-dock-result-${SIM_SCENARIO}-$$.json"
result_path="$ROS_WORKSPACE/log/$result_name"
simulation_domain="$((100 + $$ % 100))"
gazebo_partition="s5max_dock_${simulation_domain}_$$"

docker_arguments=(
    --rm
    --network host
    -e "ROS_DOMAIN_ID=$simulation_domain"
    -e "GZ_PARTITION=$gazebo_partition"
)
launch_arguments=(
    gui:=true
    "scenario:=$SIM_SCENARIO"
    "x_pose:=$x_pose"
    "y_pose:=$y_pose"
    "yaw:=$yaw"
    "result_path:=/ws/log/$result_name"
    "undock_after_docking:=false"
    "lidar_noise_profile:=$lidar_noise_profile"
    "lidar_outage_start_sec:=$SIM_LIDAR_GEOMETRY_OUTAGE_START"
    "lidar_outage_duration_sec:=$SIM_LIDAR_GEOMETRY_OUTAGE_DURATION"
    "lidar_outage_state_delay_sec:=$SIM_LIDAR_GEOMETRY_OUTAGE_STATE_DELAY"
)
if [ "$SIM_SCENARIO" = dock-undock-cycle ]; then
    launch_arguments[6]=undock_after_docking:=true
fi
if [ -n "$SIM_LIDAR_GEOMETRY_OUTAGE_STATE" ]; then
    launch_arguments+=(
        "lidar_outage_state:=$SIM_LIDAR_GEOMETRY_OUTAGE_STATE")
fi
if [ "$SIM_GUI" = 1 ]; then
    docker_arguments+=(
        --gpus all
        --user "$(id -u):$(id -g)"
        -e "DISPLAY=${DISPLAY:-}"
        -e HOME=/tmp
        -e QT_X11_NO_MITSHM=1
        -e NVIDIA_VISIBLE_DEVICES=all
        -e NVIDIA_DRIVER_CAPABILITIES=all
        -e __NV_PRIME_RENDER_OFFLOAD=1
        -e __GLX_VENDOR_LIBRARY_NAME=nvidia
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    )
else
    launch_arguments[0]=gui:=false
fi

docker run "${docker_arguments[@]}" \
    -v "$BRIDGE_REPO:/ws/src/vacuum_ros2_bridge:ro" \
    -v "$ROBOT_DESCRIPTION_REPO:/ws/src/roborock_s5max:ro" \
    -v "$ROS_WORKSPACE/build:/ws/build:ro" \
    -v "$ROS_WORKSPACE/install:/ws/install:ro" \
    -v "$ROS_WORKSPACE/log:/ws/log" \
    -w /ws "$ROS_IMAGE" bash -lc \
    "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
    ros2 launch roborock_s5max dock_sim.launch.py ${launch_arguments[*]}"

if [ ! -f "$result_path" ]; then
    echo "ERROR: The $SIM_SCENARIO simulation produced no action result." >&2
    exit 75
fi
if ! grep -q '"success": true' "$result_path"; then
    echo "ERROR: The $SIM_SCENARIO simulation did not dock." >&2
    sed -n '1p' "$result_path" >&2
    if grep -q '"last_state": null' "$result_path"; then
        exit 75
    fi
    exit 1
fi
if ! grep -q '"dock_id": "home_dock"' "$result_path"; then
    echo "ERROR: The $SIM_SCENARIO simulation did not use home_dock." >&2
    sed -n '1p' "$result_path" >&2
    exit 1
fi
echo "PASS: OpenNav Docking completed the $SIM_SCENARIO simulation."
