#!/usr/bin/env bash

set -u

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
states="${SIM_LIDAR_OUTAGE_STATES:-INITIAL_PERCEPTION CONTROLLING WAIT_FOR_CHARGE}"
duration="${SIM_LIDAR_GEOMETRY_OUTAGE_DURATION:-2.0}"
failures=0

scenario_for_state()
{
    case "$1" in
        WAIT_FOR_CHARGE)
            echo delayed-charging
            ;;
        *)
            echo centered
            ;;
    esac
}

for state in $states; do
    scenario="$(scenario_for_state "$state")"
    output_file="$(mktemp "/tmp/s5max-lidar-$state-XXXXXX.log")"
    echo "Starting LiDAR outage test: state=$state scenario=$scenario"
    if SIM_SCENARIO="$scenario" \
        SIM_LIDAR_PROFILE=realistic \
        SIM_LIDAR_GEOMETRY_OUTAGE_STATE="$state" \
        SIM_LIDAR_GEOMETRY_OUTAGE_DURATION="$duration" \
        "$script_dir/run-opennav-dock-sim.sh" 2>&1 | tee "$output_file"; then
        if grep -q "LiDAR geometry outage will start during $state" \
                "$output_file"; then
            echo "PASS: LiDAR outage during $state"
        else
            echo "FAIL: The $state outage did not start." >&2
            failures=$((failures + 1))
        fi
    else
        echo "FAIL: Docking failed with a LiDAR outage during $state." >&2
        failures=$((failures + 1))
    fi
    rm -f "$output_file"
done

if [ "$failures" -ne 0 ]; then
    echo "ERROR: $failures LiDAR outage simulations failed." >&2
    exit 1
fi

echo "All OpenNav state LiDAR outage simulations passed."
