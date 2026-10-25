#!/usr/bin/env bash

set -u
trap 'exit 130' INT TERM

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scenarios="${SIM_SCENARIOS:-centered lateral-offset heading-error beacon-loss missed-contact delayed-charging dock-undock-cycle}"
case_pause_sec="${SIM_CASE_PAUSE_SEC:-15}"
startup_restarts="${SIM_STARTUP_RESTARTS:-1}"
failures=0

for scenario in $scenarios; do
    attempt=0
    while true; do
        echo "Starting OpenNav dock scenario: $scenario"
        if SIM_SCENARIO="$scenario" "$script_dir/run-opennav-dock-sim.sh"; then
            echo "PASS: $scenario"
            break
        else
            status=$?
        fi
        if [ "$status" -eq 75 ] && [ "$attempt" -lt "$startup_restarts" ]; then
            attempt=$((attempt + 1))
            echo "RESTART: $scenario did not receive simulation data."
            sleep "$case_pause_sec"
            continue
        fi
        echo "FAIL: $scenario" >&2
        failures=$((failures + 1))
        break
    done
    sleep "$case_pause_sec"
done

if [ "$failures" -ne 0 ]; then
    echo "ERROR: $failures dock simulation scenarios failed." >&2
    exit 1
fi

echo "All OpenNav dock simulation scenarios passed."
