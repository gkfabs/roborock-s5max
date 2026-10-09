#!/usr/bin/env bash
set -euo pipefail

ROBOT="${ROBOT:-root@192.168.4.55}"
DEMO_SECONDS="${DEMO_SECONDS:-1800}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SSH=(ssh -F /dev/null)
SCP=(scp -F /dev/null -O)

wait_for_valetudo()
{
    for attempt in $(seq 1 48); do
        if "${SSH[@]}" -o ConnectTimeout=3 "$ROBOT" \
            'wget -q -O /dev/null http://127.0.0.1/ &&
             pidof rr_loader >/dev/null && pidof WatchDoge >/dev/null &&
             pidof RoboController >/dev/null && pidof AppProxy >/dev/null &&
             pidof valetudo >/dev/null' 2>/dev/null
        then
            echo "Valetudo is ready"
            return 0
        fi
        if [ $((attempt % 4)) -eq 0 ]; then
            echo "Waiting for Valetudo..."
        fi
        sleep 5
    done
    echo "ERROR: Valetudo did not become ready" >&2
    return 1
}

case "${1:-}" in
    valetudo)
        if "${SSH[@]}" -o ConnectTimeout=3 "$ROBOT" \
            'pidof rr_loader >/dev/null && pidof WatchDoge >/dev/null &&
             pidof RoboController >/dev/null && pidof AppProxy >/dev/null &&
             pidof valetudo >/dev/null && ! pidof sangamio-stationary-test >/dev/null' \
             2>/dev/null
        then
            echo "The stock robot processes are active"
        else
            echo "Rebooting the robot to restore Valetudo..."
            "${SSH[@]}" "$ROBOT" '/sbin/reboot -f' >/dev/null 2>&1 || true
        fi
        wait_for_valetudo
        ;;
    sangamio)
        if "${SSH[@]}" "$ROBOT" 'pidof sangamio-stationary-test >/dev/null && netstat -lnt 2>/dev/null | grep -q ":5555 "'
        then
            echo "SangamIO is already ready"
            exit 0
        fi
        "${SCP[@]}" "$SCRIPT_DIR/takeover.sh" "$ROBOT:/mnt/data/s5max-demo-takeover.sh"
        "${SSH[@]}" "$ROBOT" "chmod 755 /mnt/data/s5max-demo-takeover.sh && S5MAX_SESSION_SECONDS='$DEMO_SECONDS' ash -c '/mnt/data/s5max-demo-takeover.sh >/mnt/data/s5max-demo-launch.log 2>&1 &'"
        for _ in $(seq 1 20); do
            if "${SSH[@]}" "$ROBOT" 'netstat -lnt 2>/dev/null | grep -q ":5555 "'
            then
                echo "SangamIO is ready"
                exit 0
            fi
            sleep 1
        done
        "${SSH[@]}" "$ROBOT" 'cat /mnt/data/s5max-demo-launch.log' >&2 || true
        echo "ERROR: SangamIO did not become ready" >&2
        exit 1
        ;;
    *)
        echo "Usage: $0 valetudo|sangamio" >&2
        exit 2
        ;;
esac
