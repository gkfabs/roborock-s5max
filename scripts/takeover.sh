#!/bin/sh
set -eu

BIN=/mnt/data/sangamio-stationary-test
SEQUENCE_READER=/mnt/data/roborock_s5max_sequence-stationary-test
BASE_CONFIG=/mnt/data/roborock-s5max-stationary-test.toml
LIVE_CONFIG=/tmp/roborock-s5max-demo.toml
LOG=/mnt/data/sangamio-demo.log
INITTAB_COPY=/tmp/inittab.no-watchdog
CONTROL_PIDS=""
INITTAB_MOUNTED=0
SESSION_SECONDS="${S5MAX_SESSION_SECONDS:-1800}"

recover()
{
    trap - EXIT HUP INT TERM
    sync
    /sbin/reboot -f
    if [ "$INITTAB_MOUNTED" -eq 1 ]; then
        umount /etc/inittab 2>/dev/null || true
        kill -HUP 1 2>/dev/null || true
    fi
    for pid in $CONTROL_PIDS; do
        kill -CONT "$pid" 2>/dev/null || true
    done
}

fail()
{
    echo "ERROR: $*" >&2
    exit 1
}

[ -x "$BIN" ] || fail "missing $BIN"
[ -x "$SEQUENCE_READER" ] || fail "missing $SEQUENCE_READER"
[ -f "$BASE_CONFIG" ] || fail "missing $BASE_CONFIG"

WD_PIDS="$(pidof WatchDoge 2>/dev/null)"
RR_PIDS="$(pidof rr_loader 2>/dev/null)"
[ -n "$WD_PIDS" ] || fail "WatchDoge is not running"
[ -n "$RR_PIDS" ] || fail "rr_loader is not running"

RR_PID=""
for pid in $RR_PIDS; do
    for fd in /proc/$pid/fd/*; do
        if [ "$(readlink "$fd" 2>/dev/null)" = /dev/ttyS2 ]; then
            RR_PID="$pid"
            break 2
        fi
    done
done
[ -n "$RR_PID" ] || fail "no rr_loader process owns /dev/ttyS2"

grep -v WatchDoge /etc/inittab > "$INITTAB_COPY" || fail "cannot prepare inittab"
trap recover EXIT HUP INT TERM
mount --bind "$INITTAB_COPY" /etc/inittab || fail "cannot suppress WatchDoge respawn"
INITTAB_MOUNTED=1
kill -HUP 1
sleep 1

for pid in $WD_PIDS $RR_PIDS; do
    kill -STOP "$pid" 2>/dev/null || true
done
for name in RoboController AppProxy; do
    for pid in $(pidof "$name" 2>/dev/null); do
        CONTROL_PIDS="$CONTROL_PIDS $pid"
        kill -STOP "$pid" 2>/dev/null || true
    done
done

LAST_SEQUENCE="$($SEQUENCE_READER "$RR_PID")"
case "$LAST_SEQUENCE" in
    ''|*[!0-9]*) fail "cannot read the frozen stock MCU sequence" ;;
esac
echo "Captured final stock MCU sequence: $LAST_SEQUENCE"

for pid in $WD_PIDS $RR_PIDS; do
    kill -TERM "$pid" 2>/dev/null || true
    kill -CONT "$pid" 2>/dev/null || true
done
sleep 2
for pid in $WD_PIDS $RR_PIDS; do
    if kill -0 "$pid" 2>/dev/null; then
        kill -KILL "$pid" 2>/dev/null || true
    fi
done

sed "s/^# last_stock_sequence = 0/last_stock_sequence = $LAST_SEQUENCE/" \
    "$BASE_CONFIG" > "$LIVE_CONFIG"
sed 's/^allow_actuation = false/allow_actuation = true/' "$LIVE_CONFIG" \
    > "$LIVE_CONFIG.active"
mv "$LIVE_CONFIG.active" "$LIVE_CONFIG"

RUST_LOG=info "$BIN" --config "$LIVE_CONFIG" > "$LOG" 2>&1 &
SANGAM_PID=$!
sleep 2
kill -0 "$SANGAM_PID" 2>/dev/null || fail "SangamIO exited during startup"
echo "SangamIO demo is ready"

sleep "$SESSION_SECONDS"

if kill -0 "$SANGAM_PID" 2>/dev/null; then
    kill -INT "$SANGAM_PID"
    wait "$SANGAM_PID"
fi
