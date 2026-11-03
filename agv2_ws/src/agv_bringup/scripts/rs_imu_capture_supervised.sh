#!/usr/bin/env bash
# Supervise rs_imu_capture: restart it (fresh process = camera recovery) only when it exits with code 3 (IMU silence).
# Any other exit (SIGINT from launch, error) ends the loop, so `ros2 launch` shutdown is immediate and never respawns.
BIN="$(cd "$(dirname "$0")" && pwd)/rs_imu_capture"   # same install dir as this script (do not follow the symlink-install link)
trap 'kill -INT "$CHILD" 2>/dev/null' INT TERM
while true; do
    "$BIN" "$@" & CHILD=$!
    wait "$CHILD"; rc=$?
    [ "$rc" -eq 3 ] || exit "$rc"
    echo "rs_imu_capture_supervised: capture exited on IMU silence; restarting in 2 s"; sleep 2
done
