#!/usr/bin/env bash
# Supervise rs_imu_capture: restart it (fresh process = camera recovery) only when it exits with code 3 (IMU silence).
# Any other exit (SIGINT from launch, error) ends the loop, so `ros2 launch` shutdown is immediate and never respawns.
# RS_IMU_CAPTURE_BIN selects the capture binary in the same install dir: rs_imu_capture (librealsense, default) or
# rs_imu_iio_capture (reads the kernel IIO devices directly; no librealsense streaming). Inherited from the environment.
BIN="$(cd "$(dirname "$0")" && pwd)/${RS_IMU_CAPTURE_BIN:-rs_imu_capture}"
trap 'kill -INT "$CHILD" 2>/dev/null' INT TERM
while true; do
    "$BIN" "$@" & CHILD=$!
    wait "$CHILD"; rc=$?
    [ "$rc" -eq 3 ] || exit "$rc"
    echo "rs_imu_capture_supervised: capture exited on IMU silence; restarting in 2 s"; sleep 2
done
