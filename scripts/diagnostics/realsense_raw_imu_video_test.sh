#!/usr/bin/env bash
# Run on robot: test RGB-D video with raw accel/gyro topics instead of fused /camera/imu.

ROOT="${HOME}/slam_project"
source /opt/ros/melodic/setup.bash
source "${ROOT}/agv_ws/devel/setup.bash"

stop_rs() {
    rosnode kill /camera/realsense2_camera /camera/realsense2_camera_manager 2>/dev/null || true
    if [ -n "${PID:-}" ]; then
        kill "${PID}" 2>/dev/null || true
    fi
    sleep 2
    pgrep -fal "rs_camera.launch|realsense2_camera_manager|RealSenseNodeFactory" | awk '{print $1}' |
        xargs -r kill 2>/dev/null || true
}

rate() {
    local topic="$1"
    if rostopic list 2>/dev/null | grep -qx "${topic}"; then
        echo "TOPIC ${topic}"
        timeout 10 rostopic hz "${topic}" --window 20 2>&1 | tail -4
    else
        echo "MISSING ${topic}"
    fi
}

stop_rs
LOG=/tmp/rs_raw_imu_video_test.log
roslaunch realsense2_camera rs_camera.launch \
    align_depth:=true \
    enable_pointcloud:=false \
    enable_sync:=false \
    color_width:=640 \
    color_height:=480 \
    color_fps:=15 \
    depth_width:=640 \
    depth_height:=480 \
    depth_fps:=15 \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=none \
    > "${LOG}" 2>&1 &
PID=$!

echo "pid=${PID} log=${LOG}"
sleep 35

echo "== topics =="
rostopic list 2>/dev/null | grep -E '^/camera/(imu|accel|gyro|color/image_raw|aligned_depth_to_color/image_raw)' | sort || true

echo "== rates =="
rate /camera/imu
rate /camera/accel/sample
rate /camera/gyro/sample
rate /camera/color/image_raw
rate /camera/aligned_depth_to_color/image_raw

echo "== logtail =="
grep -Ei "gyro stream|accel stream|Start publisher IMU|Device USB|Sync Mode|control_transfer|warn|error" "${LOG}" | tail -80 || true

stop_rs
