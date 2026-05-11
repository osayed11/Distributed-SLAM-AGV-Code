#!/usr/bin/env bash
# Run on the robot. Checks whether lower-load or partial RealSense video modes
# allow /camera/imu to publish together with images.

ROOT="${HOME}/slam_project"
source /opt/ros/melodic/setup.bash
source "${ROOT}/agv_ws/devel/setup.bash"

set -u

check_hz() {
    local topic="$1"
    local timeout_sec="${2:-10}"
    local out
    local line

    if ! rostopic list 2>/dev/null | grep -qx "${topic}"; then
        echo "SKIP ${topic}: not registered"
        return
    fi

    out=$(timeout "${timeout_sec}" rostopic hz "${topic}" --window 20 2>&1 || true)
    line=$(printf "%s\n" "${out}" | grep "average rate" | tail -1 || true)
    if [ -n "${line}" ]; then
        echo "PASS ${topic}: ${line}"
    else
        echo "FAIL ${topic}: no average rate within ${timeout_sec}s"
        printf "%s\n" "${out}" | tail -3
    fi
}

stop_rs() {
    rosnode kill /camera/realsense2_camera /camera/realsense2_camera_manager 2>/dev/null || true
    if [ -n "${RS_PID:-}" ]; then
        kill "${RS_PID}" 2>/dev/null || true
    fi
    sleep 2
    pgrep -fal "rs_camera.launch|realsense2_camera_manager|RealSenseNodeFactory" | awk '{print $1}' |
        while read -r pid; do
            if [ "${pid}" != "$$" ] && [ -n "${pid}" ]; then
                kill "${pid}" 2>/dev/null || true
            fi
        done
    sleep 1
}

run_case() {
    local name="$1"
    shift
    local log="/tmp/realsense_reduced_${name}_$(date +%Y%m%d_%H%M%S).log"

    echo ""
    echo "== ${name} =="
    stop_rs
    roslaunch realsense2_camera rs_camera.launch "$@" > "${log}" 2>&1 &
    RS_PID=$!
    echo "pid=${RS_PID}"
    echo "log=${log}"
    sleep 40
    echo "-- rates --"
    check_hz /camera/imu 12
    check_hz /camera/color/image_raw 8
    check_hz /camera/depth/image_rect_raw 8
    check_hz /camera/aligned_depth_to_color/image_raw 8
    echo "-- log --"
    grep -Ei "Device USB|Sync Mode|depth stream|color stream|gyro stream|accel stream|Start publisher IMU|control_transfer|warn|error" "${log}" | tail -60 || true
    stop_rs
}

echo "== precheck =="
lsusb -t || true

run_case rgbd_640_no_align \
    align_depth:=false \
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
    unite_imu_method:=linear_interpolation

run_case rgbd_424_align \
    align_depth:=true \
    enable_pointcloud:=false \
    enable_sync:=false \
    color_width:=424 \
    color_height:=240 \
    color_fps:=15 \
    depth_width:=424 \
    depth_height:=240 \
    depth_fps:=15 \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=linear_interpolation

run_case color_only_imu \
    enable_depth:=false \
    enable_color:=true \
    color_width:=640 \
    color_height:=480 \
    color_fps:=15 \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=linear_interpolation

run_case depth_only_imu \
    enable_color:=false \
    enable_depth:=true \
    depth_width:=640 \
    depth_height:=480 \
    depth_fps:=15 \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=linear_interpolation

echo ""
echo "== final cleanup =="
stop_rs
pgrep -fal "roslaunch|rosmaster|roscore|realsense" || true
