#!/usr/bin/env bash
# Run on the robot. Tests RealSense IMU publication under a few launch modes.

ROOT="${HOME}/slam_project"
source /opt/ros/melodic/setup.bash
source "${ROOT}/agv_ws/devel/setup.bash"

set -u

check_hz() {
    local topic="$1"
    local timeout_sec="${2:-12}"
    local window="${3:-30}"
    local out
    local line

    out=$(timeout "${timeout_sec}" rostopic hz "${topic}" --window "${window}" 2>&1 || true)
    line=$(printf "%s\n" "${out}" | grep "average rate" | tail -1 || true)
    if [ -n "${line}" ]; then
        echo "PASS ${topic}: ${line}"
    else
        echo "FAIL ${topic}: no average rate within ${timeout_sec}s"
        printf "%s\n" "${out}" | tail -5
    fi
}

stop_ros() {
    rosnode kill /camera/realsense2_camera /camera/realsense2_camera_manager 2>/dev/null || true
    if [ -n "${RS_PID:-}" ]; then
        kill "${RS_PID}" 2>/dev/null || true
    fi
    sleep 3
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
    local log="/tmp/realsense_${name}_$(date +%Y%m%d_%H%M%S).log"

    echo ""
    echo "== ${name} =="
    stop_ros
    roslaunch realsense2_camera rs_camera.launch "$@" > "${log}" 2>&1 &
    RS_PID=$!
    echo "pid=${RS_PID}"
    echo "log=${log}"
    sleep 25

    echo "-- topics --"
    rostopic list 2>/dev/null | grep '^/camera/' | sort || true

    echo "-- rates --"
    check_hz /camera/imu 12 40
    check_hz /camera/gyro/sample 8 30
    check_hz /camera/accel/sample 8 20
    check_hz /camera/color/image_raw 8 15
    check_hz /camera/aligned_depth_to_color/image_raw 8 15

    echo "-- imu log --"
    grep -Ei "imu|gyro|accel|sync|warn|error|Device USB|Start publisher|stream is enabled|hid|control_transfer" "${log}" | tail -80 || true
    stop_ros
}

echo "== precheck =="
lsusb -t || true
rospack find realsense2_camera || true

run_case imu_only_reset \
    enable_color:=false \
    enable_depth:=false \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=linear_interpolation \
    initial_reset:=true

run_case rgbd_imu_no_sync_reset \
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
    unite_imu_method:=linear_interpolation \
    initial_reset:=true

run_case rgbd_imu_copy_no_sync \
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
    unite_imu_method:=copy \
    initial_reset:=true

echo ""
echo "== final cleanup =="
stop_ros
pgrep -fal "roslaunch|rosmaster|roscore|realsense|ydlidar|myagv|apriltag" || true
