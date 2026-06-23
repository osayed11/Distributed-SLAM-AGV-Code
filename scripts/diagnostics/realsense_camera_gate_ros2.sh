#!/usr/bin/env bash
# RealSense D455 gate for ROS2 dataset collection.
#
# Sequence:
#   USB reset -> rs-enumerate -> bounded ROS2 streaming -> topic-rate checks ->
#   stop camera -> rs-enumerate again.

set -u

ROOT="${SLAM_PROJECT_ROOT:-${HOME}/slam_project}"
LOG_DIR="${LOG_DIR:-${HOME}/agv_diagnostics/realsense_gates}"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${RUN_DIR:-${LOG_DIR}/gate_${STAMP}}"

STREAM_SECONDS="${STREAM_SECONDS:-90}"
STARTUP_WAIT_SECONDS="${STARTUP_WAIT_SECONDS:-18}"
CAMERA_COLOR_WIDTH="${CAMERA_COLOR_WIDTH:-640}"
CAMERA_COLOR_HEIGHT="${CAMERA_COLOR_HEIGHT:-480}"
CAMERA_COLOR_FPS="${CAMERA_COLOR_FPS:-15}"
CAMERA_DEPTH_WIDTH="${CAMERA_DEPTH_WIDTH:-640}"
CAMERA_DEPTH_HEIGHT="${CAMERA_DEPTH_HEIGHT:-480}"
CAMERA_DEPTH_FPS="${CAMERA_DEPTH_FPS:-15}"
ENABLE_REALSENSE_SYNC="${ENABLE_REALSENSE_SYNC:-false}"
MIN_RGBD_HZ="${MIN_RGBD_HZ:-12}"
MIN_CAMERA_IMU_HZ="${MIN_CAMERA_IMU_HZ:-150}"
STRICT_UVC_LOG="${STRICT_UVC_LOG:-false}"
STRICT_POST_ENUM="${STRICT_POST_ENUM:-false}"

FAILURES=0
LAUNCH_PID=""
LAUNCH_PGID=""
DMESG_PID=""

mkdir -p "${RUN_DIR}"

source_ros() {
    set +u
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    elif [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
    else
        echo "FAIL ros: no ROS2 setup found under /opt/ros" | tee -a "${RUN_DIR}/summary.txt"
        exit 1
    fi
    if [ -f "${ROOT}/agv2_ws/install/setup.bash" ]; then
        source "${ROOT}/agv2_ws/install/setup.bash"
    fi
    set -u
}

log_cmd() {
    local name="$1"
    shift
    {
        echo "# command: $*"
        echo "# start: $(date --iso-8601=seconds)"
        "$@"
        local rc=$?
        echo "# exit: ${rc}"
        echo "# end: $(date --iso-8601=seconds)"
        return "${rc}"
    } > "${RUN_DIR}/${name}" 2>&1
}

pass_gate() {
    echo "PASS $1: $2" | tee -a "${RUN_DIR}/summary.txt"
}

fail_gate() {
    echo "FAIL $1: $2" | tee -a "${RUN_DIR}/summary.txt"
    FAILURES=$((FAILURES + 1))
}

warn_gate() {
    echo "WARN $1: $2" | tee -a "${RUN_DIR}/summary.txt"
}

sudo_available() {
    sudo -n true >/dev/null 2>&1
}

write_sysfs() {
    local path="$1"
    local value="$2"

    if [ -w "${path}" ]; then
        echo "${value}" > "${path}" 2>/dev/null
    elif sudo_available; then
        echo "${value}" | sudo -n tee "${path}" >/dev/null 2>&1
    else
        return 1
    fi
}

cleanup() {
    if [ -n "${LAUNCH_PGID}" ]; then
        kill -INT "-${LAUNCH_PGID}" 2>/dev/null || true
        sleep 3
        kill -TERM "-${LAUNCH_PGID}" 2>/dev/null || true
        sleep 2
        kill -KILL "-${LAUNCH_PGID}" 2>/dev/null || true
    elif [ -n "${LAUNCH_PID}" ]; then
        kill -INT "${LAUNCH_PID}" 2>/dev/null || true
        sleep 3
        kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    fi
    if [ -n "${DMESG_PID}" ]; then
        kill "${DMESG_PID}" 2>/dev/null || sudo -n kill "${DMESG_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT

reset_d455() {
    local sysfs bus dev devfile rc

    sysfs="$(for p in /sys/bus/usb/devices/*/idProduct; do
        [ "$(cat "$p" 2>/dev/null)" = "0b5c" ] && dirname "$p" && break
    done)"
    if [ -z "${sysfs}" ]; then
        fail_gate "D455 USB reset" "D455 not found in sysfs"
        return 1
    fi

    bus="$(cat "${sysfs}/busnum")"
    dev="$(cat "${sysfs}/devnum")"
    devfile="$(printf "/dev/bus/usb/%03d/%03d" "${bus}" "${dev}")"

    reset_cmd=(python3 - "${devfile}")
    if [ ! -w "${devfile}" ] && sudo_available; then
        reset_cmd=(sudo -n python3 - "${devfile}")
    fi

    "${reset_cmd[@]}" > "${RUN_DIR}/usb_reset.txt" 2>&1 <<'PY'
import fcntl
import os
import sys

USBDEVFS_RESET = 0x5514
path = sys.argv[1]
fd = os.open(path, os.O_WRONLY)
try:
    fcntl.ioctl(fd, USBDEVFS_RESET, 0)
finally:
    os.close(fd)
print(f"reset {path}")
PY
    rc=$?
    if [ "${rc}" -ne 0 ]; then
        fail_gate "D455 USB reset" "USBDEVFS_RESET failed; see ${RUN_DIR}/usb_reset.txt"
        return 1
    fi

    sleep 8
    sysfs="$(for p in /sys/bus/usb/devices/*/idProduct; do
        [ "$(cat "$p" 2>/dev/null)" = "0b5c" ] && dirname "$p" && break
    done)"
    if [ -n "${sysfs}" ]; then
        write_sysfs "${sysfs}/power/control" on || true
        write_sysfs "${sysfs}/power/autosuspend" -1 || true
    fi
    pass_gate "D455 USB reset" "USB reset sent and autosuspend disabled"
}

record_enum_issue() {
    local label="$1"
    local message="$2"
    local strict="$3"

    if [ "${strict}" = true ]; then
        fail_gate "rs-enumerate ${label}" "${message}"
    else
        warn_gate "rs-enumerate ${label}" "${message}"
    fi
}

check_rs_enumerate() {
    local label="$1"
    local strict="${2:-true}"
    local outfile="${RUN_DIR}/rs_enumerate_${label}.txt"
    local rc

    if ! command -v rs-enumerate-devices >/dev/null 2>&1; then
        record_enum_issue "${label}" "rs-enumerate-devices is missing" "${strict}"
        return 1
    fi

    timeout 25 rs-enumerate-devices -s > "${outfile}" 2>&1
    rc=$?
    if [ "${rc}" -ne 0 ]; then
        record_enum_issue "${label}" "failed; see ${outfile}" "${strict}"
        return 1
    fi
    if grep -q "Intel RealSense D455" "${outfile}"; then
        pass_gate "rs-enumerate ${label}" "D455 detected"
    else
        record_enum_issue "${label}" "D455 not detected in ${outfile}" "${strict}"
        return 1
    fi
}

rate_from_log() {
    grep "average rate" "$1" | tail -1 | awk -F': ' '{print $2}' | awk '{print $1}'
}

check_rate() {
    local topic="$1"
    local file="$2"
    local min_rate="$3"
    local label="$4"
    local rate

    rate="$(rate_from_log "${file}")"
    if [ -z "${rate}" ]; then
        fail_gate "${label}" "no average rate for ${topic}; see ${file}"
        return
    fi
    if awk -v rate="${rate}" -v min="${min_rate}" 'BEGIN { exit(rate >= min ? 0 : 1) }'; then
        pass_gate "${label}" "${topic} ${rate} Hz"
    else
        fail_gate "${label}" "${topic} ${rate} Hz, expected >= ${min_rate} Hz"
    fi
}

source_ros

{
    echo "run_dir=${RUN_DIR}"
    echo "stream_seconds=${STREAM_SECONDS}"
    echo "ros_domain_id=${ROS_DOMAIN_ID:-unset}"
    echo "color_profile=${CAMERA_COLOR_WIDTH}x${CAMERA_COLOR_HEIGHT}x${CAMERA_COLOR_FPS}"
    echo "depth_profile=${CAMERA_DEPTH_WIDTH}x${CAMERA_DEPTH_HEIGHT}x${CAMERA_DEPTH_FPS}"
    echo "enable_sync=${ENABLE_REALSENSE_SYNC}"
    echo "strict_uvc_log=${STRICT_UVC_LOG}"
    echo "strict_post_enum=${STRICT_POST_ENUM}"
} | tee "${RUN_DIR}/summary.txt"

log_cmd usb_tree_before.txt lsusb -t || true
log_cmd vcgencmd_before.txt vcgencmd get_throttled || true

reset_d455 || true
check_rs_enumerate pre true || true

if sudo_available; then
    sudo -n dmesg -wT > "${RUN_DIR}/dmesg_watch.txt" 2>&1 &
    DMESG_PID=$!
else
    dmesg -wT > "${RUN_DIR}/dmesg_watch.txt" 2>&1 &
    DMESG_PID=$!
    sleep 1
    if ! kill -0 "${DMESG_PID}" 2>/dev/null; then
        warn_gate "kernel log" "dmesg unavailable without sudo"
        DMESG_PID=""
    fi
fi

COLOR_PROFILE="${CAMERA_COLOR_WIDTH}x${CAMERA_COLOR_HEIGHT}x${CAMERA_COLOR_FPS}"
DEPTH_PROFILE="${CAMERA_DEPTH_WIDTH}x${CAMERA_DEPTH_HEIGHT}x${CAMERA_DEPTH_FPS}"

setsid ros2 launch realsense2_camera rs_launch.py \
    camera_name:=camera \
    camera_namespace:=/ \
    align_depth.enable:=true \
    pointcloud.enable:=false \
    enable_sync:="${ENABLE_REALSENSE_SYNC}" \
    rgb_camera.color_profile:="${COLOR_PROFILE}" \
    depth_module.depth_profile:="${DEPTH_PROFILE}" \
    depth_module.infra_profile:="${DEPTH_PROFILE}" \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=2 \
    enable_infra1:=false \
    enable_infra2:=false \
    initial_reset:=false \
    > "${RUN_DIR}/realsense_launch.log" 2>&1 &
LAUNCH_PID=$!
sleep 1
LAUNCH_PGID="$(ps -o pgid= -p "${LAUNCH_PID}" 2>/dev/null | tr -d ' ')"

sleep "${STARTUP_WAIT_SECONDS}"

timeout "${STREAM_SECONDS}" ros2 topic hz /camera/color/image_raw --window 40 \
    > "${RUN_DIR}/hz_color.txt" 2>&1 &
HZ_COLOR_PID=$!
timeout "${STREAM_SECONDS}" ros2 topic hz /camera/aligned_depth_to_color/image_raw --window 40 \
    > "${RUN_DIR}/hz_aligned_depth.txt" 2>&1 &
HZ_DEPTH_PID=$!
timeout "${STREAM_SECONDS}" ros2 topic hz /camera/imu --window 80 \
    > "${RUN_DIR}/hz_camera_imu.txt" 2>&1 &
HZ_IMU_PID=$!

wait "${HZ_COLOR_PID}" 2>/dev/null || true
wait "${HZ_DEPTH_PID}" 2>/dev/null || true
wait "${HZ_IMU_PID}" 2>/dev/null || true

check_rate /camera/color/image_raw "${RUN_DIR}/hz_color.txt" "${MIN_RGBD_HZ}" "color stream"
check_rate /camera/aligned_depth_to_color/image_raw "${RUN_DIR}/hz_aligned_depth.txt" "${MIN_RGBD_HZ}" "aligned depth stream"
check_rate /camera/imu "${RUN_DIR}/hz_camera_imu.txt" "${MIN_CAMERA_IMU_HZ}" "camera imu stream"

cleanup
trap - EXIT
sleep 4

check_rs_enumerate post "${STRICT_POST_ENUM}" || true
log_cmd usb_tree_after.txt lsusb -t || true
log_cmd vcgencmd_after.txt vcgencmd get_throttled || true

if grep -Eiq "The device has been disconnected|USB disconnect|No such device|Device or resource busy|device removed" \
    "${RUN_DIR}/realsense_launch.log" "${RUN_DIR}/dmesg_watch.txt" 2>/dev/null; then
    fail_gate "RealSense runtime log" "camera disconnect/device-drop errors observed"
elif grep -Eiq "UVCIOC_CTRL_QUERY|VIDIOC_|Frames didn't arrived|control_transfer.*failed|Connection timed out|Failed to create device|set_xu" \
    "${RUN_DIR}/realsense_launch.log" "${RUN_DIR}/dmesg_watch.txt" 2>/dev/null; then
    if [ "${STRICT_UVC_LOG}" = true ]; then
        fail_gate "RealSense runtime log" "UVC/frame/disconnect errors observed"
    else
        warn_gate "RealSense runtime log" "UVC/control timeout text observed; stream rates decide pass/fail"
    fi
else
    pass_gate "RealSense runtime log" "no UVC/control timeout text observed"
fi

if [ "${FAILURES}" -eq 0 ]; then
    echo "REALSENSE CAMERA GATE PASS" | tee -a "${RUN_DIR}/summary.txt"
    exit 0
fi

echo "REALSENSE CAMERA GATE FAIL: ${FAILURES} failure(s)" | tee -a "${RUN_DIR}/summary.txt"
exit 1
