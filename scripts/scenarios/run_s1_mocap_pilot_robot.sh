#!/usr/bin/env bash
# Deterministic Scenario 1 MoCap circle collection for one ROS 2 AGV.
#
# This wrapper intentionally keeps the lifecycle simple:
#   1. clean stale local robot ROS processes
#   2. reset the D455 once and disable autosuspend
#   3. launch exactly one bringup
#   4. wait for required live topics
#   5. record one bag
#   6. drive one MoCap-feedback circle using best-effort MoCap QoS
#   7. stop recording/bringup and validate the bag

set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  bash scripts/scenarios/run_s1_mocap_pilot_robot.sh <robot_name> [scenario_name]

Typical agv102 lab run:
  cd ~/slam_project
  ROS_DOMAIN_ID=0 \
  MOCAP_TOPIC=/optitrack/rigid_bodies/orkar_agv102 \
  CMD_TOPIC=/agv102/cmd_vel \
  S1_RADIUS=1.0 \
  S1_DURATION=70 \
  bash scripts/scenarios/run_s1_mocap_pilot_robot.sh agv102 s1_circle_1m

Required/important environment:
  MOCAP_TOPIC       MoCap PoseStamped topic. Example: /optitrack/rigid_bodies/orkar_agv102
  CMD_TOPIC         Namespaced cmd_vel topic. Default: /<robot_name>/cmd_vel
  ROS_DOMAIN_ID     ROS 2 domain. Default: 0

Circle overrides:
  S1_RADIUS         Circle radius in metres. Default: 1.0
  S1_CENTER_X       Optional MoCap-frame center x. If omitted, center is inferred.
  S1_CENTER_Y       Optional MoCap-frame center y. If omitted, center is inferred.
  S1_DURATION       Motion duration in seconds. Default: 70
  S1_LINEAR         Linear speed. Default: 0.10
  S1_MIN_LINEAR     Minimum linear speed while correcting. Default: 0.07
  S1_DIRECTION      ccw or cw. Default: ccw
  S1_POSE_TIMEOUT   Abort if MoCap pose is stale this long. Default: 2.5
  S1_BEST_EFFORT_POSE true/false. Default: false for reliable OptiTrack DDS streams.
  S1_DRY_RUN       true/false. Default: false. Proves lifecycle without publishing motion.

Recording/gates:
  S1_RECORD         true/false. Default: true
  S1_VALIDATE       true/false. Default: true
  S1_STORAGE_ID     mcap/sqlite3/auto. Default: auto, prefer MCAP when installed.
  S1_TOPIC_WAIT_SEC Wait per required topic. Default: 180
  REQUIRE_GT        Require MoCap in validator. Default: true
  REQUIRE_IMU       Require raw D455 gyro+accel in validator. Default: true
EOF
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    usage
    exit 0
fi

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    usage >&2
    exit 2
fi

ROBOT_NAME="$1"
SCENARIO="${2:-s1_mocap_circle}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${ROOT}"

source_ros2() {
    local restore_nounset=false
    case "$-" in
        *u*)
            restore_nounset=true
            set +u
            ;;
    esac
    if [ -f /opt/ros/humble/setup.bash ]; then
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
    elif [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        # shellcheck disable=SC1090
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
    else
        echo "ERROR: ROS 2 setup not found under /opt/ros." >&2
        exit 1
    fi
    if [ -f "${ROOT}/agv2_ws/install/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "${ROOT}/agv2_ws/install/setup.bash"
    fi
    if [ "${restore_nounset}" = true ]; then
        set -u
    fi
}

require_nonempty() {
    local name="$1"
    local value="$2"
    if [ -z "${value}" ]; then
        echo "ERROR: ${name} is required." >&2
        usage >&2
        exit 2
    fi
}

bool_true() {
    case "${1:-}" in
        1|true|TRUE|yes|YES|on|ON) return 0 ;;
        *) return 1 ;;
    esac
}

sudo_sh() {
    if sudo -n true 2>/dev/null; then
        sudo sh -c "$1"
    else
        printf '%s\n' "${SUDO_PASSWORD:-ubuntu}" | sudo -S sh -c "$1"
    fi
}

sudo_python_usb_reset() {
    local devfile="$1"
    if sudo -n true 2>/dev/null; then
        sudo python3 - "${devfile}" <<'PY'
import fcntl
import os
import sys

USBDEVFS_RESET = 21780
fd = os.open(sys.argv[1], os.O_WRONLY)
try:
    fcntl.ioctl(fd, USBDEVFS_RESET, 0)
finally:
    os.close(fd)
PY
    else
        printf '%s\n' "${SUDO_PASSWORD:-ubuntu}" | sudo -S python3 - "${devfile}" <<'PY'
import fcntl
import os
import sys

USBDEVFS_RESET = 21780
fd = os.open(sys.argv[1], os.O_WRONLY)
try:
    fcntl.ioctl(fd, USBDEVFS_RESET, 0)
finally:
    os.close(fd)
PY
    fi
}

find_d455_sysfs() {
    local d
    for d in /sys/bus/usb/devices/*; do
        [ -r "${d}/idVendor" ] || continue
        [ -r "${d}/idProduct" ] || continue
        if [ "$(cat "${d}/idVendor" 2>/dev/null)" = "8086" ] && \
           [ "$(cat "${d}/idProduct" 2>/dev/null)" = "0b5c" ]; then
            printf '%s\n' "${d}"
            return 0
        fi
    done
    return 1
}

reset_d455_once() {
    local sysfs="${1:-}"
    if [ -z "${sysfs}" ]; then
        sysfs="$(find_d455_sysfs || true)"
    fi
    if [ -z "${sysfs}" ]; then
        echo "WARN: D455 not found in USB sysfs; continuing to topic gate." >&2
        return 0
    fi

    local bus dev devfile
    bus="$(cat "${sysfs}/busnum" 2>/dev/null || true)"
    dev="$(cat "${sysfs}/devnum" 2>/dev/null || true)"
    if [ -n "${bus}" ] && [ -n "${dev}" ]; then
        devfile="$(printf '/dev/bus/usb/%03d/%03d' "${bus}" "${dev}")"
        echo "Resetting D455 (${devfile})..."
        if sudo_python_usb_reset "${devfile}"; then
            echo "  D455 USB reset sent."
            sleep "${D455_POST_RESET_SLEEP_SEC:-8}"
        else
            echo "WARN: D455 USB reset failed; continuing to topic gate." >&2
        fi
    fi

    sysfs="$(find_d455_sysfs || true)"
    if [ -n "${sysfs}" ]; then
        sudo_sh "[ -f '${sysfs}/power/control' ] && echo on > '${sysfs}/power/control' || true; [ -f '${sysfs}/power/autosuspend_delay_ms' ] && echo -1 > '${sysfs}/power/autosuspend_delay_ms' || true; [ -f '${sysfs}/power/autosuspend' ] && echo -1 > '${sysfs}/power/autosuspend' || true"
        echo "  D455 autosuspend disabled."
    fi
}

kill_local_robot_graph() {
    echo "Cleaning stale local robot ROS processes..."
    local patterns=(
        "drive_mocap_circle_ros2.py"
        "ros2 bag record"
        "ros2 launch agv_bringup"
        "realsense2_camera_node"
        "ydlidar_ros2_driver_node"
        "myagv_odometry_node"
        "static_transform_publisher"
    )
    local pattern
    for pattern in "${patterns[@]}"; do
        pkill -TERM -f "${pattern}" 2>/dev/null || true
    done
    sleep 3
    for pattern in "${patterns[@]}"; do
        pkill -KILL -f "${pattern}" 2>/dev/null || true
    done
    rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* 2>/dev/null || true
}

wait_for_topic_once() {
    local topic="$1"
    local timeout_sec="$2"
    echo "Waiting for ${topic}..."
    timeout "${timeout_sec}" bash -lc \
        "source /opt/ros/humble/setup.bash; [ -f '${ROOT}/agv2_ws/install/setup.bash' ] && source '${ROOT}/agv2_ws/install/setup.bash'; export ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-0}'; until ros2 topic echo --once '${topic}' >/dev/null 2>&1; do sleep 1; done"
}

stop_process_group() {
    local pid="${1:-}"
    local signal_name="${2:-INT}"
    [ -n "${pid}" ] || return 0
    kill "-${signal_name}" "-${pid}" 2>/dev/null || kill "-${signal_name}" "${pid}" 2>/dev/null || true
}

wait_for_exit_or_kill() {
    local pid="$1"
    local label="$2"
    local timeout_sec="$3"
    local elapsed=0
    while kill -0 "${pid}" 2>/dev/null && [ "${elapsed}" -lt "${timeout_sec}" ]; do
        sleep 1
        elapsed=$((elapsed + 1))
    done
    if kill -0 "${pid}" 2>/dev/null; then
        echo "WARN: ${label} did not exit after ${timeout_sec}s; sending SIGTERM." >&2
        stop_process_group "${pid}" TERM
        sleep 5
    fi
    if kill -0 "${pid}" 2>/dev/null; then
        echo "WARN: ${label} still alive; sending SIGKILL." >&2
        stop_process_group "${pid}" KILL
    fi
}

publish_zero() {
    python3 - "${CMD_TOPIC}" <<'PY' || true
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

topic = sys.argv[1]
rclpy.init()
node = rclpy.create_node("s1_mocap_circle_zero_cmd")
pub = node.create_publisher(Twist, topic, 10)
msg = Twist()
end = time.time() + 1.0
while time.time() < end:
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.02)
node.destroy_node()
rclpy.shutdown()
PY
}

MOCAP_TOPIC="${MOCAP_TOPIC:-}"
CMD_TOPIC="${CMD_TOPIC:-/${ROBOT_NAME}/cmd_vel}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
S1_RADIUS="${S1_RADIUS:-1.0}"
S1_DURATION="${S1_DURATION:-70}"
S1_LINEAR="${S1_LINEAR:-0.10}"
S1_MIN_LINEAR="${S1_MIN_LINEAR:-0.07}"
S1_DIRECTION="${S1_DIRECTION:-ccw}"
S1_MAX_RADIUS_ERROR="${S1_MAX_RADIUS_ERROR:-0.45}"
S1_FORWARD_YAW_OFFSET_DEG="${S1_FORWARD_YAW_OFFSET_DEG:-0}"
S1_POSE_TIMEOUT="${S1_POSE_TIMEOUT:-2.5}"
S1_HEADING_KP="${S1_HEADING_KP:-1.10}"
S1_RADIUS_KP="${S1_RADIUS_KP:-1.50}"
S1_MAX_ANGULAR="${S1_MAX_ANGULAR:-0.55}"
S1_MAX_RADIUS_HEADING_OFFSET_DEG="${S1_MAX_RADIUS_HEADING_OFFSET_DEG:-35}"
S1_BEST_EFFORT_POSE="${S1_BEST_EFFORT_POSE:-false}"
S1_DRY_RUN="${S1_DRY_RUN:-false}"
S1_RECORD="${S1_RECORD:-true}"
S1_VALIDATE="${S1_VALIDATE:-true}"
S1_STORAGE_ID="${S1_STORAGE_ID:-auto}"
S1_TOPIC_WAIT_SEC="${S1_TOPIC_WAIT_SEC:-180}"
S1_RECORDER_STOP_TIMEOUT="${S1_RECORDER_STOP_TIMEOUT:-120}"
S1_BRINGUP_STOP_TIMEOUT="${S1_BRINGUP_STOP_TIMEOUT:-30}"
S1_POST_ROLL_SEC="${S1_POST_ROLL_SEC:-5}"
S1_MIN_BAG_DURATION="${S1_MIN_BAG_DURATION:-30}"
REQUIRE_GT="${REQUIRE_GT:-true}"
REQUIRE_IMU="${REQUIRE_IMU:-true}"
REQUIRE_RESILIENT_STORAGE="${REQUIRE_RESILIENT_STORAGE:-false}"

require_nonempty "MOCAP_TOPIC" "${MOCAP_TOPIC}"
if [ "${CMD_TOPIC}" = "/cmd_vel" ]; then
    echo "ERROR: refusing to run S1 pilot on root /cmd_vel. Use /${ROBOT_NAME}/cmd_vel." >&2
    exit 2
fi

source_ros2
mkdir -p "${HOME}/agv_data"

STAMP="$(date +%Y%m%d_%H%M%S)"
SESSION_ID="${ROBOT_NAME}_${SCENARIO}_${STAMP}"
BAG="${HOME}/agv_data/${SESSION_ID}"
BRINGUP_LOG="${HOME}/agv_data/${SESSION_ID}_bringup.log"
RECORD_LOG="${HOME}/agv_data/${SESSION_ID}_record.log"
SUMMARY_JSON="${HOME}/agv_data/${SESSION_ID}_circle_summary.json"
VALIDATION_JSON="${HOME}/agv_data/${SESSION_ID}_validate.json"

BRINGUP_PID=""
REC_PID=""

cleanup() {
    set +e
    publish_zero
    if [ -n "${REC_PID}" ] && kill -0 "${REC_PID}" 2>/dev/null; then
        echo "Stopping ros2 bag record..."
        stop_process_group "${REC_PID}" INT
        wait_for_exit_or_kill "${REC_PID}" "ros2 bag record" "${S1_RECORDER_STOP_TIMEOUT}"
    fi
    if [ -n "${BRINGUP_PID}" ] && kill -0 "${BRINGUP_PID}" 2>/dev/null; then
        echo "Stopping bringup..."
        stop_process_group "${BRINGUP_PID}" INT
        wait_for_exit_or_kill "${BRINGUP_PID}" "bringup" "${S1_BRINGUP_STOP_TIMEOUT}"
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

echo "========================================================================"
echo "S1 MOCAP CIRCLE COLLECTION"
echo "robot:        ${ROBOT_NAME}"
echo "scenario:     ${SCENARIO}"
echo "session:      ${SESSION_ID}"
echo "ros_domain:   ${ROS_DOMAIN_ID}"
echo "mocap_topic:  ${MOCAP_TOPIC}"
echo "cmd_topic:    ${CMD_TOPIC}"
echo "bag:          ${BAG}"
echo "radius:       ${S1_RADIUS} m"
echo "duration:     ${S1_DURATION} s"
echo "linear:       ${S1_LINEAR} m/s"
echo "pose_qos:     $(bool_true "${S1_BEST_EFFORT_POSE}" && echo best_effort || echo reliable)"
echo "dry_run:      ${S1_DRY_RUN}"
echo "storage:      ${S1_STORAGE_ID}"
echo "========================================================================"

kill_local_robot_graph
reset_d455_once

echo "Starting one bringup..."
setsid bash -lc \
    "source /opt/ros/humble/setup.bash; [ -f '${ROOT}/agv2_ws/install/setup.bash' ] && source '${ROOT}/agv2_ws/install/setup.bash'; export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'; exec ros2 launch agv_bringup bringup.launch.py agv_serial_port:=/dev/ttyACM0 agv_color_profile:=640x480x15 agv_depth_profile:=640x480x15 enable_sync:=false initial_reset:=false agv_cmd_vel_topic:='${CMD_TOPIC}'" \
    >"${BRINGUP_LOG}" 2>&1 < /dev/null &
BRINGUP_PID=$!
echo "bringup_pid: ${BRINGUP_PID}"

required_topics=(
    "/scan"
    "/odom"
    "/camera/color/image_raw"
    "/camera/depth/image_rect_raw"
    "/camera/gyro/sample"
    "/camera/accel/sample"
    "${MOCAP_TOPIC}"
)

for topic in "${required_topics[@]}"; do
    if ! wait_for_topic_once "${topic}" "${S1_TOPIC_WAIT_SEC}"; then
        echo "ERROR: required topic did not publish: ${topic}" >&2
        echo "--- bringup log tail ---" >&2
        tail -120 "${BRINGUP_LOG}" >&2 || true
        exit 1
    fi
done
echo "Required live topic gate passed."

CIRCLE_ARGS=(
    --pose-topic "${MOCAP_TOPIC}"
    --cmd-topic "${CMD_TOPIC}"
    --radius "${S1_RADIUS}"
    --duration "${S1_DURATION}"
    --linear "${S1_LINEAR}"
    --min-linear "${S1_MIN_LINEAR}"
    --heading-kp "${S1_HEADING_KP}"
    --radius-kp "${S1_RADIUS_KP}"
    --max-angular "${S1_MAX_ANGULAR}"
    --max-radius-heading-offset-deg "${S1_MAX_RADIUS_HEADING_OFFSET_DEG}"
    --pose-timeout "${S1_POSE_TIMEOUT}"
    --max-radius-error "${S1_MAX_RADIUS_ERROR}"
    --forward-yaw-offset-deg "${S1_FORWARD_YAW_OFFSET_DEG}"
    --summary-json "${SUMMARY_JSON}"
    --yes
    --verbose
)

if bool_true "${S1_BEST_EFFORT_POSE}"; then
    CIRCLE_ARGS+=(--best-effort-pose)
fi
if bool_true "${S1_DRY_RUN}"; then
    CIRCLE_ARGS+=(--dry-run)
fi

if [ -n "${S1_CENTER_X:-}" ] || [ -n "${S1_CENTER_Y:-}" ]; then
    require_nonempty "S1_CENTER_X" "${S1_CENTER_X:-}"
    require_nonempty "S1_CENTER_Y" "${S1_CENTER_Y:-}"
    CIRCLE_ARGS+=(--center-x "${S1_CENTER_X}" --center-y "${S1_CENTER_Y}")
fi

case "${S1_DIRECTION}" in
    ccw|counter-clockwise|counterclockwise)
        CIRCLE_ARGS+=(--counter-clockwise)
        ;;
    cw|clockwise)
        CIRCLE_ARGS+=(--clockwise)
        ;;
    *)
        echo "ERROR: S1_DIRECTION must be ccw or cw, got: ${S1_DIRECTION}" >&2
        exit 2
        ;;
esac

DRIVE_RC=0
VALIDATE_RC=0

if bool_true "${S1_RECORD}"; then
    STORAGE_ID="${S1_STORAGE_ID}"
    if [ "${STORAGE_ID}" = "auto" ]; then
        if ros2 bag record --help 2>/dev/null | grep -Eq -- "-s .*mcap|--storage .*mcap|\\bmcap\\b"; then
            STORAGE_ID="mcap"
        else
            STORAGE_ID="sqlite3"
        fi
    fi

    case "${STORAGE_ID}" in
        mcap|sqlite3) ;;
        *)
            echo "ERROR: S1_STORAGE_ID must be auto, mcap, or sqlite3; got ${S1_STORAGE_ID}" >&2
            exit 2
            ;;
    esac

    ROS2_STORAGE_ARGS=(-s "${STORAGE_ID}")
    if [ "${STORAGE_ID}" = "sqlite3" ] && [ -f "${ROOT}/configs/sqlite_resilient.yaml" ]; then
        ROS2_STORAGE_ARGS+=(--storage-config-file "${ROOT}/configs/sqlite_resilient.yaml")
    fi
    if [ -f "${ROOT}/configs/rosbag2_sensor_qos.yaml" ]; then
        ROS2_STORAGE_ARGS+=(--qos-profile-overrides-path "${ROOT}/configs/rosbag2_sensor_qos.yaml")
    fi

    RECORD_TOPICS=(
        "/scan"
        "/odom"
        "${CMD_TOPIC}"
        "/tf"
        "/tf_static"
        "/camera/color/image_raw"
        "/camera/color/camera_info"
        "/camera/depth/image_rect_raw"
        "/camera/depth/camera_info"
        "/camera/extrinsics/depth_to_color"
        "/camera/extrinsics/depth_to_gyro"
        "/camera/extrinsics/depth_to_accel"
        "/camera/imu"
        "/camera/gyro/sample"
        "/camera/accel/sample"
        "/imu"
        "/diagnostics"
        "${MOCAP_TOPIC}"
        "/mocap"
    )

    echo "Starting detached ros2 bag record (${STORAGE_ID})..."
    setsid ros2 bag record \
        --max-cache-size "${ROSBAG2_MAX_CACHE_SIZE:-1073741824}" \
        "${ROS2_STORAGE_ARGS[@]}" \
        -o "${BAG}" \
        "${RECORD_TOPICS[@]}" \
        >"${RECORD_LOG}" 2>&1 < /dev/null &
    REC_PID=$!
    echo "record_pid: ${REC_PID}"
    sleep 5
fi

set +e
python3 scripts/logging/drive_mocap_circle_ros2.py "${CIRCLE_ARGS[@]}"
DRIVE_RC=$?
set -e

sleep "${S1_POST_ROLL_SEC}"
publish_zero

if bool_true "${S1_RECORD}"; then
    echo "Stopping ros2 bag record..."
    stop_process_group "${REC_PID}" INT
    wait_for_exit_or_kill "${REC_PID}" "ros2 bag record" "${S1_RECORDER_STOP_TIMEOUT}"
    REC_PID=""

    echo "--- record log tail ---"
    tail -80 "${RECORD_LOG}" || true
    echo "--- bag info ---"
    ros2 bag info "${BAG}" | sed -n '1,180p' || true
fi

echo "Stopping bringup..."
stop_process_group "${BRINGUP_PID}" INT
wait_for_exit_or_kill "${BRINGUP_PID}" "bringup" "${S1_BRINGUP_STOP_TIMEOUT}"
BRINGUP_PID=""

if bool_true "${S1_RECORD}" && bool_true "${S1_VALIDATE}"; then
    echo "--- bag validation ---"
    VALIDATE_ARGS=(--min-duration "${S1_MIN_BAG_DURATION}" --json-out "${VALIDATION_JSON}")
    if bool_true "${REQUIRE_GT}"; then
        VALIDATE_ARGS+=(--require-gt)
    fi
    if bool_true "${REQUIRE_IMU}"; then
        VALIDATE_ARGS+=(--require-imu)
    fi
    if bool_true "${REQUIRE_RESILIENT_STORAGE}"; then
        VALIDATE_ARGS+=(--require-resilient-storage)
    fi
    set +e
    CMD_TOPIC="${CMD_TOPIC}" \
    MOCAP_TOPIC="${MOCAP_TOPIC}" \
    REQUIRE_GT="${REQUIRE_GT}" \
    REQUIRE_IMU="${REQUIRE_IMU}" \
    DEPTH_TOPIC="/camera/depth/image_rect_raw" \
    DEPTH_INFO_TOPIC="/camera/depth/camera_info" \
    IMU_TOPICS="/camera/gyro/sample /camera/accel/sample /camera/imu /imu" \
    python3 scripts/logging/validate_ros2_bag.py "${BAG}" "${VALIDATE_ARGS[@]}"
    VALIDATE_RC=$?
    set -e
fi

trap - EXIT INT TERM

echo "========================================================================"
echo "S1 COMPLETE"
echo "drive_rc:      ${DRIVE_RC}"
echo "validate_rc:   ${VALIDATE_RC}"
echo "bag:           ${BAG}"
echo "summary_json:  ${SUMMARY_JSON}"
echo "validate_json: ${VALIDATION_JSON}"
echo "========================================================================"

if [ "${DRIVE_RC}" -ne 0 ]; then
    exit "${DRIVE_RC}"
fi
if bool_true "${S1_VALIDATE}" && [ "${VALIDATE_RC}" -ne 0 ]; then
    exit "${VALIDATE_RC}"
fi
exit 0
