#!/usr/bin/env bash
# Deterministic Scenario 1 MoCap circle collection for one ROS 2 AGV.
#
# This wrapper intentionally keeps the lifecycle simple:
#   1. clean stale local robot ROS processes
#   2. reset the D455 once and disable autosuspend
#   3. launch exactly one bringup
#   4. wait for required live topics
#   5. record one logical dataset using stock rosbag2 recorders
#   6. drive one MoCap-feedback circle using best-effort MoCap QoS
#   7. stop recording/bringup and validate the bag

set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  bash scripts/scenarios/run_s1_mocap_pilot_robot.sh <robot_name> [scenario_name]

Typical lab run:
  cd ~/slam_project
  ROS_DOMAIN_ID=0 \
  MOCAP_TOPIC=/optitrack/rigid_bodies/<rigid_body> \
  CMD_TOPIC=/<robot_name>/cmd_vel \
  S1_RADIUS=1.0 \
  S1_DURATION=70 \
  bash scripts/scenarios/run_s1_mocap_pilot_robot.sh <robot_name> s1_circle_1m

Required/important environment:
  MOCAP_TOPIC       MoCap PoseStamped topic. Required; no naming convention is assumed.
  CMD_TOPIC         Namespaced cmd_vel topic. Default: /<robot_name>/cmd_vel
  ROS_DOMAIN_ID     ROS 2 domain. Default: 0

Circle overrides:
  S1_RADIUS         Circle radius in metres. Default: 1.0
  S1_CENTER_X       MoCap-frame center x. Required for shared/fleet circles.
  S1_CENTER_Y       MoCap-frame center y. Required for shared/fleet circles.
                    If omitted for a single robot, one center is inferred and
                    frozen across the precheck and recorded run.
  S1_DURATION       Motion duration in seconds. Default: 70
  S1_LINEAR         Linear speed. Default: 0.10
  S1_MIN_LINEAR     Minimum linear speed while correcting. Default: 0.07
  S1_DIRECTION      ccw or cw. Default: ccw
  S1_POSE_TIMEOUT   Abort if MoCap pose is stale this long. Default: 0.50s
  S1_FORWARD_YAW_OFFSET_DEG  Per-robot rigid-body-to-forward calibration. Default: 0
  S1_BEST_EFFORT_POSE true/false. Default: true for sensor-data QoS.
  S1_START_AT_EPOCH Unix epoch for synchronized fleet motion. Default: 0 (immediate).
  S1_START_SIGNAL_FILE  Wait stopped until this file contains a shared start epoch.
  S1_DRY_RUN       true/false. Default: false. Proves lifecycle without publishing motion.

Recording/gates:
  D455_RESET_MODE  none or usb-reset. Default: none; use reset only for recovery tests.
  S1_PRECHECK       Run a 5s unrecorded motion gate before recording. Default: true
  S1_PRECHECK_DURATION  Precheck motion seconds. Default: 5
  S1_PRECHECK_MAX_RADIUS_ERROR  Allowed precheck radius error. Default: 0.15m
  S1_PRECHECK_MIN_LAPS  Required direction-normalised progress. Default: 0.02
  S1_RECORD         true/false. Default: true
  S1_RECORD_GT      Include the control pose in the robot bag. Default: true
  S1_SPLIT_RECORDING  Record image and auxiliary topics in separate MCAP shards. Default: true
  S1_VALIDATE       true/false. Default: true
  S1_STORAGE_ID     Must be mcap. Default: mcap; collection fails if unavailable.
  S1_MCAP_STORAGE_CONFIG  MCAP writer config. Default: resilient fast-Zstd profile.
  S1_MCAP_STORAGE_PRESET_PROFILE  Optional MCAP preset; overrides the config file.
  ROSBAG2_MAX_CACHE_SIZE  Recorder cache bytes. Default: 536870912 (512 MiB).
  S1_RECORDER_READY_TIMEOUT  Wait for required recorder subscriptions. Default: 60s
  S1_TOPIC_WAIT_SEC Wait per required topic. Default: 180
  MOCAP_MIN_HZ     Minimum accepted GT transport rate. Default: 20
  MOCAP_MAX_HZ     Maximum accepted GT transport rate. Default: 120
  MOCAP_RATE_CHECK_SECONDS  Duration of the pre-motion rate check. Default: 5
  REQUIRE_GT        Require MoCap in validator. Default: true
  REQUIRE_IMU       Require raw D455 gyro+accel in validator. Default: true
  S1_RUNTIME_IMU_GUARD  Watch the existing bringup log for a fatal D455 HID
                    stall, capture evidence, and stop the run. Default: true
                    when REQUIRE_IMU=true. Adds no ROS subscriptions.
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

MOTION_LOCK_FILE="${S1_MOTION_LOCK_FILE:-/tmp/orkar_s1_motion.lock}"
exec 9>"${MOTION_LOCK_FILE}"
if ! flock -n 9; then
    echo "ERROR: another S1 motion/recording session owns ${MOTION_LOCK_FILE}." >&2
    echo "       Stop the existing session before starting a second launcher." >&2
    exit 73
fi
cd "${ROOT}"

# Every ROS participant in this script must use the same local transport.
if [ -r "${ROOT}/scripts/network/load_ros_transport_env.sh" ]; then
    # shellcheck disable=SC1091
    source "${ROOT}/scripts/network/load_ros_transport_env.sh"
fi

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
    elif [ -n "${SUDO_PASSWORD:-}" ]; then
        printf '%s\n' "${SUDO_PASSWORD}" | sudo -S -p '' sh -c "$1"
    else
        sudo sh -c "$1"
    fi
}

sudo_python_usb_reset() {
    local devfile="$1"
    local reset_code
    reset_code='import fcntl
import os
import sys

USBDEVFS_RESET = 21780
fd = os.open(sys.argv[1], os.O_WRONLY)
try:
    fcntl.ioctl(fd, USBDEVFS_RESET, 0)
finally:
    os.close(fd)'
    if sudo -n true 2>/dev/null; then
        sudo python3 -c "${reset_code}" "${devfile}"
    elif [ -n "${SUDO_PASSWORD:-}" ]; then
        printf '%s\n' "${SUDO_PASSWORD}" | \
            sudo -S -p '' python3 -c "${reset_code}" "${devfile}"
    else
        sudo python3 -c "${reset_code}" "${devfile}"
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
    local topic_type="${3:-}"
    local echo_args="'${topic}'"
    if [ -n "${topic_type}" ]; then
        echo_args="${echo_args} '${topic_type}'"
    fi
    echo "Waiting for ${topic}..."
    timeout "${timeout_sec}" bash -lc \
        "source /opt/ros/humble/setup.bash; source '${ROOT}/scripts/network/load_ros_transport_env.sh'; [ -f '${ROOT}/agv2_ws/install/setup.bash' ] && source '${ROOT}/agv2_ws/install/setup.bash'; until ros2 topic echo ${echo_args} --no-daemon --spin-time 2 --once >/dev/null 2>&1; do grep -q '^FAIL_RUNTIME_GUARD ' '${RUNTIME_GUARD_STATUS}' 2>/dev/null && exit 70; sleep 1; done"
}

check_topic_rate_range() {
    local topic="$1"
    local seconds="$2"
    local min_hz="$3"
    local max_hz="$4"
    local log_path="$5"
    local rate

    echo "Checking ${topic} rate for ${seconds}s (${min_hz}-${max_hz} Hz)..."
    timeout "${seconds}" ros2 topic hz "${topic}" --window 200 >"${log_path}" 2>&1 || true
    rate="$(awk '/average rate:/ { value=$3 } END { if (value != "") print value }' "${log_path}")"
    if [ -z "${rate}" ]; then
        echo "ERROR: could not measure the ground-truth rate." >&2
        tail -40 "${log_path}" >&2 || true
        return 1
    fi
    if ! awk -v rate="${rate}" -v min="${min_hz}" -v max="${max_hz}" \
        'BEGIN { exit !(rate >= min && rate <= max) }'; then
        echo "ERROR: ground-truth rate ${rate} Hz is outside ${min_hz}-${max_hz} Hz." >&2
        echo "       Check the source and bridge routes; do not record missing or amplified GT." >&2
        return 1
    fi
    echo "Ground-truth rate gate passed: ${rate} Hz."
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

wait_for_recorder_subscriptions() {
    local timeout_sec="$1"
    local recorder_pid="$2"
    local recorder_log="$3"
    local recorder_label="$4"
    shift 4
    local deadline=$((SECONDS + timeout_sec))
    local pending=()
    local topic

    while [ "${SECONDS}" -lt "${deadline}" ]; do
        if runtime_guard_failed; then
            echo "ERROR: runtime IMU guard failed while waiting for ${recorder_label} recorder." >&2
            cat "${RUNTIME_GUARD_STATUS}" >&2 || true
            return 70
        fi
        if ! kill -0 "${recorder_pid}" 2>/dev/null; then
            echo "ERROR: ${recorder_label} recorder exited before becoming ready." >&2
            tail -120 "${recorder_log}" >&2 || true
            return 1
        fi

        pending=()
        for topic in "$@"; do
            if ! grep -Fq "Subscribed to topic '${topic}'" "${recorder_log}" 2>/dev/null; then
                pending+=("${topic}")
            fi
        done
        if [ "${#pending[@]}" -eq 0 ]; then
            echo "${recorder_label} recorder subscribed to every required stream."
            return 0
        fi
        sleep 1
    done

    echo "ERROR: ${recorder_label} recorder subscription gate timed out after ${timeout_sec}s." >&2
    printf '  missing subscription: %s\n' "${pending[@]}" >&2
    tail -120 "${recorder_log}" >&2 || true
    return 1
}

check_recorder_subscription_skew() {
    local recorder_log="$1"
    local recorder_label="$2"
    local max_skew_sec="$3"
    shift 3
    python3 - "${recorder_log}" "${recorder_label}" "${max_skew_sec}" "$@" <<'PY'
import re
import sys

log_path = sys.argv[1]
label = sys.argv[2]
max_skew = float(sys.argv[3])
required = set(sys.argv[4:])
timestamps = {}
pattern = re.compile(
    r"\[[A-Z]+\]\s+\[([0-9]+(?:\.[0-9]+)?)\].*Subscribed to topic '([^']+)'"
)

with open(log_path, "r", encoding="utf-8", errors="replace") as stream:
    for line in stream:
        match = pattern.search(line)
        if not match:
            continue
        topic = match.group(2)
        if topic in required and topic not in timestamps:
            timestamps[topic] = float(match.group(1))

missing = sorted(required - timestamps.keys())
if missing:
    print("ERROR: recorder log is missing required subscription timestamps:", file=sys.stderr)
    for topic in missing:
        print(f"  {topic}", file=sys.stderr)
    raise SystemExit(1)

skew = max(timestamps.values()) - min(timestamps.values())
print(f"{label} recorder required-topic subscription skew: {skew:.3f}s")
if skew > max_skew:
    print(
        f"ERROR: subscription skew {skew:.3f}s exceeds {max_skew:.3f}s; "
        "aborting before motion.",
        file=sys.stderr,
    )
    raise SystemExit(1)
PY
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
S1_POSE_TIMEOUT="${S1_POSE_TIMEOUT:-0.50}"
S1_HEADING_KP="${S1_HEADING_KP:-1.10}"
S1_RADIUS_KP="${S1_RADIUS_KP:-1.50}"
S1_MAX_ANGULAR="${S1_MAX_ANGULAR:-0.55}"
S1_MAX_RADIUS_HEADING_OFFSET_DEG="${S1_MAX_RADIUS_HEADING_OFFSET_DEG:-35}"
S1_BEST_EFFORT_POSE="${S1_BEST_EFFORT_POSE:-true}"
D455_RESET_MODE="${D455_RESET_MODE:-none}"
S1_START_AT_EPOCH="${S1_START_AT_EPOCH:-0}"
S1_START_SIGNAL_FILE="${S1_START_SIGNAL_FILE:-}"
S1_DRY_RUN="${S1_DRY_RUN:-false}"
S1_PRECHECK="${S1_PRECHECK:-true}"
S1_PRECHECK_DURATION="${S1_PRECHECK_DURATION:-5}"
S1_PRECHECK_MAX_RADIUS_ERROR="${S1_PRECHECK_MAX_RADIUS_ERROR:-0.15}"
S1_PRECHECK_MIN_LAPS="${S1_PRECHECK_MIN_LAPS:-0.02}"
S1_PRECHECK_MAX_POSE_AGE="${S1_PRECHECK_MAX_POSE_AGE:-0.20}"
S1_PRECHECK_MIN_POSE_SAMPLES="${S1_PRECHECK_MIN_POSE_SAMPLES:-30}"
S1_RECORD="${S1_RECORD:-true}"
S1_RECORD_GT="${S1_RECORD_GT:-true}"
S1_SPLIT_RECORDING="${S1_SPLIT_RECORDING:-true}"
S1_VALIDATE="${S1_VALIDATE:-true}"
S1_STORAGE_ID="${S1_STORAGE_ID:-mcap}"
S1_MCAP_STORAGE_CONFIG="${S1_MCAP_STORAGE_CONFIG:-${ROOT}/configs/mcap_resilient_high_throughput.yaml}"
S1_MCAP_STORAGE_PRESET_PROFILE="${S1_MCAP_STORAGE_PRESET_PROFILE:-}"
ROSBAG2_MAX_CACHE_SIZE="${ROSBAG2_MAX_CACHE_SIZE:-536870912}"
S1_SENSOR_CACHE_SIZE="${S1_SENSOR_CACHE_SIZE:-402653184}"
S1_AUX_CACHE_SIZE="${S1_AUX_CACHE_SIZE:-67108864}"
S1_RECORDER_READY_TIMEOUT="${S1_RECORDER_READY_TIMEOUT:-60}"
S1_MAX_RECORDER_SUBSCRIPTION_SKEW_SEC="${S1_MAX_RECORDER_SUBSCRIPTION_SKEW_SEC:-2.0}"
S1_RECORDER_PREROLL_SEC="${S1_RECORDER_PREROLL_SEC:-2}"
S1_TOPIC_WAIT_SEC="${S1_TOPIC_WAIT_SEC:-180}"
MOCAP_MIN_HZ="${MOCAP_MIN_HZ:-20}"
MOCAP_MAX_HZ="${MOCAP_MAX_HZ:-120}"
MOCAP_RATE_CHECK_SECONDS="${MOCAP_RATE_CHECK_SECONDS:-5}"
S1_GT_HOLD_READY_TIMEOUT="${S1_GT_HOLD_READY_TIMEOUT:-15}"
S1_RECORDER_STOP_TIMEOUT="${S1_RECORDER_STOP_TIMEOUT:-120}"
S1_BRINGUP_STOP_TIMEOUT="${S1_BRINGUP_STOP_TIMEOUT:-30}"
S1_POST_ROLL_SEC="${S1_POST_ROLL_SEC:-5}"
S1_MIN_BAG_DURATION="${S1_MIN_BAG_DURATION:-30}"
REQUIRE_GT="${REQUIRE_GT:-true}"
REQUIRE_IMU="${REQUIRE_IMU:-true}"
REQUIRE_RESILIENT_STORAGE="${REQUIRE_RESILIENT_STORAGE:-true}"
S1_RUNTIME_IMU_GUARD="${S1_RUNTIME_IMU_GUARD:-${REQUIRE_IMU}}"

require_nonempty "MOCAP_TOPIC" "${MOCAP_TOPIC}"

if [ -n "${S1_START_SIGNAL_FILE}" ]; then
    rm -f -- "${S1_START_SIGNAL_FILE}"
fi
if ! awk -v min="${MOCAP_MIN_HZ}" -v max="${MOCAP_MAX_HZ}" -v seconds="${MOCAP_RATE_CHECK_SECONDS}" \
    'BEGIN { exit !(min > 0 && max > min && seconds > 0) }'; then
    echo "ERROR: require 0 < MOCAP_MIN_HZ < MOCAP_MAX_HZ and MOCAP_RATE_CHECK_SECONDS > 0." >&2
    exit 2
fi
if [ "${CMD_TOPIC}" = "/cmd_vel" ]; then
    echo "ERROR: refusing to run S1 pilot on root /cmd_vel. Use /${ROBOT_NAME}/cmd_vel." >&2
    exit 2
fi
if awk -v epoch="${S1_START_AT_EPOCH}" 'BEGIN { exit !(epoch > 0) }' && \
   { [ -z "${S1_CENTER_X:-}" ] || [ -z "${S1_CENTER_Y:-}" ]; }; then
    echo "ERROR: synchronized S1 runs require one surveyed center via S1_CENTER_X and S1_CENTER_Y." >&2
    echo "       Per-robot center inference does not produce concentric fleet paths." >&2
    exit 2
fi
if bool_true "${S1_RECORD}" && bool_true "${S1_VALIDATE}" && \
   ! bool_true "${S1_RECORD_GT}" && bool_true "${REQUIRE_GT}"; then
    echo "ERROR: REQUIRE_GT=true conflicts with S1_RECORD_GT=false." >&2
    echo "       For centrally recorded GT, set both S1_RECORD_GT=false and REQUIRE_GT=false," >&2
    echo "       then audit timestamp overlap with the central GT bag after collection." >&2
    exit 2
fi

source_ros2
if [ "${ORKAR_ROS_TRANSPORT:-}" = "zenoh-bridge-ros2dds" ]; then
    if [ "${ROS_LOCALHOST_ONLY:-}" != "1" ]; then
        echo "ERROR: Zenoh robot sessions require ROS_LOCALHOST_ONLY=1." >&2
        exit 1
    fi
    if systemctl is-active --quiet orkar-natnet-pose-source.service || \
       pgrep -af '[n]atnet_ros2_pose_publisher.py' >/dev/null 2>&1; then
        echo "ERROR: direct NatNet source is running on this Zenoh-importing robot." >&2
        echo "       Disable it: sudo systemctl disable --now orkar-natnet-pose-source.service" >&2
        exit 1
    fi
    if ! systemctl is-active --quiet orkar-zenoh-gt.service; then
        echo "ERROR: orkar-zenoh-gt.service is not active." >&2
        echo "       Run: bash scripts/network/configure_zenoh.sh status" >&2
        exit 1
    fi
fi
mkdir -p "${HOME}/agv_data"

STAMP="$(date +%Y%m%d_%H%M%S)"
SESSION_ID="${ROBOT_NAME}_${SCENARIO}_${STAMP}"
BAG="${HOME}/agv_data/${SESSION_ID}"
SENSOR_BAG="${BAG}_sensors"
AUX_BAG="${BAG}_aux"
BRINGUP_LOG="${HOME}/agv_data/${SESSION_ID}_bringup.log"
RECORD_LOG="${HOME}/agv_data/${SESSION_ID}_record.log"
SENSOR_RECORD_LOG="${HOME}/agv_data/${SESSION_ID}_sensors_record.log"
AUX_RECORD_LOG="${HOME}/agv_data/${SESSION_ID}_aux_record.log"
GT_HOLD_LOG="${HOME}/agv_data/${SESSION_ID}_gt_hold.log"
GT_RATE_LOG="${HOME}/agv_data/${SESSION_ID}_gt_rate.log"
SUMMARY_JSON="${HOME}/agv_data/${SESSION_ID}_circle_summary.json"
PRECHECK_JSON="${HOME}/agv_data/${SESSION_ID}_circle_precheck.json"
VALIDATION_JSON="${HOME}/agv_data/${SESSION_ID}_validate.json"
RUNTIME_GUARD_LOG="${HOME}/agv_data/${SESSION_ID}_runtime_imu_guard.log"
RUNTIME_GUARD_STATUS="${HOME}/agv_data/${SESSION_ID}_runtime_imu_guard.status"
RUNTIME_GUARD_EVIDENCE_DIR="${HOME}/agv_data/${SESSION_ID}_runtime_imu_guard_evidence"
RUNTIME_FAULT_CLASSIFICATION="${HOME}/agv_data/${SESSION_ID}_realsense_fault_classification.txt"
if bool_true "${S1_SPLIT_RECORDING}"; then
    BAG_PATHS=("${SENSOR_BAG}" "${AUX_BAG}")
else
    BAG_PATHS=("${BAG}")
fi

BRINGUP_PID=""
REC_PID=""
SENSOR_REC_PID=""
AUX_REC_PID=""
GT_HOLD_PID=""
RUNTIME_GUARD_PID=""
DRIVE_PID=""

stop_gt_hold() {
    if [ -n "${GT_HOLD_PID}" ] && kill -0 "${GT_HOLD_PID}" 2>/dev/null; then
        stop_process_group "${GT_HOLD_PID}" TERM
        wait_for_exit_or_kill "${GT_HOLD_PID}" "ground-truth discovery hold" 5
    fi
    GT_HOLD_PID=""
}

runtime_guard_failed() {
    [ -s "${RUNTIME_GUARD_STATUS}" ] && \
        grep -q '^FAIL_RUNTIME_GUARD ' "${RUNTIME_GUARD_STATUS}"
}

start_runtime_guard() {
    if ! bool_true "${S1_RUNTIME_IMU_GUARD}" || ! bool_true "${REQUIRE_IMU}"; then
        echo "DISABLED" > "${RUNTIME_GUARD_STATUS}"
        return 0
    fi
    if [ ! -r "${ROOT}/scripts/diagnostics/watch_realsense_runtime.py" ]; then
        echo "ERROR: runtime IMU guard is missing." >&2
        return 1
    fi

    echo "Starting no-subscription D455 runtime IMU guard..."
    setsid python3 "${ROOT}/scripts/diagnostics/watch_realsense_runtime.py" \
        --bringup-log "${BRINGUP_LOG}" \
        --status-file "${RUNTIME_GUARD_STATUS}" \
        --evidence-dir "${RUNTIME_GUARD_EVIDENCE_DIR}" \
        --require-imu \
        >"${RUNTIME_GUARD_LOG}" 2>&1 < /dev/null &
    RUNTIME_GUARD_PID=$!
    sleep 0.3
    if ! kill -0 "${RUNTIME_GUARD_PID}" 2>/dev/null; then
        echo "ERROR: runtime IMU guard exited during startup." >&2
        cat "${RUNTIME_GUARD_LOG}" >&2 || true
        return 1
    fi
    echo "runtime_imu_guard_pid: ${RUNTIME_GUARD_PID}"
}

stop_runtime_guard() {
    [ -n "${RUNTIME_GUARD_PID}" ] || return 0
    if kill -0 "${RUNTIME_GUARD_PID}" 2>/dev/null; then
        if runtime_guard_failed; then
            # Let the guard finish its bounded evidence snapshot before cleanup.
            wait_for_exit_or_kill "${RUNTIME_GUARD_PID}" "runtime IMU guard evidence capture" 15
        else
            stop_process_group "${RUNTIME_GUARD_PID}" TERM
            wait_for_exit_or_kill "${RUNTIME_GUARD_PID}" "runtime IMU guard" 5
        fi
    fi
    wait "${RUNTIME_GUARD_PID}" 2>/dev/null || true
    RUNTIME_GUARD_PID=""
}

classify_runtime_fault() {
    runtime_guard_failed || return 0
    [ ! -s "${RUNTIME_FAULT_CLASSIFICATION}" ] || return 0

    python3 "${ROOT}/scripts/diagnostics/classify_realsense_fault.py" \
        --label "${SESSION_ID}" \
        --readiness-log "${RUNTIME_GUARD_STATUS}" \
        --bringup-log "${BRINGUP_LOG}" \
        --kernel-log "${RUNTIME_GUARD_EVIDENCE_DIR}/kernel_runtime.log" \
        --hardware-log "${RUNTIME_GUARD_EVIDENCE_DIR}/power_thermal.log" \
        --hardware-log "${RUNTIME_GUARD_EVIDENCE_DIR}/usb_topology.log" \
        > "${RUNTIME_FAULT_CLASSIFICATION}" 2>&1 || true
    echo "--- RealSense runtime fault classification ---"
    sed -n '1,100p' "${RUNTIME_FAULT_CLASSIFICATION}" || true
}

run_circle_with_runtime_guard() {
    local label="$1"
    shift
    local rc=0

    if runtime_guard_failed; then
        echo "ERROR: runtime IMU guard failed before ${label}." >&2
        cat "${RUNTIME_GUARD_STATUS}" >&2 || true
        return 70
    fi

    setsid python3 scripts/logging/drive_mocap_circle_ros2.py "$@" &
    DRIVE_PID=$!
    while kill -0 "${DRIVE_PID}" 2>/dev/null; do
        if runtime_guard_failed; then
            echo "ERROR: D455 IMU source stalled during ${label}; stopping immediately." >&2
            cat "${RUNTIME_GUARD_STATUS}" >&2 || true
            stop_process_group "${DRIVE_PID}" INT
            wait_for_exit_or_kill "${DRIVE_PID}" "${label}" 5
            wait "${DRIVE_PID}" 2>/dev/null || true
            DRIVE_PID=""
            publish_zero
            return 70
        fi
        sleep 0.2
    done
    wait "${DRIVE_PID}" || rc=$?
    DRIVE_PID=""

    if runtime_guard_failed; then
        echo "ERROR: D455 IMU source stalled as ${label} ended." >&2
        cat "${RUNTIME_GUARD_STATUS}" >&2 || true
        publish_zero
        return 70
    fi
    return "${rc}"
}

cleanup() {
    set +e
    if [ -n "${DRIVE_PID}" ] && kill -0 "${DRIVE_PID}" 2>/dev/null; then
        stop_process_group "${DRIVE_PID}" INT
        wait_for_exit_or_kill "${DRIVE_PID}" "circle controller" 5
    fi
    DRIVE_PID=""
    publish_zero
    if [ -n "${REC_PID}" ] && kill -0 "${REC_PID}" 2>/dev/null; then
        echo "Stopping ros2 bag record..."
        stop_process_group "${REC_PID}" INT
        wait_for_exit_or_kill "${REC_PID}" "ros2 bag record" "${S1_RECORDER_STOP_TIMEOUT}"
    fi
    if [ -n "${SENSOR_REC_PID}" ] && kill -0 "${SENSOR_REC_PID}" 2>/dev/null; then
        echo "Stopping sensor bag recorder..."
        stop_process_group "${SENSOR_REC_PID}" INT
        wait_for_exit_or_kill "${SENSOR_REC_PID}" "sensor bag recorder" "${S1_RECORDER_STOP_TIMEOUT}"
    fi
    if [ -n "${AUX_REC_PID}" ] && kill -0 "${AUX_REC_PID}" 2>/dev/null; then
        echo "Stopping auxiliary bag recorder..."
        stop_process_group "${AUX_REC_PID}" INT
        wait_for_exit_or_kill "${AUX_REC_PID}" "auxiliary bag recorder" "${S1_RECORDER_STOP_TIMEOUT}"
    fi
    stop_gt_hold
    stop_runtime_guard
    classify_runtime_fault
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
echo "mocap_rate:   ${MOCAP_MIN_HZ}-${MOCAP_MAX_HZ} Hz"
echo "cmd_topic:    ${CMD_TOPIC}"
echo "bag_base:     ${BAG}"
echo "radius:       ${S1_RADIUS} m"
echo "duration:     ${S1_DURATION} s"
echo "linear:       ${S1_LINEAR} m/s"
echo "pose_qos:     $(bool_true "${S1_BEST_EFFORT_POSE}" && echo best_effort || echo reliable)"
echo "d455_reset:   ${D455_RESET_MODE}"
echo "start_epoch:  ${S1_START_AT_EPOCH}"
echo "start_signal: ${S1_START_SIGNAL_FILE:-none}"
echo "transport:    ${ORKAR_ROS_TRANSPORT:-local DDS}"
echo "localhost:    ${ROS_LOCALHOST_ONLY:-unset}"
echo "precheck:     ${S1_PRECHECK} (${S1_PRECHECK_DURATION}s)"
echo "dry_run:      ${S1_DRY_RUN}"
echo "record_gt:    ${S1_RECORD_GT}"
echo "split_record: ${S1_SPLIT_RECORDING}"
echo "storage:      ${S1_STORAGE_ID}"
echo "mcap_config:  ${S1_MCAP_STORAGE_CONFIG}"
echo "mcap_preset:  ${S1_MCAP_STORAGE_PRESET_PROFILE:-none}"
echo "record_cache: ${ROSBAG2_MAX_CACHE_SIZE} bytes"
echo "runtime_guard: ${S1_RUNTIME_IMU_GUARD}"
echo "========================================================================"

kill_local_robot_graph
case "${D455_RESET_MODE}" in
    none|false)
        echo "Skipping D455 reset before bringup (D455_RESET_MODE=${D455_RESET_MODE})."
        ;;
    usb-reset)
        reset_d455_once
        ;;
    *)
        echo "ERROR: unsupported D455_RESET_MODE='${D455_RESET_MODE}' (use none or usb-reset)." >&2
        exit 2
        ;;
esac

echo "Starting one bringup..."
setsid bash -lc \
    "source /opt/ros/humble/setup.bash; source '${ROOT}/scripts/network/load_ros_transport_env.sh'; [ -f '${ROOT}/agv2_ws/install/setup.bash' ] && source '${ROOT}/agv2_ws/install/setup.bash'; exec ros2 launch agv_bringup bringup.launch.py agv_serial_port:=/dev/ttyACM0 agv_color_profile:=640x480x15 agv_depth_profile:=640x480x15 enable_sync:=false initial_reset:=false agv_cmd_vel_topic:='${CMD_TOPIC}'" \
    >"${BRINGUP_LOG}" 2>&1 < /dev/null &
BRINGUP_PID=$!
echo "bringup_pid: ${BRINGUP_PID}"
start_runtime_guard

# A Zenoh-imported topic disappears from the local DDS graph when its last
# local subscriber exits. Establish one subscriber before any one-shot probes
# and keep it alive through recorder startup so the imported route never gaps.
echo "Holding ground-truth discovery route..."
setsid env PYTHONUNBUFFERED=1 stdbuf -oL -eL \
    ros2 topic echo "${MOCAP_TOPIC}" geometry_msgs/msg/PoseStamped \
    --qos-reliability best_effort >"${GT_HOLD_LOG}" 2>&1 &
GT_HOLD_PID=$!
if ! kill -0 "${GT_HOLD_PID}" 2>/dev/null; then
    echo "ERROR: ground-truth discovery hold exited before recording." >&2
    tail -40 "${GT_HOLD_LOG}" >&2 || true
    exit 1
fi
if ! timeout "${S1_GT_HOLD_READY_TIMEOUT}" bash -c \
    'until grep -q "^header:" "$1" 2>/dev/null; do sleep 0.2; done' \
    _ "${GT_HOLD_LOG}"; then
    echo "ERROR: ground-truth discovery hold received no pose within ${S1_GT_HOLD_READY_TIMEOUT}s." >&2
    tail -40 "${GT_HOLD_LOG}" >&2 || true
    exit 1
fi
echo "Ground-truth discovery route is receiving poses."

required_topics=(
    "/scan"
    "/odom"
    "/camera/color/image_raw"
    "/camera/depth/image_rect_raw"
    "${MOCAP_TOPIC}"
)
if bool_true "${REQUIRE_IMU}"; then
    required_topics+=("/camera/gyro/sample" "/camera/accel/sample")
fi

for topic in "${required_topics[@]}"; do
    topic_type=""
    if [ "${topic}" = "${MOCAP_TOPIC}" ]; then
        topic_type="geometry_msgs/msg/PoseStamped"
    fi
    if ! wait_for_topic_once "${topic}" "${S1_TOPIC_WAIT_SEC}" "${topic_type}"; then
        echo "ERROR: required topic did not publish: ${topic}" >&2
        echo "--- bringup log tail ---" >&2
        tail -120 "${BRINGUP_LOG}" >&2 || true
        exit 1
    fi
done
echo "Required live topic gate passed."

check_topic_rate_range \
    "${MOCAP_TOPIC}" \
    "${MOCAP_RATE_CHECK_SECONDS}" \
    "${MOCAP_MIN_HZ}" \
    "${MOCAP_MAX_HZ}" \
    "${GT_RATE_LOG}"

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

if bool_true "${S1_PRECHECK}" && ! bool_true "${S1_DRY_RUN}"; then
    echo "Running required ${S1_PRECHECK_DURATION}s unrecorded circle precheck..."
    PRECHECK_ARGS=("${CIRCLE_ARGS[@]}")
    PRECHECK_ARGS+=(
        --duration "${S1_PRECHECK_DURATION}"
        --max-radius-error "${S1_PRECHECK_MAX_RADIUS_ERROR}"
        --summary-json "${PRECHECK_JSON}"
    )
    run_circle_with_runtime_guard "circle precheck" "${PRECHECK_ARGS[@]}"
    python3 scripts/scenarios/validate_s1_circle_summary.py \
        "${PRECHECK_JSON}" \
        --max-radius-error "${S1_PRECHECK_MAX_RADIUS_ERROR}" \
        --min-laps "${S1_PRECHECK_MIN_LAPS}" \
        --max-pose-age "${S1_PRECHECK_MAX_POSE_AGE}" \
        --min-pose-samples "${S1_PRECHECK_MIN_POSE_SAMPLES}"
    publish_zero
    sleep 2
    echo "Precheck passed; recording may start."
fi

FULL_CIRCLE_ARGS=("${CIRCLE_ARGS[@]}")
if bool_true "${S1_PRECHECK}" && ! bool_true "${S1_DRY_RUN}" && \
   [ -z "${S1_CENTER_X:-}" ] && [ -z "${S1_CENTER_Y:-}" ]; then
    FROZEN_CENTER_VALUES="$(python3 - "${PRECHECK_JSON}" <<'PY'
import json
import math
import sys

with open(sys.argv[1], encoding="utf-8") as handle:
    summary = json.load(handle)
center_x = float(summary["center_x_m"])
center_y = float(summary["center_y_m"])
if not math.isfinite(center_x) or not math.isfinite(center_y):
    raise SystemExit("precheck center is not finite")
print(center_x, center_y)
PY
    )"
    FROZEN_CENTER_X="${FROZEN_CENTER_VALUES%% *}"
    FROZEN_CENTER_Y="${FROZEN_CENTER_VALUES#* }"
    FULL_CIRCLE_ARGS+=(--center-x "${FROZEN_CENTER_X}" --center-y "${FROZEN_CENTER_Y}")
    echo "Frozen inferred center for full run: (${FROZEN_CENTER_X}, ${FROZEN_CENTER_Y})"
fi
if ! awk -v epoch="${S1_START_AT_EPOCH}" 'BEGIN { exit !(epoch >= 0) }'; then
    echo "ERROR: S1_START_AT_EPOCH must be a non-negative Unix epoch." >&2
    exit 2
fi
if awk -v epoch="${S1_START_AT_EPOCH}" 'BEGIN { exit !(epoch > 0) }'; then
    FULL_CIRCLE_ARGS+=(--start-at-epoch "${S1_START_AT_EPOCH}")
fi
if [ -n "${S1_START_SIGNAL_FILE}" ]; then
    echo "FLEET_READY: sensors passed; waiting stopped before recorder startup on ${S1_START_SIGNAL_FILE}"
    while [ ! -s "${S1_START_SIGNAL_FILE}" ]; do
        if grep -q '^FAIL_RUNTIME_GUARD ' "${RUNTIME_GUARD_STATUS}" 2>/dev/null; then
            echo "ERROR: D455 IMU failed while waiting for fleet release." >&2
            cat "${RUNTIME_GUARD_STATUS}" >&2 || true
            exit 70
        fi
        sleep 0.2
    done
    RELEASE_EPOCH="$(tr -d '[:space:]' < "${S1_START_SIGNAL_FILE}")"
    if ! awk -v epoch="${RELEASE_EPOCH}" 'BEGIN { exit !(epoch > 0) }'; then
        echo "ERROR: fleet release file must contain a positive Unix epoch." >&2
        exit 2
    fi
    S1_START_AT_EPOCH="${RELEASE_EPOCH}"
    FULL_CIRCLE_ARGS+=(--start-at-epoch "${S1_START_AT_EPOCH}")
    echo "Fleet release received; recorder startup begins for epoch ${S1_START_AT_EPOCH}."
fi

if bool_true "${S1_RECORD}"; then
    if [ "${S1_STORAGE_ID}" != "mcap" ]; then
        echo "ERROR: S1 dataset collection is MCAP-only; got S1_STORAGE_ID=${S1_STORAGE_ID}." >&2
        exit 2
    fi
    if ! ros2 bag record --help 2>/dev/null | grep -Eq -- "-s .*mcap|--storage .*mcap|\\bmcap\\b"; then
        echo "ERROR: rosbag2 MCAP storage is unavailable; refusing dataset collection." >&2
        echo "       Install ros-${ROS_DISTRO:-humble}-rosbag2-storage-mcap and rerun setup." >&2
        exit 1
    fi
    STORAGE_ID="mcap"

    ROS2_STORAGE_ARGS=(-s "${STORAGE_ID}")
    if [ -n "${S1_MCAP_STORAGE_PRESET_PROFILE}" ]; then
        ROS2_STORAGE_ARGS+=(--storage-preset-profile "${S1_MCAP_STORAGE_PRESET_PROFILE}")
    else
        if [ ! -r "${S1_MCAP_STORAGE_CONFIG}" ]; then
            echo "ERROR: MCAP storage config is not readable: ${S1_MCAP_STORAGE_CONFIG}" >&2
            exit 1
        fi
        ROS2_STORAGE_ARGS+=(--storage-config-file "${S1_MCAP_STORAGE_CONFIG}")
    fi
    if [ -f "${ROOT}/configs/rosbag2_sensor_qos.yaml" ]; then
        ROS2_STORAGE_ARGS+=(--qos-profile-overrides-path "${ROOT}/configs/rosbag2_sensor_qos.yaml")
    fi

    SENSOR_RECORD_TOPICS=(
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
    )
    SENSOR_REQUIRED_TOPICS=(
        "/scan"
        "/odom"
        "/tf"
        "/tf_static"
        "/camera/color/image_raw"
        "/camera/color/camera_info"
        "/camera/depth/image_rect_raw"
        "/camera/depth/camera_info"
    )
    AUX_RECORD_TOPICS=(
        "/camera/imu"
        "/camera/gyro/sample"
        "/camera/accel/sample"
        "/imu"
        "/diagnostics"
        "/tag_detections"
        "/aruco/target_pose"
    )
    AUX_REQUIRED_TOPICS=()
    if bool_true "${REQUIRE_IMU}"; then
        AUX_REQUIRED_TOPICS+=("/camera/gyro/sample" "/camera/accel/sample")
    fi
    if bool_true "${S1_RECORD_GT}"; then
        AUX_RECORD_TOPICS+=("${MOCAP_TOPIC}" "/mocap")
        AUX_REQUIRED_TOPICS+=("${MOCAP_TOPIC}")
    fi

    if bool_true "${S1_SPLIT_RECORDING}"; then
        echo "Starting topic-partitioned ros2 bag recorders (${STORAGE_ID})..."
        setsid ros2 bag record \
            --max-cache-size "${S1_SENSOR_CACHE_SIZE}" \
            "${ROS2_STORAGE_ARGS[@]}" \
            -o "${SENSOR_BAG}" \
            "${SENSOR_RECORD_TOPICS[@]}" \
            >"${SENSOR_RECORD_LOG}" 2>&1 < /dev/null &
        SENSOR_REC_PID=$!
        setsid ros2 bag record \
            --max-cache-size "${S1_AUX_CACHE_SIZE}" \
            "${ROS2_STORAGE_ARGS[@]}" \
            -o "${AUX_BAG}" \
            "${AUX_RECORD_TOPICS[@]}" \
            >"${AUX_RECORD_LOG}" 2>&1 < /dev/null &
        AUX_REC_PID=$!
        echo "sensor_record_pid: ${SENSOR_REC_PID}"
        echo "aux_record_pid:    ${AUX_REC_PID}"
        wait_for_recorder_subscriptions \
            "${S1_RECORDER_READY_TIMEOUT}" \
            "${SENSOR_REC_PID}" \
            "${SENSOR_RECORD_LOG}" \
            "sensor" \
            "${SENSOR_REQUIRED_TOPICS[@]}"
        if [ "${#AUX_REQUIRED_TOPICS[@]}" -gt 0 ]; then
            wait_for_recorder_subscriptions \
                "${S1_RECORDER_READY_TIMEOUT}" \
                "${AUX_REC_PID}" \
                "${AUX_RECORD_LOG}" \
                "auxiliary" \
                "${AUX_REQUIRED_TOPICS[@]}"
        fi
        check_recorder_subscription_skew \
            "${SENSOR_RECORD_LOG}" \
            "sensor" \
            "${S1_MAX_RECORDER_SUBSCRIPTION_SKEW_SEC}" \
            "${SENSOR_REQUIRED_TOPICS[@]}"
        if [ "${#AUX_REQUIRED_TOPICS[@]}" -gt 1 ]; then
            check_recorder_subscription_skew \
                "${AUX_RECORD_LOG}" \
                "auxiliary" \
                "${S1_MAX_RECORDER_SUBSCRIPTION_SKEW_SEC}" \
                "${AUX_REQUIRED_TOPICS[@]}"
        fi
    else
        RECORD_TOPICS=("${SENSOR_RECORD_TOPICS[@]}" "${AUX_RECORD_TOPICS[@]}")
        REQUIRED_RECORD_TOPICS=("${SENSOR_REQUIRED_TOPICS[@]}" "${AUX_REQUIRED_TOPICS[@]}")
        echo "Starting single ros2 bag recorder (${STORAGE_ID})..."
        setsid ros2 bag record \
            --max-cache-size "${ROSBAG2_MAX_CACHE_SIZE}" \
            "${ROS2_STORAGE_ARGS[@]}" \
            -o "${BAG}" \
            "${RECORD_TOPICS[@]}" \
            >"${RECORD_LOG}" 2>&1 < /dev/null &
        REC_PID=$!
        echo "record_pid: ${REC_PID}"
        wait_for_recorder_subscriptions \
            "${S1_RECORDER_READY_TIMEOUT}" \
            "${REC_PID}" \
            "${RECORD_LOG}" \
            "single" \
            "${REQUIRED_RECORD_TOPICS[@]}"
        check_recorder_subscription_skew \
            "${RECORD_LOG}" \
            "single" \
            "${S1_MAX_RECORDER_SUBSCRIPTION_SKEW_SEC}" \
            "${REQUIRED_RECORD_TOPICS[@]}"
    fi
    sleep "${S1_RECORDER_PREROLL_SEC}"
    echo "Required recorder subscriptions are active; scenario motion may start."
    if bool_true "${S1_RECORD_GT}"; then
        # The recorder is now the persistent DDS subscriber that keeps the
        # on-demand Zenoh route alive. The discovery hold would otherwise
        # deserialize and format every GT pose for the full run.
        echo "Ground-truth recorder owns the Zenoh route; stopping discovery hold."
        stop_gt_hold
    fi
fi

set +e
run_circle_with_runtime_guard "recorded circle" "${FULL_CIRCLE_ARGS[@]}"
DRIVE_RC=$?
set -e

if runtime_guard_failed; then
    echo "Skipping post-roll because the D455 IMU source failed."
else
    sleep "${S1_POST_ROLL_SEC}"
fi
publish_zero

if bool_true "${S1_RECORD}"; then
    if bool_true "${S1_SPLIT_RECORDING}"; then
        echo "Stopping topic-partitioned ros2 bag recorders..."
        stop_process_group "${SENSOR_REC_PID}" INT
        stop_process_group "${AUX_REC_PID}" INT
        wait_for_exit_or_kill "${SENSOR_REC_PID}" "sensor bag recorder" "${S1_RECORDER_STOP_TIMEOUT}"
        wait_for_exit_or_kill "${AUX_REC_PID}" "auxiliary bag recorder" "${S1_RECORDER_STOP_TIMEOUT}"
        SENSOR_REC_PID=""
        AUX_REC_PID=""
    else
        echo "Stopping ros2 bag record..."
        stop_process_group "${REC_PID}" INT
        wait_for_exit_or_kill "${REC_PID}" "ros2 bag record" "${S1_RECORDER_STOP_TIMEOUT}"
        REC_PID=""
    fi
    stop_gt_hold

    if bool_true "${S1_SPLIT_RECORDING}"; then
        echo "--- sensor record log tail ---"
        tail -60 "${SENSOR_RECORD_LOG}" || true
        echo "--- auxiliary record log tail ---"
        tail -60 "${AUX_RECORD_LOG}" || true
    else
        echo "--- record log tail ---"
        tail -80 "${RECORD_LOG}" || true
    fi
    for bag_path in "${BAG_PATHS[@]}"; do
        echo "--- bag info: ${bag_path} ---"
        ros2 bag info "${bag_path}" | sed -n '1,180p' || true
    done
fi

stop_gt_hold
stop_runtime_guard
classify_runtime_fault

echo "Stopping bringup..."
stop_process_group "${BRINGUP_PID}" INT
wait_for_exit_or_kill "${BRINGUP_PID}" "bringup" "${S1_BRINGUP_STOP_TIMEOUT}"
BRINGUP_PID=""

if bool_true "${S1_RECORD}" && bool_true "${S1_VALIDATE}"; then
    echo "--- bag validation ---"
    VALIDATE_ARGS=(--min-duration "${S1_MIN_BAG_DURATION}" --json-out "${VALIDATION_JSON}")
    if awk -v epoch="${S1_START_AT_EPOCH}" 'BEGIN { exit !(epoch > 0) }'; then
        # Recorder pre-roll is useful evidence but is not part of the scenario.
        # Keep strict continuity checks scoped to the synchronized motion window.
        VALIDATE_ARGS+=(
            --window-start-epoch "${S1_START_AT_EPOCH}"
            --window-duration "${S1_DURATION}"
        )
    fi
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
    COLOR_BAG_MIN_HZ="${COLOR_BAG_MIN_HZ:-12}" \
    DEPTH_BAG_MIN_HZ="${DEPTH_BAG_MIN_HZ:-12}" \
    DEPTH_TOPIC="/camera/depth/image_rect_raw" \
    DEPTH_INFO_TOPIC="/camera/depth/camera_info" \
    IMU_TOPICS="/camera/gyro/sample /camera/accel/sample /camera/imu /imu" \
    python3 scripts/logging/validate_ros2_bag.py "${BAG_PATHS[@]}" "${VALIDATE_ARGS[@]}"
    VALIDATE_RC=$?
    set -e
fi

trap - EXIT INT TERM

echo "========================================================================"
echo "S1 COMPLETE"
echo "drive_rc:      ${DRIVE_RC}"
echo "validate_rc:   ${VALIDATE_RC}"
printf 'bag:           %s\n' "${BAG_PATHS[@]}"
echo "precheck_json: ${PRECHECK_JSON}"
echo "summary_json:  ${SUMMARY_JSON}"
echo "validate_json: ${VALIDATION_JSON}"
echo "runtime_guard: ${RUNTIME_GUARD_STATUS}"
echo "fault_report:  ${RUNTIME_FAULT_CLASSIFICATION}"
echo "========================================================================"

if [ "${DRIVE_RC}" -ne 0 ]; then
    exit "${DRIVE_RC}"
fi
if bool_true "${S1_VALIDATE}" && [ "${VALIDATE_RC}" -ne 0 ]; then
    exit "${VALIDATE_RC}"
fi
exit 0
