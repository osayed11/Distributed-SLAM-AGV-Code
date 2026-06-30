#!/bin/bash
# start_session.sh - Run one managed dataset recording session.
#
# Usage:
#   ./start_session.sh <robot_name> <scenario>
#   REQUIRE_IMU=true REQUIRE_GT=true ./start_session.sh agv110 corridor_loop
#
# What it does:
#   1. Waits for clock sync before naming the session.
#   2. Captures pre-run chrony and hardware evidence.
#   3. Stops stale bringup/recording processes.
#   4. Starts ROS bringup and waits for required sensor topics.
#   5. Runs the required RealSense live gate on the active bringup.
#   6. Records ROS 2 data with MCAP/QoS overrides when available.
#   7. On Ctrl+C, stops recording cleanly, stops bringup, and finalizes the
#      manifest with watchdog, hardware, and RealSense fault evidence.
#
# Run this on the robot. It is location-independent as long as this repo is
# intact, e.g. ~/slam_project/scripts/logging/start_session.sh.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# ---------------------------------------------------------------------------
# Args
# ---------------------------------------------------------------------------
ROBOT_NAME="${1:-agv_unknown}"
SCENARIO="${2:-unknown_scenario}"
MOCAP_TOPIC="${MOCAP_TOPIC:-/optitrack/rigid_bodies/orkar_agv1}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
REQUIRE_GT="${REQUIRE_GT:-false}"
WAIT_FOR_CLOCK_SYNC="${WAIT_FOR_CLOCK_SYNC:-true}"
CLOCK_SYNC_TIMEOUT="${CLOCK_SYNC_TIMEOUT:-120}"
if [ -z "${REQUIRE_CLOCK_SYNC+x}" ]; then
    if [ "${REQUIRE_GT}" = true ]; then
        REQUIRE_CLOCK_SYNC=true
    else
        REQUIRE_CLOCK_SYNC=false
    fi
fi
REQUIRE_IMU="${REQUIRE_IMU:-false}"
IMU_TOPICS="${IMU_TOPICS:-/camera/imu /imu}"
ENABLE_IMU_RECORDING_KEEPALIVE="${ENABLE_IMU_RECORDING_KEEPALIVE:-false}"
IMU_RECORDING_KEEPALIVE_SECONDS="${IMU_RECORDING_KEEPALIVE_SECONDS:-30}"
ENABLE_REALSENSE_SYNC="${ENABLE_REALSENSE_SYNC:-false}"
ROSBAG2_MAX_CACHE_SIZE="${ROSBAG2_MAX_CACHE_SIZE:-536870912}"
ROSBAG2_MAX_BAG_SIZE="${ROSBAG2_MAX_BAG_SIZE:-2147483648}"
ROSBAG2_STORAGE_ID="${ROSBAG2_STORAGE_ID:-auto}"
ROSBAG2_STORAGE_ID_EFFECTIVE="${ROSBAG2_STORAGE_ID}"
ROSBAG2_STORAGE_CONFIG="${ROSBAG2_STORAGE_CONFIG:-${ROOT}/configs/sqlite_resilient.yaml}"
ROSBAG2_STORAGE_PRESET_PROFILE="${ROSBAG2_STORAGE_PRESET_PROFILE:-}"
ROSBAG2_QOS_OVERRIDES="${ROSBAG2_QOS_OVERRIDES:-${ROOT}/configs/rosbag2_sensor_qos.yaml}"
ROSBAG_STOP_TIMEOUT="${ROSBAG_STOP_TIMEOUT:-180}"
BRINGUP_STOP_TIMEOUT="${BRINGUP_STOP_TIMEOUT:-60}"
WATCHDOG_STOP_TIMEOUT="${WATCHDOG_STOP_TIMEOUT:-15}"
RUN_REALSENSE_CAMERA_GATE="${RUN_REALSENSE_CAMERA_GATE:-true}"
REALSENSE_CAMERA_GATE_SECONDS="${REALSENSE_CAMERA_GATE_SECONDS:-90}"
D455_RESET_MODE="${D455_RESET_MODE:-hardware-reset}"
D455_RESET_REENUMERATE_WAIT="${D455_RESET_REENUMERATE_WAIT:-15}"
STRICT_REALSENSE_UVC_LOG="${STRICT_REALSENSE_UVC_LOG:-false}"
RATE_EPSILON_HZ="${RATE_EPSILON_HZ:-0.05}"
REALSENSE_ACTIVE_RGBD_GAP_ABORT="${REALSENSE_ACTIVE_RGBD_GAP_ABORT:-false}"
REALSENSE_ACTIVE_IMU_GAP_ABORT="${REALSENSE_ACTIVE_IMU_GAP_ABORT:-false}"
ENABLE_RUNTIME_WATCHDOG="${ENABLE_RUNTIME_WATCHDOG:-false}"
ENABLE_RUNTIME_RGBD_WATCHDOG="${ENABLE_RUNTIME_RGBD_WATCHDOG:-false}"
ENABLE_RUNTIME_CAMERA_IMU_WATCHDOG="${ENABLE_RUNTIME_CAMERA_IMU_WATCHDOG:-false}"
RUNTIME_WATCHDOG_STARTUP_DELAY="${RUNTIME_WATCHDOG_STARTUP_DELAY:-15}"
RUNTIME_WATCHDOG_INTERVAL="${RUNTIME_WATCHDOG_INTERVAL:-20}"
RUNTIME_WATCHDOG_HZ_TIMEOUT="${RUNTIME_WATCHDOG_HZ_TIMEOUT:-12}"
RUNTIME_WATCHDOG_MAX_CONSECUTIVE_FAILURES="${RUNTIME_WATCHDOG_MAX_CONSECUTIVE_FAILURES:-2}"
RUNTIME_WATCHDOG_ABORT_ON_FAILURE="${RUNTIME_WATCHDOG_ABORT_ON_FAILURE:-false}"
MIN_RGBD_HZ="${MIN_RGBD_HZ:-12}"
MIN_CAMERA_IMU_HZ="${MIN_CAMERA_IMU_HZ:-150}"
MIN_SCAN_HZ="${MIN_SCAN_HZ:-5}"
MIN_ODOM_HZ="${MIN_ODOM_HZ:-10}"
MIN_GT_HZ="${MIN_GT_HZ:-5}"
RGBD_WARN_GATE_GAP_SEC="${RGBD_WARN_GATE_GAP_SEC:-0.25}"
MAX_RGBD_GATE_GAP_SEC="${MAX_RGBD_GATE_GAP_SEC:-0.75}"
MAX_CAMERA_IMU_GATE_GAP_SEC="${MAX_CAMERA_IMU_GATE_GAP_SEC:-0.10}"
RGBD_STARTUP_TIMEOUT="${RGBD_STARTUP_TIMEOUT:-90}"
IMU_STARTUP_TIMEOUT="${IMU_STARTUP_TIMEOUT:-30}"
MIN_REALSENSE_FPS="${MIN_REALSENSE_FPS:-15}"

CAMERA_COLOR_WIDTH="${CAMERA_COLOR_WIDTH:-640}"
CAMERA_COLOR_HEIGHT="${CAMERA_COLOR_HEIGHT:-480}"
CAMERA_COLOR_FPS="${CAMERA_COLOR_FPS:-15}"
CAMERA_DEPTH_WIDTH="${CAMERA_DEPTH_WIDTH:-640}"
CAMERA_DEPTH_HEIGHT="${CAMERA_DEPTH_HEIGHT:-480}"
CAMERA_DEPTH_FPS="${CAMERA_DEPTH_FPS:-15}"

clock_is_synced() {
    local timedate_synced
    local chrony_text

    if command -v timedatectl >/dev/null 2>&1; then
        timedate_synced="$(timedatectl show -p NTPSynchronized --value 2>/dev/null || true)"
        if [ "${timedate_synced}" = "yes" ]; then
            return 0
        fi
        timedate_synced="$(timedatectl show -p SystemClockSynchronized --value 2>/dev/null || true)"
        if [ "${timedate_synced}" = "yes" ]; then
            return 0
        fi
    fi

    if command -v chronyc >/dev/null 2>&1; then
        chrony_text="$(chronyc tracking 2>/dev/null || true)"
        printf "%s\n" "${chrony_text}" | grep -q "Leap status[[:space:]]*: Normal" || return 1
        printf "%s\n" "${chrony_text}" | awk -F: '
            /System time/ {
                value = $2
                gsub(/^[[:space:]]+/, "", value)
                split(value, parts, " ")
                offset = parts[1] + 0
                if (offset < 0) {
                    offset = -offset
                }
                exit(offset <= 0.100 ? 0 : 1)
            }
            END {
                if (NR == 0) {
                    exit 1
                }
            }
        ' && return 0
    fi

    return 1
}

wait_for_clock_sync_before_session_id() {
    local end

    if [ "${WAIT_FOR_CLOCK_SYNC}" != true ]; then
        echo "  [i] clock sync wait disabled before session naming."
        return 0
    fi

    if clock_is_synced; then
        echo "  [OK] clock is synced before session naming."
        return 0
    fi

    echo "  [i] waiting up to ${CLOCK_SYNC_TIMEOUT}s for clock sync before session naming..."
    end=$((SECONDS + CLOCK_SYNC_TIMEOUT))
    while [ "${SECONDS}" -lt "${end}" ]; do
        sleep 2
        if clock_is_synced; then
            echo "  [OK] clock synced before session naming."
            return 0
        fi
    done

    if [ "${REQUIRE_CLOCK_SYNC}" = true ]; then
        echo "ERROR: clock did not sync within ${CLOCK_SYNC_TIMEOUT}s; refusing dataset session." >&2
        echo "       Check chrony/NTP/network before recording multi-robot data." >&2
        exit 1
    fi

    echo "  [WARN] clock did not sync within ${CLOCK_SYNC_TIMEOUT}s; continuing because REQUIRE_CLOCK_SYNC=false." >&2
}

wait_for_clock_sync_before_session_id
DATESTAMP=$(date +%Y%m%d_%H%M%S)
SESSION_ID="${ROBOT_NAME}_${SCENARIO}_${DATESTAMP}"
BAG_DIR="${BAG_DIR:-${HOME}/agv_data}"
BAG_PATH="${BAG_DIR}/${SESSION_ID}"
MANIFEST_FILE="${BAG_DIR}/${SESSION_ID}_manifest.yaml"
CHRONY_FILE="${BAG_DIR}/${SESSION_ID}_chrony.txt"
CAMERA_GATE_PRE_LOG="${BAG_DIR}/${SESSION_ID}_camera_gate_pre.log"
CAMERA_GATE_POST_LOG="${BAG_DIR}/${SESSION_ID}_camera_gate_post.log"
HARDWARE_PRE_LOG="${BAG_DIR}/${SESSION_ID}_hardware_pre.log"
HARDWARE_POST_LOG="${BAG_DIR}/${SESSION_ID}_hardware_post.log"
RUNTIME_WATCHDOG_LOG="${BAG_DIR}/${SESSION_ID}_runtime_watchdog.log"
RUNTIME_WATCHDOG_STATUS_FILE="${BAG_DIR}/${SESSION_ID}_runtime_watchdog.status"
KERNEL_RUNTIME_LOG="${BAG_DIR}/${SESSION_ID}_kernel_runtime.log"
FAULT_CLASSIFICATION_FILE="${BAG_DIR}/${SESSION_ID}_realsense_fault_classification.txt"
KERNEL_RUNTIME_START_LINE=""

mkdir -p "${BAG_DIR}"

require_min_fps() {
    local value="$1"
    local name="$2"

    if ! printf "%s" "${value}" | grep -Eq '^[0-9]+([.][0-9]+)?$'; then
        echo "ERROR: ${name} must be numeric, got '${value}'." >&2
        exit 1
    fi
    if ! awk -v fps="${value}" -v min="${MIN_REALSENSE_FPS}" 'BEGIN { exit(fps >= min ? 0 : 1) }'; then
        echo "ERROR: ${name}=${value} is below ${MIN_REALSENSE_FPS} FPS." >&2
        echo "       Keep D455 hardware streams at >=${MIN_REALSENSE_FPS} FPS; drop frames downstream if needed." >&2
        exit 1
    fi
}

require_min_fps "${CAMERA_COLOR_FPS}" "CAMERA_COLOR_FPS"
require_min_fps "${CAMERA_DEPTH_FPS}" "CAMERA_DEPTH_FPS"

# ---------------------------------------------------------------------------
# Source ROS 2
# ---------------------------------------------------------------------------
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
else
    echo "ERROR: no supported ROS 2 setup found under /opt/ros (expected Humble or Iron)." >&2
    exit 1
fi

if [ -f "${ROOT}/agv2_ws/install/setup.bash" ]; then
    source "${ROOT}/agv2_ws/install/setup.bash"
else
    echo "ERROR: missing ROS 2 workspace setup: ${ROOT}/agv2_ws/install/setup.bash" >&2
    echo "       Run bash scripts/setup_robot_ros2.sh ${ROBOT_NAME} before recording." >&2
    exit 1
fi

ROS_VERSION=2
if [ "${ROSBAG2_STORAGE_ID}" = "auto" ]; then
    if ros2 bag record --help 2>/dev/null | grep -Eq '\bmcap\b'; then
        ROSBAG2_STORAGE_ID_EFFECTIVE="mcap"
    else
        ROSBAG2_STORAGE_ID_EFFECTIVE="sqlite3"
    fi
else
    ROSBAG2_STORAGE_ID_EFFECTIVE="${ROSBAG2_STORAGE_ID}"
fi

# ---------------------------------------------------------------------------
# Pre-flight checks
# ---------------------------------------------------------------------------
echo "=== Pre-flight checks ==="

# Record clock-sync state before every run. This is evidence that robot sensor
# stamps can be compared with mocap stamps from a chrony-synced GT machine.
{
    echo "# Chrony snapshot for ${SESSION_ID}"
    echo "# Captured: $(date --iso-8601=ns)"
    echo ""
    if command -v chronyc >/dev/null 2>&1; then
        echo "## chronyc tracking"
        chronyc tracking 2>&1 || true
        echo ""
        echo "## chronyc sources -v"
        chronyc sources -v 2>&1 || true
    else
        echo "chronyc not installed"
    fi
} > "${CHRONY_FILE}"
echo "  [i] chrony snapshot: ${CHRONY_FILE}"

capture_hardware_snapshot() {
    local label="$1"
    local file="$2"

    {
        echo "# Hardware snapshot (${label}) for ${SESSION_ID}"
        echo "# Captured: $(date --iso-8601=ns)"
        echo ""
        echo "## host"
        hostname
        hostname -I 2>/dev/null || true
        uname -a
        echo ""
        echo "## Pi power/throttle"
        command -v vcgencmd >/dev/null 2>&1 && vcgencmd get_throttled || echo "vcgencmd unavailable"
        command -v vcgencmd >/dev/null 2>&1 && vcgencmd measure_volts core || true
        echo ""
        echo "## USB autosuspend"
        [ -r /sys/module/usbcore/parameters/autosuspend ] && \
            cat /sys/module/usbcore/parameters/autosuspend || true
        echo ""
        echo "## WiFi power save"
        command -v iw >/dev/null 2>&1 && iw dev wlan0 get power_save 2>&1 || true
        echo ""
        echo "## USB topology"
        lsusb -t 2>&1 || true
        echo ""
        echo "## D455 sysfs"
        for p in /sys/bus/usb/devices/*/idProduct; do
            if [ "$(cat "$p" 2>/dev/null)" = "0b5c" ]; then
                local d
                d="$(dirname "$p")"
                echo "device=${d}"
                for f in product manufacturer serial speed busnum devnum bMaxPower power/control power/autosuspend; do
                    [ -e "${d}/${f}" ] && printf "%s=" "${f}" && cat "${d}/${f}"
                done
            fi
        done
        echo ""
        echo "## recent USB/camera kernel log"
        dmesg -T 2>/dev/null | \
            grep -Ei 'usb|uvc|video|hid|iio|realsense|under-voltage|voltage|reset|disconnect|timeout|error' | \
            tail -120 || true
    } > "${file}" 2>&1 || true
}

kernel_line_count() {
    if sudo -n true >/dev/null 2>&1; then
        sudo -n dmesg -T 2>/dev/null | wc -l | awk '{print $1}'
    else
        dmesg -T 2>/dev/null | wc -l | awk '{print $1}'
    fi
}

capture_runtime_kernel_log() {
    local all_log
    all_log="${BAG_DIR}/${SESSION_ID}_kernel_all_after.log"

    if sudo -n true >/dev/null 2>&1; then
        sudo -n dmesg -T > "${all_log}" 2>&1 || true
    else
        dmesg -T > "${all_log}" 2>&1 || true
    fi

    if [ -n "${KERNEL_RUNTIME_START_LINE}" ] && \
       printf "%s" "${KERNEL_RUNTIME_START_LINE}" | grep -Eq '^[0-9]+$'; then
        tail -n "+$((KERNEL_RUNTIME_START_LINE + 1))" "${all_log}" > "${KERNEL_RUNTIME_LOG}" 2>/dev/null || \
            cp "${all_log}" "${KERNEL_RUNTIME_LOG}" 2>/dev/null || true
    else
        cp "${all_log}" "${KERNEL_RUNTIME_LOG}" 2>/dev/null || true
    fi
}

capture_hardware_snapshot "pre-run" "${HARDWARE_PRE_LOG}"
echo "  [i] hardware snapshot: ${HARDWARE_PRE_LOG}"

# Check required topics are publishing (best-effort, bounded timeout).
# If logging.launch is allowed to start bringup itself these checks may warn
# before sensors exist; validate_ros2_bag.py remains the authoritative post-run gate.
REQUIRED_TOPICS="/scan /odom /tf /camera/color/image_raw /camera/aligned_depth_to_color/image_raw"
OPTIONAL_TOPICS=""
GROUND_TRUTH_TOPICS="${MOCAP_TOPIC} /mocap"
ALL_OK=true

_topic_list_ok() {
    timeout 3 ros2 topic list > /dev/null 2>&1
}

_topic_hz_ok() {
    local topic="$1"
    timeout 6 ros2 topic hz --window 10 "$topic" 2>/dev/null | grep -q "average rate"
}

_topic_echo_ok() {
    local topic="$1"
    timeout 3 ros2 topic echo --once "$topic" > /dev/null 2>&1
}

echo "  [i] Skipping pre-flight topic probes; sensor gate runs after bringup."

if [ "$ALL_OK" = false ]; then
    echo ""
    echo "WARNING: Some topics not detected. Starting logging anyway."
    echo "Run validate_ros2_bag.py after recording to check data quality."
    echo ""
fi

# ---------------------------------------------------------------------------
# Write initial manifest
# ---------------------------------------------------------------------------
ROS_DISTRO_VAL=$(echo "${ROS_DISTRO:-humble}")
CALIB_DIR="${ROOT}/agv2_ws/src/agv_bringup/calibration"
if [ -d "$CALIB_DIR" ]; then
    CALIB_HASH=$(find "$CALIB_DIR" -type f | sort | xargs sha256sum 2>/dev/null | sha256sum | cut -d' ' -f1)
else
    CALIB_HASH="unavailable"
fi

cat > "${MANIFEST_FILE}" << EOF
# Session manifest - auto-generated by start_session.sh
session_id: ${SESSION_ID}
robot_id: ${ROBOT_NAME}
scenario: ${SCENARIO}
date: $(date +%Y-%m-%d)
time_start: $(date +%H:%M:%S)
time_end: ~
operator: $(whoami)
ros_distro: ${ROS_DISTRO_VAL}
bag_dir: ${SESSION_ID}
bag_file: ${SESSION_ID}
chrony_file: ${SESSION_ID}_chrony.txt
bag_size_mb: ~
duration_sec: ~
calibration_hash: "sha256:${CALIB_HASH}"
mocap_topic: "${MOCAP_TOPIC}"
cmd_topic: "${CMD_TOPIC}"
ground_truth_required: ${REQUIRE_GT}
wait_for_clock_sync: ${WAIT_FOR_CLOCK_SYNC}
clock_sync_timeout_sec: ${CLOCK_SYNC_TIMEOUT}
clock_sync_required: ${REQUIRE_CLOCK_SYNC}
imu_required: ${REQUIRE_IMU}
imu_topics: "${IMU_TOPICS}"
imu_recording_keepalive_enabled: ${ENABLE_IMU_RECORDING_KEEPALIVE}
imu_recording_keepalive_seconds: ${IMU_RECORDING_KEEPALIVE_SECONDS}
camera_imu: enabled
enable_realsense_sync: ${ENABLE_REALSENSE_SYNC}
rosbag2_max_cache_size_bytes: ${ROSBAG2_MAX_CACHE_SIZE}
rosbag2_max_bag_size_bytes: ${ROSBAG2_MAX_BAG_SIZE}
rosbag2_storage_id_requested: "${ROSBAG2_STORAGE_ID}"
rosbag2_storage_id_effective: "${ROSBAG2_STORAGE_ID_EFFECTIVE}"
rosbag2_storage_config: "${ROSBAG2_STORAGE_CONFIG}"
rosbag2_storage_preset_profile: "${ROSBAG2_STORAGE_PRESET_PROFILE}"
rosbag2_qos_overrides: "${ROSBAG2_QOS_OVERRIDES}"
rosbag_stop_timeout_sec: ${ROSBAG_STOP_TIMEOUT}
bringup_stop_timeout_sec: ${BRINGUP_STOP_TIMEOUT}
d455_reset_mode: "${D455_RESET_MODE}"
d455_reset_reenumerate_wait_sec: ${D455_RESET_REENUMERATE_WAIT}
realsense_camera_gate_required: ${RUN_REALSENSE_CAMERA_GATE}
realsense_camera_gate_seconds: ${REALSENSE_CAMERA_GATE_SECONDS}
rgbd_warn_gate_gap_sec: ${RGBD_WARN_GATE_GAP_SEC}
rgbd_hard_gate_gap_sec: ${MAX_RGBD_GATE_GAP_SEC}
camera_imu_hard_gate_gap_sec: ${MAX_CAMERA_IMU_GATE_GAP_SEC}
realsense_active_rgbd_gap_abort: ${REALSENSE_ACTIVE_RGBD_GAP_ABORT}
realsense_active_imu_gap_abort: ${REALSENSE_ACTIVE_IMU_GAP_ABORT}
realsense_camera_gate_pre_log: ${SESSION_ID}_camera_gate_pre.log
realsense_camera_gate_post_log: ${SESSION_ID}_camera_gate_post.log
strict_realsense_uvc_log: ${STRICT_REALSENSE_UVC_LOG}
rate_epsilon_hz: ${RATE_EPSILON_HZ}
realsense_active_rgbd_gap_abort: ${REALSENSE_ACTIVE_RGBD_GAP_ABORT}
runtime_watchdog_enabled: ${ENABLE_RUNTIME_WATCHDOG}
runtime_rgbd_watchdog_enabled: ${ENABLE_RUNTIME_RGBD_WATCHDOG}
runtime_camera_imu_watchdog_enabled: ${ENABLE_RUNTIME_CAMERA_IMU_WATCHDOG}
runtime_watchdog_abort_on_failure: ${RUNTIME_WATCHDOG_ABORT_ON_FAILURE}
runtime_watchdog_log: ${SESSION_ID}_runtime_watchdog.log
runtime_watchdog_status: ~
rgbd_startup_timeout_sec: ${RGBD_STARTUP_TIMEOUT}
imu_startup_timeout_sec: ${IMU_STARTUP_TIMEOUT}
hardware_pre_log: ${SESSION_ID}_hardware_pre.log
hardware_post_log: ${SESSION_ID}_hardware_post.log
kernel_runtime_log: ${SESSION_ID}_kernel_runtime.log
realsense_fault_classification_log: ${SESSION_ID}_realsense_fault_classification.txt

camera_profile:
  color_width: ${CAMERA_COLOR_WIDTH}
  color_height: ${CAMERA_COLOR_HEIGHT}
  color_fps: ${CAMERA_COLOR_FPS}
  depth_width: ${CAMERA_DEPTH_WIDTH}
  depth_height: ${CAMERA_DEPTH_HEIGHT}
  depth_fps: ${CAMERA_DEPTH_FPS}
notes: ""
usb_mode_note: "D455 observed on USB 3.x; RGB-D and /camera/imu are recorded when available."
EOF

echo ""
echo "=== Session: ${SESSION_ID} ==="
echo "Bag:      ${BAG_PATH}"
echo "Manifest: ${MANIFEST_FILE}"
echo ""
echo "Press Ctrl+C to stop recording."
echo ""

# ---------------------------------------------------------------------------
# Launch bringup, wait for sensors, then record.
# ---------------------------------------------------------------------------
START_EPOCH=$(date +%s)
BRINGUP_PID=""
BRINGUP_PGID=""

ROSBAG_PID=""
WATCHDOG_PID=""
IMU_KEEPALIVE_PID=""
RECORDING_STARTED=false
CLEANED_UP=false

wait_or_kill() {
    local pid="$1"
    local label="$2"
    local timeout_s="${3:-20}"
    local end=$((SECONDS + timeout_s))

    if [ -z "${pid}" ] || ! kill -0 "${pid}" 2>/dev/null; then
        return 0
    fi

    while [ "${SECONDS}" -lt "${end}" ]; do
        if ! kill -0 "${pid}" 2>/dev/null; then
            wait "${pid}" 2>/dev/null || true
            return 0
        fi
        sleep 1
    done

    echo "  [WARN] ${label} did not exit after ${timeout_s}s; sending SIGTERM."
    kill -TERM "${pid}" 2>/dev/null || true
    sleep 5
    if kill -0 "${pid}" 2>/dev/null; then
        echo "  [WARN] ${label} still running; sending SIGKILL."
        kill -KILL "${pid}" 2>/dev/null || true
    fi
    wait "${pid}" 2>/dev/null || true
}

wait_or_kill_group() {
    local pgid="$1"
    local pid="$2"
    local label="$3"
    local timeout_s="${4:-20}"
    local end=$((SECONDS + timeout_s))

    if [ -z "${pgid}" ] || ! kill -0 "-${pgid}" 2>/dev/null; then
        wait_or_kill "${pid}" "${label}" "${timeout_s}"
        return
    fi

    while [ "${SECONDS}" -lt "${end}" ]; do
        if ! kill -0 "-${pgid}" 2>/dev/null; then
            wait "${pid}" 2>/dev/null || true
            return
        fi
        sleep 1
    done

    echo "  [WARN] ${label} process group did not exit after ${timeout_s}s; sending SIGTERM."
    kill -TERM "-${pgid}" 2>/dev/null || true
    sleep 5
    if kill -0 "-${pgid}" 2>/dev/null; then
        echo "  [WARN] ${label} process group still running; sending SIGKILL."
        kill -KILL "-${pgid}" 2>/dev/null || true
    fi
    wait "${pid}" 2>/dev/null || true
}

finalise_manifest() {
    echo ""
    echo "=== Finalising manifest ==="
    END_EPOCH=$(date +%s)
    DURATION=$((END_EPOCH - START_EPOCH))
    WATCHDOG_STATUS="not_started"

    if [ -d "${BAG_PATH}" ]; then
        BAG_SIZE_MB=$(du -sm "${BAG_PATH}" 2>/dev/null | cut -f1)
    else
        BAG_SIZE_MB="~"
    fi

    # Update manifest with final values
    if [ -s "${RUNTIME_WATCHDOG_STATUS_FILE}" ]; then
        WATCHDOG_STATUS="$(head -1 "${RUNTIME_WATCHDOG_STATUS_FILE}" | tr -cd 'A-Za-z0-9_ .:=-' | sed 's/[[:space:]]*$//')"
    fi
    sed -i "s/time_end: ~/time_end: $(date +%H:%M:%S)/" "${MANIFEST_FILE}"
    sed -i "s/bag_size_mb: ~/bag_size_mb: ${BAG_SIZE_MB:-unknown}/" "${MANIFEST_FILE}"
    sed -i "s/duration_sec: ~/duration_sec: ${DURATION}/" "${MANIFEST_FILE}"
    sed -i "s/runtime_watchdog_status: ~/runtime_watchdog_status: \"${WATCHDOG_STATUS}\"/" "${MANIFEST_FILE}"

    echo "Duration: ${DURATION}s"
    echo "Bag size: ${BAG_SIZE_MB:-unknown} MB"
    echo "Runtime watchdog: ${WATCHDOG_STATUS}"
    echo "Manifest written: ${MANIFEST_FILE}"
    echo ""
    echo "Run quality check:"
    echo "  REQUIRE_IMU=${REQUIRE_IMU} CMD_TOPIC=${CMD_TOPIC} python3 scripts/logging/validate_ros2_bag.py ${BAG_PATH} --require-resilient-storage"
}

run_camera_pre_gate() {
    if [ "${RUN_REALSENSE_CAMERA_GATE}" != "true" ]; then
        return 0
    fi

    local color_log="${BAG_DIR}/${SESSION_ID}_camera_gate_pre_color_hz.txt"
    local depth_log="${BAG_DIR}/${SESSION_ID}_camera_gate_pre_aligned_depth_hz.txt"
    local imu_log="${BAG_DIR}/${SESSION_ID}_camera_gate_pre_imu_hz.txt"
    local gyro_log="${BAG_DIR}/${SESSION_ID}_camera_gate_pre_gyro_hz.txt"
    local accel_log="${BAG_DIR}/${SESSION_ID}_camera_gate_pre_accel_hz.txt"
    local gate_bringup_log="${BAG_DIR}/${SESSION_ID}_camera_gate_bringup_window.log"
    local gate_start_line=0
    local failures=0
    local rate

    echo "Running required RealSense live pre-run gate (${REALSENSE_CAMERA_GATE_SECONDS}s stream test on active bringup)..."
    echo "  log: ${CAMERA_GATE_PRE_LOG}"

    {
        echo "# RealSense live pre-run gate"
        echo "# session: ${SESSION_ID}"
        echo "# captured: $(date --iso-8601=seconds)"
        echo "stream_seconds: ${REALSENSE_CAMERA_GATE_SECONDS}"
        echo "min_rgbd_hz: ${MIN_RGBD_HZ}"
        echo "min_camera_imu_hz: ${MIN_CAMERA_IMU_HZ}"
        echo "rgbd_warn_gate_gap_sec: ${RGBD_WARN_GATE_GAP_SEC}"
        echo "rgbd_hard_gate_gap_sec: ${MAX_RGBD_GATE_GAP_SEC}"
        echo "camera_imu_hard_gate_gap_sec: ${MAX_CAMERA_IMU_GATE_GAP_SEC}"
        echo "rate_epsilon_hz: ${RATE_EPSILON_HZ}"
        echo "active_rgbd_gap_abort: ${REALSENSE_ACTIVE_RGBD_GAP_ABORT}"
        echo "active_imu_gap_abort: ${REALSENSE_ACTIVE_IMU_GAP_ABORT}"
        echo "color_log: $(basename "${color_log}")"
        echo "aligned_depth_log: $(basename "${depth_log}")"
        echo "imu_log: $(basename "${imu_log}")"
        echo "gyro_log: $(basename "${gyro_log}")"
        echo "accel_log: $(basename "${accel_log}")"
        echo "bringup_window_log: $(basename "${gate_bringup_log}")"
        echo ""
    } > "${CAMERA_GATE_PRE_LOG}"

    gate_start_line="$(wc -l < "${BRINGUP_LOG}" 2>/dev/null || echo 0)"
    timeout "${REALSENSE_CAMERA_GATE_SECONDS}" ros2 topic hz /camera/color/image_raw --window 40 \
        > "${color_log}" 2>&1 &
    local color_pid=$!
    timeout "${REALSENSE_CAMERA_GATE_SECONDS}" ros2 topic hz /camera/aligned_depth_to_color/image_raw --window 40 \
        > "${depth_log}" 2>&1 &
    local depth_pid=$!
    timeout "${REALSENSE_CAMERA_GATE_SECONDS}" ros2 topic hz /camera/imu --window 80 \
        > "${imu_log}" 2>&1 &
    local imu_pid=$!

    wait "${color_pid}" 2>/dev/null || true
    wait "${depth_pid}" 2>/dev/null || true
    wait "${imu_pid}" 2>/dev/null || true

    if printf "%s" "${gate_start_line}" | grep -Eq '^[0-9]+$'; then
        tail -n "+$((gate_start_line + 1))" "${BRINGUP_LOG}" > "${gate_bringup_log}" 2>/dev/null || \
            cp "${BRINGUP_LOG}" "${gate_bringup_log}" 2>/dev/null || true
    else
        cp "${BRINGUP_LOG}" "${gate_bringup_log}" 2>/dev/null || true
    fi

    _camera_rate_from_log() {
        awk '/average rate:/ { print $3 }' "$1" | sort -n | awk '
            { rates[NR] = $1 }
            END {
                if (NR == 0) {
                    exit
                }
                if (NR % 2 == 1) {
                    print rates[(NR + 1) / 2]
                } else {
                    printf "%.3f\n", (rates[NR / 2] + rates[(NR / 2) + 1]) / 2
                }
            }
        '
    }

    _camera_max_gap_from_log() {
        local file="$1"
        local min_window="$2"
        awk -v min_window="${min_window}" '
            /min: .* max: .* window:/ {
                max_gap = ""
                window = ""
                for (i = 1; i <= NF; i++) {
                    if ($i == "max:") {
                        max_gap = $(i + 1)
                        gsub("s", "", max_gap)
                    }
                    if ($i == "window:") {
                        window = $(i + 1)
                    }
                }
                if (max_gap != "" && window + 0 >= min_window && max_gap + 0 > max) {
                    max = max_gap + 0
                }
            }
            END { if (max != "") print max }
        ' "${file}"
    }

    _camera_check_rate_log() {
        local topic="$1"
        local file="$2"
        local min_rate="$3"
        local label="$4"
        local warn_gap_limit="$5"
        local hard_gap_limit="$6"
        local min_gap_window="$7"
        local abort_on_hard_gap="${8:-true}"
        local max_gap

        rate="$(_camera_rate_from_log "${file}")"
        if [ -z "${rate}" ]; then
            echo "FAIL ${label}: no average rate for ${topic}; see ${file}" | tee -a "${CAMERA_GATE_PRE_LOG}"
            return 1
        fi
        if awk -v rate="${rate}" -v min="${min_rate}" 'BEGIN { exit(rate >= min ? 0 : 1) }'; then
            echo "PASS ${label}: ${topic} ${rate} Hz" | tee -a "${CAMERA_GATE_PRE_LOG}"
        elif awk -v rate="${rate}" -v min="${min_rate}" -v eps="${RATE_EPSILON_HZ}" 'BEGIN { exit(rate + eps >= min ? 0 : 1) }'; then
            echo "WARN ${label}: ${topic} ${rate} Hz is within ${RATE_EPSILON_HZ} Hz of required ${min_rate} Hz" | tee -a "${CAMERA_GATE_PRE_LOG}"
        else
            echo "FAIL ${label}: ${topic} ${rate} Hz, expected >= ${min_rate} Hz" | tee -a "${CAMERA_GATE_PRE_LOG}"
            return 1
        fi

        max_gap="$(_camera_max_gap_from_log "${file}" "${min_gap_window}")"
        if [ -z "${max_gap}" ]; then
            echo "WARN ${label}: no steady-state max-gap data found for ${topic} after window ${min_gap_window}" | tee -a "${CAMERA_GATE_PRE_LOG}"
        elif awk -v gap="${max_gap}" -v limit="${warn_gap_limit}" 'BEGIN { exit(gap <= limit ? 0 : 1) }'; then
            echo "PASS ${label} steady max gap: ${max_gap}s <= warning ${warn_gap_limit}s after window ${min_gap_window}" | tee -a "${CAMERA_GATE_PRE_LOG}"
        elif awk -v gap="${max_gap}" -v limit="${hard_gap_limit}" 'BEGIN { exit(gap <= limit ? 0 : 1) }'; then
            echo "WARN ${label} steady max gap: ${max_gap}s exceeds warning ${warn_gap_limit}s but is <= hard ${hard_gap_limit}s after window ${min_gap_window}" | tee -a "${CAMERA_GATE_PRE_LOG}"
        elif [ "${abort_on_hard_gap}" != true ]; then
            echo "WARN ${label} steady max gap: ${max_gap}s exceeds active-monitor hard ${hard_gap_limit}s after window ${min_gap_window}; post-run bag validation is authoritative" | tee -a "${CAMERA_GATE_PRE_LOG}"
        else
            echo "FAIL ${label} steady max gap: ${max_gap}s exceeds hard ${hard_gap_limit}s after window ${min_gap_window}" | tee -a "${CAMERA_GATE_PRE_LOG}"
            return 1
        fi
        return 0
    }

    _camera_check_rate_log /camera/color/image_raw "${color_log}" "${MIN_RGBD_HZ}" "color stream" "${RGBD_WARN_GATE_GAP_SEC}" "${MAX_RGBD_GATE_GAP_SEC}" 40 "${REALSENSE_ACTIVE_RGBD_GAP_ABORT}" || failures=$((failures + 1))
    _camera_check_rate_log /camera/aligned_depth_to_color/image_raw "${depth_log}" "${MIN_RGBD_HZ}" "aligned depth stream" "${RGBD_WARN_GATE_GAP_SEC}" "${MAX_RGBD_GATE_GAP_SEC}" 40 "${REALSENSE_ACTIVE_RGBD_GAP_ABORT}" || failures=$((failures + 1))
    if ! _camera_check_rate_log /camera/imu "${imu_log}" "${MIN_CAMERA_IMU_HZ}" "camera imu stream" "${MAX_CAMERA_IMU_GATE_GAP_SEC}" "${MAX_CAMERA_IMU_GATE_GAP_SEC}" 80 "${REALSENSE_ACTIVE_IMU_GAP_ABORT}"; then
        echo "WARN camera imu stream: fused /camera/imu failed; checking raw gyro+accel fallback" | tee -a "${CAMERA_GATE_PRE_LOG}"
        timeout 30 ros2 topic hz /camera/gyro/sample --window 80 > "${gyro_log}" 2>&1 || true
        timeout 30 ros2 topic hz /camera/accel/sample --window 40 > "${accel_log}" 2>&1 || true
        raw_failures=0
        _camera_check_rate_log /camera/gyro/sample "${gyro_log}" "150" "camera gyro stream" "${MAX_CAMERA_IMU_GATE_GAP_SEC}" "${MAX_CAMERA_IMU_GATE_GAP_SEC}" 80 || raw_failures=$((raw_failures + 1))
        _camera_check_rate_log /camera/accel/sample "${accel_log}" "60" "camera accel stream" "${MAX_CAMERA_IMU_GATE_GAP_SEC}" "${MAX_CAMERA_IMU_GATE_GAP_SEC}" 40 || raw_failures=$((raw_failures + 1))
        if [ "${raw_failures}" -eq 0 ]; then
            echo "PASS camera imu fallback: raw gyro+accel streams satisfy IMU gate" | tee -a "${CAMERA_GATE_PRE_LOG}"
        else
            failures=$((failures + 1))
        fi
    fi

    if grep -Eiq "The device has been disconnected|USB disconnect|No such device|device removed" "${gate_bringup_log}" 2>/dev/null; then
        echo "FAIL RealSense runtime log: camera disconnect/device-drop errors observed" | tee -a "${CAMERA_GATE_PRE_LOG}"
        failures=$((failures + 1))
    elif grep -Eiq "UVCIOC_CTRL_QUERY|VIDIOC_|Frames didn't arrived|control_transfer.*failed|Connection timed out|Failed to create device|set_xu" "${gate_bringup_log}" 2>/dev/null; then
        if [ "${STRICT_REALSENSE_UVC_LOG}" = true ]; then
            echo "FAIL RealSense runtime log: UVC/control timeout text observed" | tee -a "${CAMERA_GATE_PRE_LOG}"
            failures=$((failures + 1))
        else
            echo "WARN RealSense runtime log: UVC/control timeout text observed; stream rates decide pass/fail" | tee -a "${CAMERA_GATE_PRE_LOG}"
        fi
    else
        echo "PASS RealSense runtime log: no UVC/control timeout text observed" | tee -a "${CAMERA_GATE_PRE_LOG}"
    fi

    if [ "${failures}" -ne 0 ]; then
        echo "ERROR: RealSense live pre-run gate failed. Not starting publishable data collection." >&2
        tail -80 "${CAMERA_GATE_PRE_LOG}" 2>/dev/null || true
        exit 1
    fi

    echo "  [OK] RealSense live pre-run gate passed."
}

watchdog_rate_check() {
    local topic="$1"
    local min_rate="$2"
    local timeout_s="${3:-${RUNTIME_WATCHDOG_HZ_TIMEOUT}}"
    local window="${4:-20}"
    local out
    local line
    local rate

    out=$(timeout "${timeout_s}" ros2 topic hz --window "${window}" "${topic}" 2>&1 || true)
    line=$(printf "%s\n" "${out}" | grep "average rate" | tail -1 || true)
    if [ -z "${line}" ]; then
        {
            echo "FAIL ${topic}: no average rate within ${timeout_s}s"
            printf "%s\n" "${out}" | tail -5
        } >> "${RUNTIME_WATCHDOG_LOG}"
        return 1
    fi

    rate=$(printf "%s\n" "${line}" | awk -F': ' '{print $2}' | awk '{print $1}')
    if awk -v rate="${rate}" -v min="${min_rate}" 'BEGIN { exit(rate >= min ? 0 : 1) }'; then
        echo "PASS ${topic}: ${rate} Hz" >> "${RUNTIME_WATCHDOG_LOG}"
        return 0
    fi

    echo "FAIL ${topic}: ${rate} Hz, expected >= ${min_rate} Hz" >> "${RUNTIME_WATCHDOG_LOG}"
    return 1
}

watchdog_liveness_check() {
    local topic="$1"
    local timeout_s="${2:-6}"

    if timeout "${timeout_s}" ros2 topic echo --once "${topic}" > /dev/null 2>&1; then
        echo "PASS ${topic}: live sample received" >> "${RUNTIME_WATCHDOG_LOG}"
        return 0
    fi

    echo "FAIL ${topic}: no live sample within ${timeout_s}s" >> "${RUNTIME_WATCHDOG_LOG}"
    return 1
}

run_runtime_watchdog() {
    local cycle=0
    local failures

    {
        echo "# Runtime sensor watchdog for ${SESSION_ID}"
        echo "# Started: $(date --iso-8601=ns)"
        echo "startup_delay_sec: ${RUNTIME_WATCHDOG_STARTUP_DELAY}"
        echo "interval_sec: ${RUNTIME_WATCHDOG_INTERVAL}"
        echo "hz_timeout_sec: ${RUNTIME_WATCHDOG_HZ_TIMEOUT}"
        echo "max_consecutive_failure_cycles: ${RUNTIME_WATCHDOG_MAX_CONSECUTIVE_FAILURES}"
        echo "abort_on_failure: ${RUNTIME_WATCHDOG_ABORT_ON_FAILURE}"
        echo "runtime_rgbd_watchdog_enabled: ${ENABLE_RUNTIME_RGBD_WATCHDOG}"
        echo "runtime_camera_imu_watchdog_enabled: ${ENABLE_RUNTIME_CAMERA_IMU_WATCHDOG}"
        echo "min_scan_hz: ${MIN_SCAN_HZ}"
        echo "min_odom_hz: ${MIN_ODOM_HZ}"
        echo "min_rgbd_hz: ${MIN_RGBD_HZ}"
        echo "min_camera_imu_hz: ${MIN_CAMERA_IMU_HZ}"
        echo "require_gt: ${REQUIRE_GT}"
        echo "mocap_topic: ${MOCAP_TOPIC}"
        echo ""
    } > "${RUNTIME_WATCHDOG_LOG}"
    echo "RUNNING" > "${RUNTIME_WATCHDOG_STATUS_FILE}"

    local consecutive_failure_cycles=0
    sleep "${RUNTIME_WATCHDOG_STARTUP_DELAY}"
    while [ -n "${ROSBAG_PID}" ] && kill -0 "${ROSBAG_PID}" 2>/dev/null; do
        cycle=$((cycle + 1))
        failures=0
        {
            echo ""
            echo "== watchdog cycle ${cycle}: $(date --iso-8601=seconds) =="
        } >> "${RUNTIME_WATCHDOG_LOG}"

        watchdog_rate_check /scan "${MIN_SCAN_HZ}" "${RUNTIME_WATCHDOG_HZ_TIMEOUT}" 20 || failures=$((failures + 1))
        watchdog_rate_check /odom "${MIN_ODOM_HZ}" "${RUNTIME_WATCHDOG_HZ_TIMEOUT}" 20 || failures=$((failures + 1))
        if [ "${ENABLE_RUNTIME_RGBD_WATCHDOG}" = true ]; then
            watchdog_liveness_check /camera/color/image_raw 8 || failures=$((failures + 1))
            watchdog_liveness_check /camera/aligned_depth_to_color/image_raw 8 || failures=$((failures + 1))
        else
            echo "SKIP /camera/color/image_raw: high-bandwidth stream checked by pre-run gate and post-run bag audit" >> "${RUNTIME_WATCHDOG_LOG}"
            echo "SKIP /camera/aligned_depth_to_color/image_raw: high-bandwidth stream checked by pre-run gate and post-run bag audit" >> "${RUNTIME_WATCHDOG_LOG}"
        fi
        if [ "${ENABLE_RUNTIME_CAMERA_IMU_WATCHDOG}" = true ]; then
            watchdog_liveness_check /camera/imu 8 || failures=$((failures + 1))
        else
            echo "SKIP /camera/imu: high-rate camera stream checked by pre-run gate and post-run bag audit" >> "${RUNTIME_WATCHDOG_LOG}"
        fi
        if [ "${REQUIRE_GT}" = true ]; then
            watchdog_rate_check "${MOCAP_TOPIC}" "${MIN_GT_HZ}" "${RUNTIME_WATCHDOG_HZ_TIMEOUT}" 20 || failures=$((failures + 1))
        fi

        if [ "${failures}" -ne 0 ]; then
            consecutive_failure_cycles=$((consecutive_failure_cycles + 1))
            echo "WARN: ${failures} runtime watchdog check(s) failed in cycle ${cycle}; consecutive_failure_cycles=${consecutive_failure_cycles}/${RUNTIME_WATCHDOG_MAX_CONSECUTIVE_FAILURES}" | tee -a "${RUNTIME_WATCHDOG_LOG}"
            if [ "${consecutive_failure_cycles}" -ge "${RUNTIME_WATCHDOG_MAX_CONSECUTIVE_FAILURES}" ]; then
                if [ "${RUNTIME_WATCHDOG_ABORT_ON_FAILURE}" = true ]; then
                    echo "FAIL: runtime watchdog failed ${consecutive_failure_cycles} consecutive cycle(s); stopping recording." | tee -a "${RUNTIME_WATCHDOG_LOG}"
                    echo "FAIL_RUNTIME_WATCHDOG" > "${RUNTIME_WATCHDOG_STATUS_FILE}"
                    kill -INT "${ROSBAG_PID}" 2>/dev/null || true
                    return 1
                fi
                echo "WARN: runtime watchdog failed ${consecutive_failure_cycles} consecutive cycle(s); recording continues and post-run bag validation is authoritative." | tee -a "${RUNTIME_WATCHDOG_LOG}"
                echo "WARN_RUNTIME_WATCHDOG consecutive_failure_cycles=${consecutive_failure_cycles}" > "${RUNTIME_WATCHDOG_STATUS_FILE}"
            fi
            sleep "${RUNTIME_WATCHDOG_INTERVAL}"
            continue
        fi

        consecutive_failure_cycles=0
        echo "PASS watchdog cycle ${cycle}" >> "${RUNTIME_WATCHDOG_LOG}"
        sleep "${RUNTIME_WATCHDOG_INTERVAL}"
    done

    if [ -f "${RUNTIME_WATCHDOG_STATUS_FILE}" ] && \
       grep -q "^FAIL_RUNTIME_WATCHDOG" "${RUNTIME_WATCHDOG_STATUS_FILE}"; then
        return 1
    fi
    if [ -f "${RUNTIME_WATCHDOG_STATUS_FILE}" ] && \
       grep -q "^WARN_RUNTIME_WATCHDOG" "${RUNTIME_WATCHDOG_STATUS_FILE}"; then
        echo "STOPPED_WITH_RUNTIME_WARNINGS cycles=${cycle}" > "${RUNTIME_WATCHDOG_STATUS_FILE}"
    else
        echo "STOPPED_CLEANLY cycles=${cycle}" > "${RUNTIME_WATCHDOG_STATUS_FILE}"
    fi
    echo "STOPPED_CLEANLY: ros2 bag recorder no longer running" >> "${RUNTIME_WATCHDOG_LOG}"
    return 0
}

write_watchdog_stopped_cleanly_status() {
    local cycles

    cycles="$(grep -c "^PASS watchdog cycle" "${RUNTIME_WATCHDOG_LOG}" 2>/dev/null || true)"
    echo "STOPPED_CLEANLY cycles=${cycles:-0}" > "${RUNTIME_WATCHDOG_STATUS_FILE}"
}

start_runtime_watchdog() {
    if [ "${ENABLE_RUNTIME_WATCHDOG}" != "true" ]; then
        echo "DISABLED" > "${RUNTIME_WATCHDOG_STATUS_FILE}"
        return 0
    fi

    run_runtime_watchdog &
    WATCHDOG_PID=$!
    echo "Runtime watchdog started; log: ${RUNTIME_WATCHDOG_LOG}"
}

run_camera_post_enumerate_gate() {
    if [ "${RUN_REALSENSE_CAMERA_GATE}" != "true" ]; then
        return 0
    fi

    {
        echo "# post-run rs-enumerate-devices"
        echo "# session: ${SESSION_ID}"
        echo "# captured: $(date --iso-8601=seconds)"
        echo "# command: timeout 25 rs-enumerate-devices -s"
        rc=0
        if command -v rs-enumerate-devices >/dev/null 2>&1; then
            timeout 25 rs-enumerate-devices -s || rc=$?
        else
            echo "rs-enumerate-devices missing"
            rc=127
        fi
        echo "# exit: ${rc}"
    } > "${CAMERA_GATE_POST_LOG}" 2>&1

    if grep -q "Intel RealSense D455" "${CAMERA_GATE_POST_LOG}" && \
       grep -q "# exit: 0" "${CAMERA_GATE_POST_LOG}"; then
        echo "  [OK] post-run rs-enumerate-devices detected D455."
    else
        echo "  [WARN] post-run rs-enumerate-devices did not cleanly detect D455."
        echo "         log: ${CAMERA_GATE_POST_LOG}"
    fi
}

run_realsense_fault_classification() {
    if [ ! -f "${ROOT}/scripts/diagnostics/classify_realsense_fault.py" ]; then
        {
            echo "classification: INCONCLUSIVE"
            echo "reason: classifier not found at ${ROOT}/scripts/diagnostics/classify_realsense_fault.py"
        } > "${FAULT_CLASSIFICATION_FILE}"
        return 0
    fi

    python3 "${ROOT}/scripts/diagnostics/classify_realsense_fault.py" \
        --label "${SESSION_ID}" \
        --readiness-log "${CAMERA_GATE_PRE_LOG}" \
        --bringup-log "${BRINGUP_LOG}" \
        --kernel-log "${KERNEL_RUNTIME_LOG}" \
        --hardware-log "${HARDWARE_PRE_LOG}" \
        --hardware-log "${HARDWARE_POST_LOG}" \
        > "${FAULT_CLASSIFICATION_FILE}" 2>&1 || true

    echo "  [i] RealSense fault classification: ${FAULT_CLASSIFICATION_FILE}"
    sed -n '1,80p' "${FAULT_CLASSIFICATION_FILE}" || true
}

cleanup() {
    if [ "$CLEANED_UP" = true ]; then
        return
    fi
    CLEANED_UP=true
    trap - EXIT INT TERM

    if [ -n "${ROSBAG_PID}" ] && kill -0 "${ROSBAG_PID}" 2>/dev/null; then
        echo ""
        echo "Stopping ros2 bag recorder..."
        kill -INT "${ROSBAG_PID}" 2>/dev/null || true
        wait_or_kill "${ROSBAG_PID}" "ros2 bag recorder" "${ROSBAG_STOP_TIMEOUT}"
    fi

    if [ -n "${WATCHDOG_PID}" ] && kill -0 "${WATCHDOG_PID}" 2>/dev/null; then
        kill -TERM "${WATCHDOG_PID}" 2>/dev/null || true
        wait_or_kill "${WATCHDOG_PID}" "runtime watchdog" "${WATCHDOG_STOP_TIMEOUT}"
    fi

    if [ -n "${IMU_KEEPALIVE_PID}" ] && kill -0 "${IMU_KEEPALIVE_PID}" 2>/dev/null; then
        kill -TERM "${IMU_KEEPALIVE_PID}" 2>/dev/null || true
        wait_or_kill "${IMU_KEEPALIVE_PID}" "IMU recording keepalive" 5
    fi
    if [ ! -s "${RUNTIME_WATCHDOG_STATUS_FILE}" ]; then
        if [ "${RECORDING_STARTED}" = true ]; then
            write_watchdog_stopped_cleanly_status
        else
            echo "PRE_RECORDING_ABORT" > "${RUNTIME_WATCHDOG_STATUS_FILE}"
        fi
    elif ! grep -q "^FAIL_RUNTIME_WATCHDOG" "${RUNTIME_WATCHDOG_STATUS_FILE}" && \
         grep -q "^RUNNING" "${RUNTIME_WATCHDOG_STATUS_FILE}"; then
        if [ "${RECORDING_STARTED}" = true ]; then
            write_watchdog_stopped_cleanly_status
        else
            echo "PRE_RECORDING_ABORT" > "${RUNTIME_WATCHDOG_STATUS_FILE}"
        fi
    fi

    if [ -n "${BRINGUP_PID}" ] && kill -0 "${BRINGUP_PID}" 2>/dev/null; then
        echo "Stopping bringup..."
        if [ -n "${BRINGUP_PGID}" ] && kill -0 "-${BRINGUP_PGID}" 2>/dev/null; then
            kill -INT "-${BRINGUP_PGID}" 2>/dev/null || true
            wait_or_kill_group "${BRINGUP_PGID}" "${BRINGUP_PID}" "bringup" "${BRINGUP_STOP_TIMEOUT}"
        else
            kill -INT "${BRINGUP_PID}" 2>/dev/null || true
            wait_or_kill "${BRINGUP_PID}" "bringup" "${BRINGUP_STOP_TIMEOUT}"
        fi
    fi

    capture_hardware_snapshot "post-run" "${HARDWARE_POST_LOG}"
    capture_runtime_kernel_log
    run_camera_post_enumerate_gate
    run_realsense_fault_classification
    finalise_manifest
}

handle_signal() {
    cleanup
    exit 130
}

trap cleanup EXIT
trap handle_signal INT TERM

# Export env vars for any launched child tools.
export ROBOT_NAME="$ROBOT_NAME"
export SCENARIO="$SCENARIO"
export DATESTAMP="$DATESTAMP"
export MOCAP_TOPIC="$MOCAP_TOPIC"
export CMD_TOPIC="$CMD_TOPIC"
export REQUIRE_GT="$REQUIRE_GT"
export REQUIRE_IMU="$REQUIRE_IMU"
export IMU_TOPICS="$IMU_TOPICS"
export ENABLE_IMU_RECORDING_KEEPALIVE="$ENABLE_IMU_RECORDING_KEEPALIVE"
export IMU_RECORDING_KEEPALIVE_SECONDS="$IMU_RECORDING_KEEPALIVE_SECONDS"

export ENABLE_REALSENSE_SYNC="$ENABLE_REALSENSE_SYNC"
export ROSBAG2_MAX_CACHE_SIZE="$ROSBAG2_MAX_CACHE_SIZE"
export ROSBAG2_MAX_BAG_SIZE="$ROSBAG2_MAX_BAG_SIZE"
export ROSBAG2_STORAGE_ID="$ROSBAG2_STORAGE_ID"
export ROSBAG2_STORAGE_ID_EFFECTIVE="$ROSBAG2_STORAGE_ID_EFFECTIVE"
export ROSBAG2_STORAGE_CONFIG="$ROSBAG2_STORAGE_CONFIG"
export ROSBAG2_STORAGE_PRESET_PROFILE="$ROSBAG2_STORAGE_PRESET_PROFILE"
export ROSBAG2_QOS_OVERRIDES="$ROSBAG2_QOS_OVERRIDES"
export ROSBAG_STOP_TIMEOUT="$ROSBAG_STOP_TIMEOUT"
export BRINGUP_STOP_TIMEOUT="$BRINGUP_STOP_TIMEOUT"
export WATCHDOG_STOP_TIMEOUT="$WATCHDOG_STOP_TIMEOUT"
export RUNTIME_WATCHDOG_ABORT_ON_FAILURE="$RUNTIME_WATCHDOG_ABORT_ON_FAILURE"
export RUN_REALSENSE_CAMERA_GATE="$RUN_REALSENSE_CAMERA_GATE"
export REALSENSE_CAMERA_GATE_SECONDS="$REALSENSE_CAMERA_GATE_SECONDS"
export STRICT_REALSENSE_UVC_LOG="$STRICT_REALSENSE_UVC_LOG"
export RATE_EPSILON_HZ="$RATE_EPSILON_HZ"
export REALSENSE_ACTIVE_RGBD_GAP_ABORT="$REALSENSE_ACTIVE_RGBD_GAP_ABORT"
export REALSENSE_ACTIVE_IMU_GAP_ABORT="$REALSENSE_ACTIVE_IMU_GAP_ABORT"
export RGBD_STARTUP_TIMEOUT="$RGBD_STARTUP_TIMEOUT"
export IMU_STARTUP_TIMEOUT="$IMU_STARTUP_TIMEOUT"

export CAMERA_COLOR_WIDTH="$CAMERA_COLOR_WIDTH"
export CAMERA_COLOR_HEIGHT="$CAMERA_COLOR_HEIGHT"
export CAMERA_COLOR_FPS="$CAMERA_COLOR_FPS"
export CAMERA_DEPTH_WIDTH="$CAMERA_DEPTH_WIDTH"
export CAMERA_DEPTH_HEIGHT="$CAMERA_DEPTH_HEIGHT"
export CAMERA_DEPTH_FPS="$CAMERA_DEPTH_FPS"

wait_for_topic_rate() {
    topic="$1"
    timeout_s="$2"
    end=$((SECONDS + timeout_s))
    while [ "$SECONDS" -lt "$end" ]; do
        if _topic_hz_ok "$topic"; then
            echo "  [OK] $topic publishing"
            return 0
        fi
        sleep 1
    done
    echo "ERROR: timed out waiting for $topic" >&2
    return 1
}

write_sysfs_best_effort() {
    local path="$1"
    local value="$2"

    if [ -w "${path}" ]; then
        echo "${value}" > "${path}" 2>/dev/null || true
    elif sudo -n true >/dev/null 2>&1; then
        echo "${value}" | sudo -n tee "${path}" > /dev/null 2>&1 || true
    fi
}

write_sysfs_required() {
    local path="$1"
    local value="$2"

    if [ -w "${path}" ]; then
        echo "${value}" > "${path}"
        return 0
    fi
    if sudo -n true >/dev/null 2>&1; then
        echo "${value}" | sudo -n tee "${path}" > /dev/null
        return 0
    fi
    if [ -n "${SUDO_PASSWORD:-}" ]; then
        printf "%s\n" "${SUDO_PASSWORD}" | sudo -S tee "${path}" > /dev/null
        return 0
    fi
    return 1
}

sudo_best_effort() {
    if sudo -n true >/dev/null 2>&1; then
        sudo -n "$@" >/dev/null 2>&1
        return $?
    fi
    if [ -n "${SUDO_PASSWORD:-}" ]; then
        printf "%s\n" "${SUDO_PASSWORD}" | sudo -S "$@" >/dev/null 2>&1
        return $?
    fi
    return 1
}

find_d455_sysfs() {
    local p
    for p in /sys/bus/usb/devices/*/idProduct; do
        [ "$(cat "$p" 2>/dev/null)" = "0b5c" ] && dirname "$p" && return 0
    done
    return 1
}

apply_d455_power_policy() {
    local rs_sysfs="$1"

    [ -n "${rs_sysfs}" ] || return 0
    write_sysfs_best_effort "${rs_sysfs}/power/control" on
    write_sysfs_best_effort "${rs_sysfs}/power/autosuspend" -1
    write_sysfs_best_effort "${rs_sysfs}/power/autosuspend_delay_ms" -1
    echo "  D455 autosuspend disabled"
}

usb_reset_d455() {
    local rs_sysfs="$1"
    local bus dev devfile

    bus="$(cat "${rs_sysfs}/busnum" 2>/dev/null || true)"
    dev="$(cat "${rs_sysfs}/devnum" 2>/dev/null || true)"
    if [ -z "${bus}" ] || [ -z "${dev}" ]; then
        echo "  [WARN] D455 USB bus/dev unavailable; cannot send USBDEVFS_RESET"
        return 1
    fi

    devfile="$(printf "/dev/bus/usb/%03d/%03d" "${bus}" "${dev}")"
    python3 -c "
import fcntl, os, sys
USBDEVFS_RESET = 0x5514
try:
    fd = os.open('${devfile}', os.O_WRONLY)
    fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    os.close(fd)
    print('  D455 USB reset sent (${devfile})')
except Exception as e:
    print('  USB reset failed:', e, file=sys.stderr)
    sys.exit(1)
"
}

hardware_reset_d455() {
    python3 - <<'PY'
import sys
import time

try:
    import pyrealsense2 as rs
except Exception as exc:
    print(f"  [WARN] pyrealsense2 unavailable for D455 hardware_reset: {exc}", file=sys.stderr)
    raise SystemExit(1)

ctx = rs.context()
for dev in ctx.query_devices():
    product_id = dev.get_info(rs.camera_info.product_id).lower() if dev.supports(rs.camera_info.product_id) else ""
    if product_id == "0b5c":
        serial = dev.get_info(rs.camera_info.serial_number) if dev.supports(rs.camera_info.serial_number) else "unknown"
        dev.hardware_reset()
        print(f"  D455 hardware_reset sent ({serial})")
        raise SystemExit(0)

print("  [WARN] no D455 found by pyrealsense2 for hardware_reset", file=sys.stderr)
raise SystemExit(1)
PY
}

authorize_cycle_d455() {
    local rs_sysfs="$1"

    if [ ! -f "${rs_sysfs}/authorized" ]; then
        echo "  [WARN] ${rs_sysfs}/authorized missing; cannot deauthorize/reauthorize D455"
        return 1
    fi

    echo "  D455 deauthorize/reauthorize cycle (${rs_sysfs})"
    write_sysfs_required "${rs_sysfs}/authorized" 0 || return 1
    sleep 4
    write_sysfs_required "${rs_sysfs}/authorized" 1 || return 1
}

retrigger_d455_udev() {
    if ! command -v udevadm >/dev/null 2>&1; then
        return 0
    fi

    if sudo_best_effort udevadm control --reload-rules; then
        sudo_best_effort udevadm trigger --subsystem-match=video4linux || true
        sudo_best_effort udevadm trigger --subsystem-match=media || true
        sudo_best_effort udevadm trigger --subsystem-match=iio || true
        sudo_best_effort udevadm trigger --subsystem-match=hidraw || true
        sudo_best_effort udevadm settle --timeout=10 || true
        echo "  D455 udev rules retriggered"
    else
        echo "  [WARN] could not retrigger udev rules; set SUDO_PASSWORD if device permissions look stale"
    fi
}

reset_d455_before_bringup() {
    local rs_sysfs

    if [ "${D455_RESET_MODE}" = "none" ] || [ "${D455_RESET_MODE}" = "false" ]; then
        echo "Skipping D455 reset before bringup (D455_RESET_MODE=${D455_RESET_MODE})"
        return 0
    fi

    rs_sysfs="$(find_d455_sysfs || true)"
    if [ -z "${rs_sysfs}" ]; then
        echo "  [WARN] D455 not found in sysfs; camera may not work"
        return 0
    fi

    case "${D455_RESET_MODE}" in
        hardware-reset)
            echo "Resetting D455 (RealSense hardware_reset)..."
            hardware_reset_d455 || {
                echo "  [WARN] hardware_reset failed; falling back to USB deauthorize/reauthorize"
                authorize_cycle_d455 "${rs_sysfs}" || {
                    echo "  [WARN] authorize cycle failed; falling back to USBDEVFS_RESET"
                    usb_reset_d455 "${rs_sysfs}" || true
                }
            }
            sleep "${D455_RESET_REENUMERATE_WAIT}"
            retrigger_d455_udev
            ;;
        authorize-cycle)
            echo "Resetting D455 (USB deauthorize/reauthorize cycle)..."
            authorize_cycle_d455 "${rs_sysfs}" || {
                echo "  [WARN] authorize cycle failed; falling back to USBDEVFS_RESET"
                usb_reset_d455 "${rs_sysfs}" || true
            }
            sleep "${D455_RESET_REENUMERATE_WAIT}"
            retrigger_d455_udev
            ;;
        usb-reset)
            echo "Resetting D455 (USBDEVFS_RESET)..."
            usb_reset_d455 "${rs_sysfs}" || true
            sleep "${D455_RESET_REENUMERATE_WAIT}"
            retrigger_d455_udev
            ;;
        *)
            echo "ERROR: unsupported D455_RESET_MODE='${D455_RESET_MODE}' (use hardware-reset, authorize-cycle, usb-reset, or none)." >&2
            exit 1
            ;;
    esac

    rs_sysfs="$(find_d455_sysfs || true)"
    if [ -n "${rs_sysfs}" ]; then
        apply_d455_power_policy "${rs_sysfs}"
    else
        echo "  [WARN] D455 did not reappear after reset; camera may not work"
    fi
}

BRINGUP_LOG="${BAG_DIR}/${SESSION_ID}_bringup.log"

# Kill any stale bringup from a previous session before starting a new one.
# Without this, two myagv_odometry_node instances share /dev/ttyACM0 and each
# only gets half the serial packets, halving the odom/tf publish rate.
echo "Checking for stale bringup processes..."
STALE_PIDS=$(pgrep -f 'myagv_odometry_node|ydlidar_ros2_driver_node|realsense2_camera_node' 2>/dev/null | tr '\n' ' ')
if [ -n "${STALE_PIDS}" ]; then
    echo "  Found stale bringup pids: ${STALE_PIDS}; killing before starting fresh."
    pkill -INT -f 'myagv_odometry_node|ydlidar_ros2_driver_node|realsense2_camera_node' 2>/dev/null || true
    sleep 3
    pkill -TERM -f 'myagv_odometry_node|ydlidar_ros2_driver_node|realsense2_camera_node' 2>/dev/null || true
    sleep 2
    pkill -KILL -f 'myagv_odometry_node|ydlidar_ros2_driver_node|realsense2_camera_node' 2>/dev/null || true
    sleep 1
fi

reset_d455_before_bringup

KERNEL_RUNTIME_START_LINE="$(kernel_line_count)"
echo "Starting bringup; log: ${BRINGUP_LOG}"
COLOR_PROFILE="${CAMERA_COLOR_WIDTH}x${CAMERA_COLOR_HEIGHT}x${CAMERA_COLOR_FPS}"
DEPTH_PROFILE="${CAMERA_DEPTH_WIDTH}x${CAMERA_DEPTH_HEIGHT}x${CAMERA_DEPTH_FPS}"
setsid ros2 launch agv_bringup bringup.launch.py \
    agv_serial_port:="/dev/ttyACM0" \
    agv_color_profile:="${COLOR_PROFILE}" \
    agv_depth_profile:="${DEPTH_PROFILE}" \
    enable_sync:="${ENABLE_REALSENSE_SYNC}" \
    initial_reset:="false" \
    agv_cmd_vel_topic:="${CMD_TOPIC}" \
    > "${BRINGUP_LOG}" 2>&1 &
BRINGUP_PID=$!
sleep 1
BRINGUP_PGID="$(ps -o pgid= -p "${BRINGUP_PID}" 2>/dev/null | tr -d ' ')"

echo "Waiting for required sensor streams before recording..."
FAILED_TOPICS=()

# Function to check a topic without exiting early
check_topic_silent() {
    local topic="$1"
    local timeout_s="$2"
    local end=$((SECONDS + timeout_s))
    while [ "$SECONDS" -lt "$end" ]; do
        if _topic_hz_ok "$topic"; then
            echo "  [OK] $topic is live."
            return 0
        fi
        sleep 1
    done
    echo "  [!] $topic TIMEOUT (not publishing)"
    return 1
}

check_topic_silent /scan 30 || FAILED_TOPICS+=("/scan")
check_topic_silent /odom 20 || FAILED_TOPICS+=("/odom")
check_topic_silent /camera/color/image_raw "${RGBD_STARTUP_TIMEOUT}" || FAILED_TOPICS+=("/camera/color/image_raw")
check_topic_silent /camera/aligned_depth_to_color/image_raw "${RGBD_STARTUP_TIMEOUT}" || FAILED_TOPICS+=("/camera/aligned_depth_to_color/image_raw")

if [ "$REQUIRE_IMU" = true ]; then
    IMU_OK=false
    for topic in $IMU_TOPICS; do
        if check_topic_silent "$topic" "${IMU_STARTUP_TIMEOUT}"; then
            IMU_OK=true
            break
        fi
    done
    if [ "$IMU_OK" = false ]; then
        FAILED_TOPICS+=("IMU (${IMU_TOPICS})")
    fi
fi

if [ ${#FAILED_TOPICS[@]} -ne 0 ]; then
    echo ""
    echo "❌ ERROR: The following required topics are NOT publishing:"
    for ft in "${FAILED_TOPICS[@]}"; do echo "   - $ft"; done
    echo ""
    echo "Check the bringup log for errors: ${BRINGUP_LOG}"
    echo "Exiting."
    exit 1
fi

run_camera_pre_gate

echo "Sensors are live; starting bag recording."
START_EPOCH=$(date +%s)
ROS2_RECORD_TOPICS=(
    /scan
    /odom
    "${CMD_TOPIC}"
    /tf
    /tf_static
    /camera/color/image_raw
    /camera/color/camera_info
    /camera/depth/camera_info
    /camera/aligned_depth_to_color/image_raw
    /camera/aligned_depth_to_color/camera_info
    /camera/extrinsics/depth_to_color
    /camera/imu
    /camera/gyro/sample
    /camera/accel/sample
    /imu
    /diagnostics
    /tag_detections
    "${MOCAP_TOPIC}"
    /mocap
)
ROS2_STORAGE_ARGS=()
if [ -n "${ROSBAG2_STORAGE_ID_EFFECTIVE}" ]; then
    ROS2_STORAGE_ARGS+=(-s "${ROSBAG2_STORAGE_ID_EFFECTIVE}")
fi
if [ "${ROSBAG2_STORAGE_ID_EFFECTIVE}" = "sqlite3" ] && [ -n "${ROSBAG2_STORAGE_CONFIG}" ]; then
    if [ -f "${ROSBAG2_STORAGE_CONFIG}" ]; then
        ROS2_STORAGE_ARGS+=(--storage-config-file "${ROSBAG2_STORAGE_CONFIG}")
    else
        echo "  [WARN] rosbag2 storage config not found: ${ROSBAG2_STORAGE_CONFIG}; using rosbag2 defaults."
    fi
fi
if [ -n "${ROSBAG2_STORAGE_PRESET_PROFILE}" ]; then
    ROS2_STORAGE_ARGS+=(--storage-preset-profile "${ROSBAG2_STORAGE_PRESET_PROFILE}")
fi
if [ -n "${ROSBAG2_QOS_OVERRIDES}" ]; then
    if [ -f "${ROSBAG2_QOS_OVERRIDES}" ]; then
        ROS2_STORAGE_ARGS+=(--qos-profile-overrides-path "${ROSBAG2_QOS_OVERRIDES}")
    else
        echo "  [WARN] rosbag2 QoS overrides file not found: ${ROSBAG2_QOS_OVERRIDES}; using rosbag2 QoS defaults."
    fi
fi
if [ -n "${ROSBAG2_MAX_BAG_SIZE}" ] && [ "${ROSBAG2_MAX_BAG_SIZE}" != "0" ]; then
    ROS2_STORAGE_ARGS+=(--max-bag-size "${ROSBAG2_MAX_BAG_SIZE}")
fi

if [ "${REQUIRE_IMU}" = true ] && [ "${ENABLE_IMU_RECORDING_KEEPALIVE}" = true ]; then
    echo "Starting bounded IMU recording keepalive (${IMU_RECORDING_KEEPALIVE_SECONDS}s on /camera/imu)..."
    timeout "${IMU_RECORDING_KEEPALIVE_SECONDS}" \
        ros2 topic echo --qos-reliability best_effort /camera/imu \
        > /dev/null 2>&1 &
    IMU_KEEPALIVE_PID=$!
    sleep 1
fi

ros2 bag record \
    --max-cache-size "${ROSBAG2_MAX_CACHE_SIZE}" \
    "${ROS2_STORAGE_ARGS[@]}" \
    -o "${BAG_PATH}" \
    "${ROS2_RECORD_TOPICS[@]}" &
ROSBAG_PID=$!
RECORDING_STARTED=true
start_runtime_watchdog
set +e
wait "${ROSBAG_PID}"
ROSBAG_RC=$?
set -e
ROSBAG_PID=""

if [ -s "${RUNTIME_WATCHDOG_STATUS_FILE}" ] && \
   grep -q "^FAIL_RUNTIME_WATCHDOG" "${RUNTIME_WATCHDOG_STATUS_FILE}"; then
    echo "ERROR: runtime watchdog stopped this recording; do not use this bag as publishable data." >&2
    cleanup
    exit 1
fi

if [ "${ROSBAG_RC}" -ne 0 ]; then
    echo "ERROR: ros2 bag recorder exited with status ${ROSBAG_RC}." >&2
    cleanup
    exit "${ROSBAG_RC}"
fi

cleanup
