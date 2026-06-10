#!/bin/bash
# start_session.sh - Start a dataset recording session with auto-generated manifest.
#
# Usage:
#   ./start_session.sh <robot_name> <scenario>
#   ./start_session.sh agv1 corridor_loop
#
# What it does:
#   1. Validates that ROS is running and all required topics are publishing
#   2. Generates a session_manifest.yaml before recording starts
#   3. Launches roslaunch agv_bringup bringup.launch
#   4. Waits for all sensor streams to stabilise before starting rosbag
#   5. On Ctrl+C, finalises the manifest with duration and bag size
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
DATESTAMP=$(date +%Y%m%d_%H%M%S)
MOCAP_TOPIC="${MOCAP_TOPIC:-/optitrack/rigid_bodies/orkar_agv1}"
REQUIRE_GT="${REQUIRE_GT:-false}"
REQUIRE_IMU="${REQUIRE_IMU:-false}"
ENABLE_REALSENSE_SYNC="${ENABLE_REALSENSE_SYNC:-true}"

CAMERA_COLOR_WIDTH="${CAMERA_COLOR_WIDTH:-640}"
CAMERA_COLOR_HEIGHT="${CAMERA_COLOR_HEIGHT:-480}"
CAMERA_COLOR_FPS="${CAMERA_COLOR_FPS:-15}"
CAMERA_DEPTH_WIDTH="${CAMERA_DEPTH_WIDTH:-640}"
CAMERA_DEPTH_HEIGHT="${CAMERA_DEPTH_HEIGHT:-480}"
CAMERA_DEPTH_FPS="${CAMERA_DEPTH_FPS:-15}"
SESSION_ID="${ROBOT_NAME}_${SCENARIO}_${DATESTAMP}"
BAG_DIR="${HOME}/agv_data"
BAG_FILE="${BAG_DIR}/${SESSION_ID}.bag"
MANIFEST_FILE="${BAG_DIR}/${SESSION_ID}_manifest.yaml"
CHRONY_FILE="${BAG_DIR}/${SESSION_ID}_chrony.txt"

mkdir -p "${BAG_DIR}"

# ---------------------------------------------------------------------------
# Source ROS
# ---------------------------------------------------------------------------
# Detect ROS version available on this machine
ROS_VERSION=$(command -v ros2 >/dev/null 2>&1 && echo 2 || echo 1)

# Source ROS2 if available (preferred for agv2_ws robots)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
fi

# Source ROS1 as fallback for Melodic robots
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
fi

# Source whichever workspace is built
if [ -f "${ROOT}/agv2_ws/install/setup.bash" ]; then
    source "${ROOT}/agv2_ws/install/setup.bash"
elif [ -f "${ROOT}/agv_ws/devel/setup.bash" ]; then
    source "${ROOT}/agv_ws/devel/setup.bash"
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

# Check required topics are publishing (best-effort, bounded timeout).
# If logging.launch is allowed to start bringup itself these checks may warn
# before sensors exist; validate_bag.py remains the authoritative post-run gate.
REQUIRED_TOPICS="/scan /odom /tf /camera/color/image_raw /camera/aligned_depth_to_color/image_raw"
OPTIONAL_TOPICS=""
GROUND_TRUTH_TOPICS="${MOCAP_TOPIC} /mocap"
ALL_OK=true

_topic_list_ok() {
    if [ "${ROS_VERSION}" = "2" ]; then
        timeout 3 ros2 topic list > /dev/null 2>&1
    else
        rostopic list > /dev/null 2>&1
    fi
}

_topic_hz_ok() {
    local topic="$1"
    if [ "${ROS_VERSION}" = "2" ]; then
        timeout 6 ros2 topic hz --window 10 "$topic" 2>/dev/null | grep -q "average rate"
    else
        timeout 6 rostopic hz "$topic" --window 10 2>/dev/null | grep -q "average rate"
    fi
}

_topic_echo_ok() {
    local topic="$1"
    if [ "${ROS_VERSION}" = "2" ]; then
        timeout 3 ros2 topic echo --once "$topic" > /dev/null 2>&1
    else
        timeout 3 rostopic echo "$topic" -n 1 > /dev/null 2>&1
    fi
}

if [ "${ROS_VERSION}" = "2" ] || ! _topic_list_ok; then
    echo "  [i] Skipping pre-flight topic probes; sensor gate runs after bringup."
else
    for topic in $REQUIRED_TOPICS; do
        if _topic_hz_ok "$topic"; then
            echo "  [OK] $topic publishing"
        else
            # Try simpler check
            if _topic_echo_ok "$topic"; then
                echo "  [OK] $topic publishing"
            else
                echo "  [!] $topic not detected - may not be running yet"
                ALL_OK=false
            fi
        fi
    done

    for topic in $OPTIONAL_TOPICS; do
        if _topic_hz_ok "$topic"; then
            echo "  [OK] optional $topic publishing"
        else
            echo "  [i] optional $topic not detected"
        fi
    done

    GT_OK=false
    for topic in $GROUND_TRUTH_TOPICS; do
        if _topic_echo_ok "$topic"; then
            echo "  [OK] ground truth topic detected: $topic"
            GT_OK=true
            break
        fi
    done
    if [ "$GT_OK" = false ]; then
        if [ "$REQUIRE_GT" = true ]; then
            echo "ERROR: no ground truth topic detected (${GROUND_TRUTH_TOPICS})"
            exit 1
        else
            echo "  [i] no ground truth topic detected yet (${GROUND_TRUTH_TOPICS})"
            echo "      Recording can proceed; use REQUIRE_GT=true when GT must be present."
        fi
    fi

    if [ "$REQUIRE_IMU" = true ]; then
        if ! _topic_hz_ok /imu; then
            echo "ERROR: REQUIRE_IMU=true but base /imu is not publishing."
            exit 1
        fi
    fi
fi

if [ "$ALL_OK" = false ]; then
    echo ""
    echo "WARNING: Some topics not detected. Starting logging anyway."
    echo "Run validate_bag.py after recording to check data quality."
    echo ""
fi

# ---------------------------------------------------------------------------
# Write initial manifest
# ---------------------------------------------------------------------------
ROS_DISTRO_VAL=$(echo "${ROS_DISTRO:-noetic}")
CALIB_DIR="${ROOT}/agv_ws/src/agv_bringup/calibration"
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
chrony_file: ${SESSION_ID}_chrony.txt
bag_size_mb: ~
duration_sec: ~
calibration_hash: "sha256:${CALIB_HASH}"
mocap_topic: "${MOCAP_TOPIC}"
ground_truth_required: ${REQUIRE_GT}
imu_required: ${REQUIRE_IMU}
camera_imu: disabled
enable_realsense_sync: ${ENABLE_REALSENSE_SYNC}

camera_profile:
  color_width: ${CAMERA_COLOR_WIDTH}
  color_height: ${CAMERA_COLOR_HEIGHT}
  color_fps: ${CAMERA_COLOR_FPS}
  depth_width: ${CAMERA_DEPTH_WIDTH}
  depth_height: ${CAMERA_DEPTH_HEIGHT}
  depth_fps: ${CAMERA_DEPTH_FPS}
notes: ""
usb_mode_note: "D455 observed on USB 3.2; RGB-D stable. D455 IMU disabled by default because video+motion publishes no IMU messages on current wrapper/device stack."
EOF

echo ""
echo "=== Session: ${SESSION_ID} ==="
echo "Bag:      ${BAG_FILE}"
echo "Manifest: ${MANIFEST_FILE}"
echo ""
echo "Press Ctrl+C to stop recording."
echo ""

# ---------------------------------------------------------------------------
# Launch bringup, wait for sensors, then record.
# ---------------------------------------------------------------------------
START_EPOCH=$(date +%s)
BRINGUP_PID=""

ROSBAG_PID=""
CLEANED_UP=false

finalise_manifest() {
    echo ""
    echo "=== Finalising manifest ==="
    END_EPOCH=$(date +%s)
    DURATION=$((END_EPOCH - START_EPOCH))

    if [ "${ROS_VERSION}" = "2" ]; then
        # ROS2 bag is a directory
        BAG_PATH="${BAG_DIR}/${SESSION_ID}"
        if [ -d "${BAG_PATH}" ]; then
            BAG_SIZE_MB=$(du -sm "${BAG_PATH}" 2>/dev/null | cut -f1)
        else
            BAG_SIZE_MB="~"
        fi
    else
        # ROS1 bag is a .bag file
        if [ -f "${BAG_FILE}" ]; then
            BAG_SIZE_MB=$(du -m "${BAG_FILE}" 2>/dev/null | cut -f1)
        else
            # rosbag appends .bag automatically but also sometimes names it differently
            ACTUAL_BAG=$(ls "${BAG_DIR}/${SESSION_ID}"*.bag 2>/dev/null | head -1)
            BAG_SIZE_MB=$(du -m "${ACTUAL_BAG}" 2>/dev/null | cut -f1 || echo "~")
        fi
    fi

    # Update manifest with final values
    sed -i "s/time_end: ~/time_end: $(date +%H:%M:%S)/" "${MANIFEST_FILE}"
    sed -i "s/bag_size_mb: ~/bag_size_mb: ${BAG_SIZE_MB:-unknown}/" "${MANIFEST_FILE}"
    sed -i "s/duration_sec: ~/duration_sec: ${DURATION}/" "${MANIFEST_FILE}"

    echo "Duration: ${DURATION}s"
    echo "Bag size: ${BAG_SIZE_MB:-unknown} MB"
    echo "Manifest written: ${MANIFEST_FILE}"
    echo ""
    echo "Run quality check:"
    if [ "${ROS_VERSION}" = "2" ]; then
        echo "  python3 scripts/logging/validate_bag.py ${BAG_DIR}/${SESSION_ID}"
    else
        echo "  python3 scripts/logging/validate_bag.py ${BAG_DIR}/${SESSION_ID}.bag"
    fi
}

cleanup() {
    if [ "$CLEANED_UP" = true ]; then
        return
    fi
    CLEANED_UP=true
    trap - EXIT INT TERM

    if [ -n "${ROSBAG_PID}" ] && kill -0 "${ROSBAG_PID}" 2>/dev/null; then
        echo ""
        echo "Stopping rosbag..."
        kill -INT "${ROSBAG_PID}" 2>/dev/null || true
        wait "${ROSBAG_PID}" 2>/dev/null || true
    fi

    if [ -n "${BRINGUP_PID}" ] && kill -0 "${BRINGUP_PID}" 2>/dev/null; then
        echo "Stopping bringup..."
        kill -INT "${BRINGUP_PID}" 2>/dev/null || true
        wait "${BRINGUP_PID}" 2>/dev/null || true
    fi

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
export REQUIRE_GT="$REQUIRE_GT"
export REQUIRE_IMU="$REQUIRE_IMU"

export ENABLE_REALSENSE_SYNC="$ENABLE_REALSENSE_SYNC"

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

BRINGUP_LOG="${BAG_DIR}/${SESSION_ID}_bringup.log"

# Kill any stale bringup from a previous session before starting a new one.
# Without this, two myagv_odometry_node instances share /dev/ttyACM0 and each
# only gets half the serial packets, halving the odom/tf publish rate.
echo "Checking for stale bringup processes..."
STALE_PIDS=$(pgrep -f 'myagv_odometry_node|ydlidar_ros2_driver_node|realsense2_camera_node' 2>/dev/null | tr '\n' ' ')
if [ -n "${STALE_PIDS}" ]; then
    echo "  Found stale bringup pids: ${STALE_PIDS}— killing before starting fresh."
    kill ${STALE_PIDS} 2>/dev/null || true
    sleep 3
fi

echo "Starting bringup; log: ${BRINGUP_LOG}"
if [ "${ROS_VERSION}" = "2" ]; then
    COLOR_PROFILE="${CAMERA_COLOR_WIDTH}x${CAMERA_COLOR_HEIGHT}x${CAMERA_COLOR_FPS}"
    DEPTH_PROFILE="${CAMERA_DEPTH_WIDTH}x${CAMERA_DEPTH_HEIGHT}x${CAMERA_DEPTH_FPS}"
    ros2 launch agv_bringup bringup.launch.py \
        serial_port:="/dev/ttyACM0" \
        color_profile:="${COLOR_PROFILE}" \
        depth_profile:="${DEPTH_PROFILE}" \
        enable_sync:="${ENABLE_REALSENSE_SYNC}" \
        > "${BRINGUP_LOG}" 2>&1 &
else
    roslaunch agv_bringup bringup.launch \
        enable_realsense_sync:="${ENABLE_REALSENSE_SYNC}" \
        color_width:="${CAMERA_COLOR_WIDTH}" \
        color_height:="${CAMERA_COLOR_HEIGHT}" \
        color_fps:="${CAMERA_COLOR_FPS}" \
        depth_width:="${CAMERA_DEPTH_WIDTH}" \
        depth_height:="${CAMERA_DEPTH_HEIGHT}" \
        depth_fps:="${CAMERA_DEPTH_FPS}" \
        > "${BRINGUP_LOG}" 2>&1 &
fi
BRINGUP_PID=$!

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
check_topic_silent /camera/color/image_raw 45 || FAILED_TOPICS+=("/camera/color/image_raw")
check_topic_silent /camera/aligned_depth_to_color/image_raw 25 || FAILED_TOPICS+=("/camera/aligned_depth_to_color/image_raw")

if [ "$REQUIRE_IMU" = true ]; then
    check_topic_silent /imu 15 || FAILED_TOPICS+=("/imu")
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



echo "Sensors are live; starting bag recording."
START_EPOCH=$(date +%s)
if [ "${ROS_VERSION}" = "2" ]; then
    # ROS2: ros2 bag record writes to a directory; -o specifies the directory name
    ros2 bag record \
        -o "${BAG_DIR}/${SESSION_ID}" \
        /scan \
        /odom \
        /cmd_vel \
        /tf \
        /tf_static \
        /camera/color/image_raw \
        /camera/color/camera_info \
        /camera/depth/camera_info \
        /camera/aligned_depth_to_color/image_raw \
        /camera/aligned_depth_to_color/camera_info \
        /camera/extrinsics/depth_to_color \
        /camera/imu \
        /diagnostics \
        /tag_detections \
        "${MOCAP_TOPIC}" \
        /mocap &
else
    # ROS1 fallback for Melodic/Noetic robots
    rosbag record --buffsize=2048 --lz4 -O "${BAG_FILE}" \
        /scan \
        /odom \
        /cmd_vel \
        /tf \
        /tf_static \
        /camera/color/image_raw \
        /camera/color/camera_info \
        /camera/depth/camera_info \
        /camera/aligned_depth_to_color/image_raw \
        /camera/aligned_depth_to_color/camera_info \
        /camera/extrinsics/depth_to_color \
        /camera/imu \
        /diagnostics \
        /tag_detections \
        "${MOCAP_TOPIC}" \
        /mocap &
fi
ROSBAG_PID=$!
wait "${ROSBAG_PID}"
ROSBAG_PID=""
