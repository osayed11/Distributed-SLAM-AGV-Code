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
#   3. Launches roslaunch agv_bringup logging.launch
#   4. On Ctrl+C, finalises the manifest with duration and bag size
#
# Run this on the robot. It is location-independent as long as this repo is
# intact, e.g. ~/slam_project/scripts/logging/start_session.sh.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# Force disable RealSense time-sync polling to prevent USB overflows/hwmon errors
export RS2_GLOBAL_TIME_ENABLED=0

# ---------------------------------------------------------------------------
# Args
# ---------------------------------------------------------------------------
ROBOT_NAME="${1:-agv_unknown}"
SCENARIO="${2:-unknown_scenario}"
DATESTAMP=$(date +%Y%m%d_%H%M%S)
MOCAP_TOPIC="${MOCAP_TOPIC:-/phasespace/rigids}"
REQUIRE_GT="${REQUIRE_GT:-false}"
REQUIRE_IMU="${REQUIRE_IMU:-false}"
ENABLE_IMU="${ENABLE_IMU:-${REQUIRE_IMU}}"
SESSION_ID="${ROBOT_NAME}_${SCENARIO}_${DATESTAMP}"
BAG_DIR="${HOME}/agv_data"
BAG_FILE="${BAG_DIR}/${SESSION_ID}.bag"
MANIFEST_FILE="${BAG_DIR}/${SESSION_ID}_manifest.yaml"
CHRONY_FILE="${BAG_DIR}/${SESSION_ID}_chrony.txt"

mkdir -p "${BAG_DIR}"

# ---------------------------------------------------------------------------
# Source ROS
# ---------------------------------------------------------------------------
source /opt/ros/noetic/setup.bash
source "${ROOT}/agv_ws/devel/setup.bash"

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
OPTIONAL_TOPICS="/imu /camera/imu /camera/accel/sample /camera/gyro/sample"
GROUND_TRUTH_TOPICS="${MOCAP_TOPIC} /mocap"
ALL_OK=true

if ! rostopic list > /dev/null 2>&1; then
    echo "  [i] ROS master not running yet; logging.launch will start bringup."
    echo "      Skipping live topic probes before launch."
else
    for topic in $REQUIRED_TOPICS; do
        if timeout 6 rostopic hz "$topic" --window 10 2>/dev/null | grep -q "average rate"; then
            echo "  [OK] $topic publishing"
        else
            # Try simpler check
            if timeout 3 rostopic echo "$topic" -n 1 > /dev/null 2>&1; then
                echo "  [OK] $topic publishing"
            else
                echo "  [!] $topic not detected - may not be running yet"
                ALL_OK=false
            fi
        fi
    done

    for topic in $OPTIONAL_TOPICS; do
        if timeout 4 rostopic hz "$topic" --window 10 2>/dev/null | grep -q "average rate"; then
            echo "  [OK] optional $topic publishing"
        else
            echo "  [i] optional $topic not detected"
        fi
    done

    GT_OK=false
    for topic in $GROUND_TRUTH_TOPICS; do
        if timeout 3 rostopic echo "$topic" -n 1 > /dev/null 2>&1; then
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
        IMU_OK=false
        for topic in /imu /camera/imu /camera/accel/sample /camera/gyro/sample; do
            if timeout 4 rostopic hz "$topic" --window 10 2>/dev/null | grep -q "average rate"; then
                IMU_OK=true
                break
            fi
        done
        if [ "$IMU_OK" = false ]; then
            echo "ERROR: REQUIRE_IMU=true but no live IMU topic is publishing."
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
bag_file: ${SESSION_ID}.bag
chrony_file: ${SESSION_ID}_chrony.txt
bag_size_mb: ~
duration_sec: ~
calibration_hash: "sha256:${CALIB_HASH}"
mocap_topic: "${MOCAP_TOPIC}"
ground_truth_required: ${REQUIRE_GT}
imu_required: ${REQUIRE_IMU}
enable_imu: ${ENABLE_IMU}
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
# Record start time for duration calculation
# ---------------------------------------------------------------------------
START_EPOCH=$(date +%s)

# ---------------------------------------------------------------------------
# Launch logging (trap Ctrl+C to finalise manifest)
# ---------------------------------------------------------------------------
finalise_manifest() {
    echo ""
    echo "=== Finalising manifest ==="
    END_EPOCH=$(date +%s)
    DURATION=$((END_EPOCH - START_EPOCH))

    if [ -f "${BAG_FILE}" ]; then
        BAG_SIZE_MB=$(du -m "${BAG_FILE}" 2>/dev/null | cut -f1)
    else
        # rosbag appends .bag automatically but also sometimes names it differently
        ACTUAL_BAG=$(ls "${BAG_DIR}/${SESSION_ID}"*.bag 2>/dev/null | head -1)
        BAG_SIZE_MB=$(du -m "${ACTUAL_BAG}" 2>/dev/null | cut -f1 || echo "~")
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
    echo "  python3 scripts/logging/validate_bag.py ${BAG_DIR}/${SESSION_ID}.bag"
}

trap finalise_manifest EXIT

# Export env vars for logging.launch
export ROBOT_NAME="$ROBOT_NAME"
export SCENARIO="$SCENARIO"
export DATESTAMP="$DATESTAMP"
export MOCAP_TOPIC="$MOCAP_TOPIC"
export REQUIRE_GT="$REQUIRE_GT"
export REQUIRE_IMU="$REQUIRE_IMU"
export ENABLE_IMU="$ENABLE_IMU"

roslaunch agv_bringup logging.launch enable_imu:="${ENABLE_IMU}"
