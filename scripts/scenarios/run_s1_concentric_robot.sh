#!/usr/bin/env bash
# Run one robot in Scenario 1: concentric circles with 30 s staggered starts.
#
# Usage:
#   bash scripts/scenarios/run_s1_concentric_robot.sh <robot_name> <robot_index> [robot_count]
#
# Example:
#   S1_BASE_START_EPOCH=1778515200 bash scripts/scenarios/run_s1_concentric_robot.sh agv2 2 5
#
# robot_index is 1-based. Robot 1 uses radius 0.50 m and starts at T0.
# Robot 2 uses radius 0.75 m and starts at T0 + 30 s, etc.

set -euo pipefail

usage() {
    echo "Usage: $0 <robot_name> <robot_index:1-5> [robot_count]"
    echo ""
    echo "Environment overrides:"
    echo "  S1_BASE_START_EPOCH     Common T0 for all robots. Default: now + S1_ARM_LEAD_SEC."
    echo "  S1_ARM_LEAD_SEC         Lead time when T0 is auto-generated. Default: 180."
    echo "  S1_DURATION             Motion duration after each robot starts. Default: 600."
    echo "  S1_LINEAR               Forward speed. Default: 0.16."
    echo "  S1_RECORD               true/false, start rosbag recording. Default: true."
    echo "  S1_RECORD_WAIT_SEC      Time to wait for rosbag to begin. Default: 180."
    echo "  S1_MIN_START_MARGIN_SEC Required time between rosbag live and motion. Default: 10."
    echo "  REQUIRE_GT              Passed to start_session.sh. Default: false."
    echo "  REQUIRE_IMU             Passed to start_session.sh. Default: true."
    echo "  ENABLE_APRILTAG         Passed to start_session.sh. Default: false."
    echo "  ENABLE_ARUCO            Passed to start_session.sh. Default: false."
}

if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
    usage >&2
    exit 2
fi

ROBOT_NAME="$1"
ROBOT_INDEX="$2"
ROBOT_COUNT="${3:-5}"

if ! [[ "${ROBOT_INDEX}" =~ ^[1-5]$ ]]; then
    echo "ERROR: robot_index must be 1..5 for S1." >&2
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${ROOT}"

source_ros() {
    if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
    elif [ -f /opt/ros/noetic/setup.bash ]; then
        source /opt/ros/noetic/setup.bash
    elif [ -f /opt/ros/melodic/setup.bash ]; then
        source /opt/ros/melodic/setup.bash
    else
        echo "ERROR: no supported ROS setup found under /opt/ros" >&2
        exit 1
    fi

    if [ -f "${ROOT}/myagv_ros/devel/setup.bash" ]; then
        source "${ROOT}/myagv_ros/devel/setup.bash"
    fi
    source "${ROOT}/agv_ws/devel/setup.bash"
}

RADII=(0.50 0.75 1.00 1.25 1.50)
DELAYS=(0 30 60 90 120)
IDX=$((ROBOT_INDEX - 1))

RADIUS="${S1_RADIUS:-${RADII[$IDX]}}"
START_DELAY="${S1_START_OFFSET:-${DELAYS[$IDX]}}"
DURATION="${S1_DURATION:-600}"
LINEAR="${S1_LINEAR:-0.16}"
RECORD="${S1_RECORD:-true}"
ARM_LEAD_SEC="${S1_ARM_LEAD_SEC:-180}"
RECORD_WAIT_SEC="${S1_RECORD_WAIT_SEC:-180}"
MIN_START_MARGIN_SEC="${S1_MIN_START_MARGIN_SEC:-10}"
POST_ROLL_SEC="${S1_POST_ROLL_SEC:-5}"
SCENARIO="${S1_SCENARIO:-s1_concentric_${ROBOT_COUNT}robot_r${ROBOT_INDEX}}"

if [ -n "${S1_BASE_START_EPOCH:-}" ]; then
    BASE_START_EPOCH="${S1_BASE_START_EPOCH}"
else
    BASE_START_EPOCH="$(($(date +%s) + ARM_LEAD_SEC))"
fi

START_EPOCH="$((BASE_START_EPOCH + START_DELAY))"
SESSION_PID=""

cleanup() {
    if [ -n "${SESSION_PID}" ] && kill -0 "${SESSION_PID}" 2>/dev/null; then
        echo "Stopping S1 recording session..."
        kill -INT "${SESSION_PID}" 2>/dev/null || true
        wait "${SESSION_PID}" 2>/dev/null || true
    fi
}
cleanup_and_exit() {
    cleanup
    exit 130
}
trap cleanup EXIT
trap cleanup_and_exit INT TERM

echo "=== S1 robot plan ==="
echo "robot_name:        ${ROBOT_NAME}"
echo "robot_index:       ${ROBOT_INDEX}/${ROBOT_COUNT}"
echo "radius_m:          ${RADIUS}"
echo "base_start_epoch:  ${BASE_START_EPOCH}"
echo "start_delay_sec:   ${START_DELAY}"
echo "scheduled_epoch:   ${START_EPOCH}"
echo "duration_sec:      ${DURATION}"
echo "record:            ${RECORD}"
echo "scenario:          ${SCENARIO}"
echo ""

source_ros

if [ "${RECORD}" = "true" ]; then
    export REQUIRE_GT="${REQUIRE_GT:-false}"
    export REQUIRE_IMU="${REQUIRE_IMU:-true}"
    export ENABLE_APRILTAG="${ENABLE_APRILTAG:-false}"
    export ENABLE_ARUCO="${ENABLE_ARUCO:-false}"

    echo "Starting recording via start_session.sh..."
    bash scripts/logging/start_session.sh "${ROBOT_NAME}" "${SCENARIO}" &
    SESSION_PID=$!

    echo "Waiting for rosbag process before arming motion..."
    DEADLINE=$((SECONDS + RECORD_WAIT_SEC))
    while [ "${SECONDS}" -lt "${DEADLINE}" ]; do
        if pgrep -f "rosbag record.*${ROBOT_NAME}_${SCENARIO}_" >/dev/null 2>&1; then
            echo "rosbag is live."
            break
        fi
        if ! kill -0 "${SESSION_PID}" 2>/dev/null; then
            wait "${SESSION_PID}" || true
            echo "ERROR: recording session exited before rosbag started." >&2
            exit 1
        fi
        sleep 2
    done

    if ! pgrep -f "rosbag record.*${ROBOT_NAME}_${SCENARIO}_" >/dev/null 2>&1; then
        echo "ERROR: timed out waiting for rosbag to start." >&2
        exit 1
    fi
fi

SECONDS_TO_START=$((START_EPOCH - $(date +%s)))
if [ "${SECONDS_TO_START}" -lt "${MIN_START_MARGIN_SEC}" ]; then
    echo "ERROR: scheduled start is only ${SECONDS_TO_START}s away." >&2
    echo "Use a later S1_BASE_START_EPOCH so every robot records before motion starts." >&2
    exit 1
fi

python scripts/logging/drive_circle.py \
    --radius "${RADIUS}" \
    --linear "${LINEAR}" \
    --duration "${DURATION}" \
    --start-at-epoch "${BASE_START_EPOCH}" \
    --start-delay "${START_DELAY}" \
    --no-prompt \
    --verbose

if [ "${RECORD}" = "true" ]; then
    sleep "${POST_ROLL_SEC}"
fi

cleanup
trap - EXIT INT TERM
