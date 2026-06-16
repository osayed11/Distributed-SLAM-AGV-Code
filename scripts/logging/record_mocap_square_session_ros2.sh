#!/bin/bash
# Run a full ROS2 dataset session and drive a mocap-feedback square.
#
# Example:
#   ROS_DOMAIN_ID=78 bash scripts/logging/record_mocap_square_session_ros2.sh agv1 square_1m_full
#   ROS_DOMAIN_ID=77 bash scripts/logging/record_mocap_square_session_ros2.sh agv2 square_1m_full

set -euo pipefail

ROBOT_NAME="${1:-agv1}"
SCENARIO="${2:-square_1m_full}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

case "${ROBOT_NAME}" in
    agv1) DEFAULT_DOMAIN=78 ;;
    agv2) DEFAULT_DOMAIN=77 ;;
    *) DEFAULT_DOMAIN="${ROS_DOMAIN_ID:-0}" ;;
esac

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${DEFAULT_DOMAIN}}"
export MOCAP_TOPIC="${MOCAP_TOPIC:-/optitrack/rigid_bodies/orkar_${ROBOT_NAME}}"
export CMD_TOPIC="${CMD_TOPIC:-/${ROBOT_NAME}/cmd_vel}"
export REQUIRE_GT="${REQUIRE_GT:-true}"
export REQUIRE_IMU="${REQUIRE_IMU:-true}"

SIDE_LENGTH="${SIDE_LENGTH:-1.0}"
LINEAR="${LINEAR:-0.18}"
MIN_LINEAR="${MIN_LINEAR:-0.12}"
LINE_YAW_OFFSET_DEG="${LINE_YAW_OFFSET_DEG:-0}"
MAX_LATERAL_ERROR="${MAX_LATERAL_ERROR:-0.30}"
LEG_TIMEOUT="${LEG_TIMEOUT:-25}"
TURN_TIMEOUT="${TURN_TIMEOUT:-18}"
POSE_TIMEOUT="${POSE_TIMEOUT:-0.60}"
TURN_SIGN="${TURN_SIGN:-1}"
SESSION_READY_TIMEOUT="${SESSION_READY_TIMEOUT:-150}"
BASE_SERVICE="${BASE_SERVICE:-agv-base-d${ROS_DOMAIN_ID}.service}"
RESTART_BASE_SERVICE="${RESTART_BASE_SERVICE:-true}"

cd "${ROOT}"

source_ros_setup() {
    set +u
    # ROS setup scripts are not safe under bash nounset.
    source "$1"
    set -u
}

source_ros_setup /opt/ros/humble/setup.bash
if [ -f "${ROOT}/agv2_ws/install/setup.bash" ]; then
    source_ros_setup "${ROOT}/agv2_ws/install/setup.bash"
elif [ -f "${ROOT}/install/setup.bash" ]; then
    source_ros_setup "${ROOT}/install/setup.bash"
fi

SESSION_LOG="/tmp/${ROBOT_NAME}_${SCENARIO}_session_$(date +%Y%m%d_%H%M%S).log"
SESSION_PID=""

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    if [ -n "${SESSION_PID}" ] && kill -0 "${SESSION_PID}" 2>/dev/null; then
        kill -INT "${SESSION_PID}" 2>/dev/null || true
        wait "${SESSION_PID}" 2>/dev/null || true
    fi
    if [ "${RESTART_BASE_SERVICE}" = "true" ]; then
        systemctl --user start "${BASE_SERVICE}" >/dev/null 2>&1 || true
    fi
    exit "${status}"
}
trap cleanup EXIT INT TERM

echo "Robot:       ${ROBOT_NAME}"
echo "Scenario:    ${SCENARIO}"
echo "Domain:      ${ROS_DOMAIN_ID}"
echo "Mocap topic: ${MOCAP_TOPIC}"
echo "Cmd topic:   ${CMD_TOPIC}"
echo "Session log: ${SESSION_LOG}"

if systemctl --user list-unit-files "${BASE_SERVICE}" >/dev/null 2>&1; then
    echo "Stopping user base service during managed session: ${BASE_SERVICE}"
    systemctl --user stop "${BASE_SERVICE}" >/dev/null 2>&1 || true
fi

echo "Checking mocap topic..."
timeout 8 ros2 topic echo --once "${MOCAP_TOPIC}" >/dev/null

echo "Starting full sensor logging pipeline..."
bash scripts/logging/start_session.sh "${ROBOT_NAME}" "${SCENARIO}" > "${SESSION_LOG}" 2>&1 &
SESSION_PID=$!

deadline=$((SECONDS + SESSION_READY_TIMEOUT))
while [ "${SECONDS}" -lt "${deadline}" ]; do
    if ! kill -0 "${SESSION_PID}" 2>/dev/null; then
        echo "ERROR: session logger exited before recording started."
        tail -80 "${SESSION_LOG}" || true
        exit 1
    fi
    if grep -q "Sensors are live; starting bag recording." "${SESSION_LOG}"; then
        break
    fi
    sleep 2
done

if ! grep -q "Sensors are live; starting bag recording." "${SESSION_LOG}"; then
    echo "ERROR: timed out waiting for sensor gate."
    tail -100 "${SESSION_LOG}" || true
    exit 1
fi

sleep 5
echo "Recording is live; driving mocap square."
python3 scripts/logging/drive_mocap_square_ros2.py \
    --pose-topic "${MOCAP_TOPIC}" \
    --cmd-topic "${CMD_TOPIC}" \
    --side-length "${SIDE_LENGTH}" \
    --legs 4 \
    --linear "${LINEAR}" \
    --min-linear "${MIN_LINEAR}" \
    --line-yaw-offset-deg "${LINE_YAW_OFFSET_DEG}" \
    --max-lateral-error "${MAX_LATERAL_ERROR}" \
    --leg-timeout "${LEG_TIMEOUT}" \
    --turn-timeout "${TURN_TIMEOUT}" \
    --pose-timeout "${POSE_TIMEOUT}" \
    --turn-sign "${TURN_SIGN}" \
    --yes \
    --verbose

echo "Square complete; stopping full session."
kill -INT "${SESSION_PID}" 2>/dev/null || true
wait "${SESSION_PID}" 2>/dev/null || true
SESSION_PID=""

BAG_PATH="$(find "${HOME}/agv_data" -maxdepth 1 -type d -name "${ROBOT_NAME}_${SCENARIO}_*" | sort | tail -1)"
if [ -z "${BAG_PATH}" ]; then
    echo "ERROR: could not find output bag directory."
    tail -80 "${SESSION_LOG}" || true
    exit 1
fi

echo "Bag: ${BAG_PATH}"
ros2 bag info "${BAG_PATH}"
echo ""
echo "Fast audit:"
python3 scripts/logging/audit_bag_fast.py "${BAG_PATH}" || true
echo ""
echo "Full validation:"
python3 scripts/logging/validate_bag.py "${BAG_PATH}" --require-gt --require-imu || true
echo ""
echo "Session log: ${SESSION_LOG}"
