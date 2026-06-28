#!/usr/bin/env bash
# Deploy the diagnostic pipeline to one robot, run it, and copy evidence back.
#
# Usage:
#   SSH_PASS=ubuntu bash scripts/diagnostics/run_robot_doctor_remote.sh agv102 192.168.0.71 -- --profile preflight
#
# Arguments before `--` are handled by this wrapper:
#   <robot_id> <host>
#
# Arguments after `--` are passed directly to robot_doctor.py on the robot.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

if [ "$#" -lt 2 ]; then
    sed -n '1,14p' "$0" >&2
    exit 2
fi

ROBOT_ID="$1"
HOST="$2"
shift 2

if [ "${1:-}" = "--" ]; then
    shift
fi

REMOTE_DOCTOR_ARGS=""
for arg in "$@"; do
    printf -v quoted_arg "%q" "${arg}"
    REMOTE_DOCTOR_ARGS+=" ${quoted_arg}"
done

SSH_USER="${SSH_USER:-ubuntu}"
SSH_PASS="${SSH_PASS:-}"
SSH_PORT="${SSH_PORT:-22}"
REMOTE_ROOT="${REMOTE_ROOT:-/home/${SSH_USER}/slam_project}"
REMOTE_OUTPUT_ROOT="${REMOTE_OUTPUT_ROOT:-/home/${SSH_USER}/agv_data/diagnostics}"
LOCAL_OUTPUT_ROOT="${LOCAL_OUTPUT_ROOT:-${ROOT}/diagnostic_reports}"
RUN_REMOTE_SELFTEST="${RUN_REMOTE_SELFTEST:-true}"
SYNTH_PROFILE="${SYNTH_PROFILE:-dataset}"

COMMON_SSH_OPTS=(
    -o ConnectTimeout=10
    -o ServerAliveInterval=5
    -o ServerAliveCountMax=3
    -o NumberOfPasswordPrompts=1
    -o StrictHostKeyChecking=no
    -o UserKnownHostsFile=/dev/null
)

SSH_OPTS=(
    -p "${SSH_PORT}"
    "${COMMON_SSH_OPTS[@]}"
)

SCP_OPTS=(
    -P "${SSH_PORT}"
    "${COMMON_SSH_OPTS[@]}"
)

if [ -n "${SSH_PASS}" ]; then
    if ! command -v sshpass >/dev/null 2>&1; then
        echo "ERROR: SSH_PASS is set but sshpass is not installed on this laptop." >&2
        exit 2
    fi
    SSH_CMD=(sshpass -p "${SSH_PASS}" ssh "${SSH_OPTS[@]}")
    SCP_CMD=(sshpass -p "${SSH_PASS}" scp "${SCP_OPTS[@]}")
else
    SSH_CMD=(ssh "${SSH_OPTS[@]}")
    SCP_CMD=(scp "${SCP_OPTS[@]}")
fi

run_ssh() {
    local rc=0
    local attempt
    for attempt in 1 2 3; do
        "${SSH_CMD[@]}" "$@" && return 0
        rc=$?
        if [ "${rc}" -ne 255 ]; then
            return "${rc}"
        fi
        sleep "${attempt}"
    done
    return "${rc}"
}

run_scp() {
    local rc=0
    local attempt
    for attempt in 1 2 3; do
        "${SCP_CMD[@]}" "$@" && return 0
        rc=$?
        if [ "${rc}" -ne 255 ]; then
            return "${rc}"
        fi
        sleep "${attempt}"
    done
    return "${rc}"
}

REMOTE="${SSH_USER}@${HOST}"
LOCAL_DEST="${LOCAL_OUTPUT_ROOT}/${ROBOT_ID}_$(date +%Y%m%d_%H%M%S)"

echo "== connectivity =="
run_ssh "${REMOTE}" "hostname; hostname -I; mkdir -p '${REMOTE_ROOT}/scripts/diagnostics' '${REMOTE_ROOT}/scripts/logging' '${REMOTE_ROOT}/docs' '${REMOTE_ROOT}/configs' '${REMOTE_OUTPUT_ROOT}'"

echo ""
echo "== deploy diagnostic pipeline =="
run_scp \
    "${ROOT}/scripts/diagnostics/robot_doctor.py" \
    "${ROOT}/scripts/diagnostics/robot_doctor.sh" \
    "${ROOT}/scripts/diagnostics/dataset_ready_gate.sh" \
    "${ROOT}/scripts/diagnostics/robot_doctor_selftest.py" \
    "${ROOT}/scripts/diagnostics/fleet_doctor_summary.py" \
    "${ROOT}/scripts/diagnostics/apply_robot_doctor_fix.sh" \
    "${ROOT}/scripts/diagnostics/dataset_run_audit.py" \
    "${ROOT}/scripts/diagnostics/diagnostic_pipeline_audit.py" \
    "${ROOT}/scripts/diagnostics/validate_robot_doctor_report.py" \
    "${ROOT}/scripts/diagnostics/synthesize_robot_doctor_failure.py" \
    "${ROOT}/scripts/diagnostics/run_fleet_doctor_remote.sh" \
    "${ROOT}/scripts/diagnostics/run_robot_doctor_remote.sh" \
    "${REMOTE}:${REMOTE_ROOT}/scripts/diagnostics/"

if [ -f "${ROOT}/scripts/setup_robot_ros2.sh" ]; then
    run_scp \
        "${ROOT}/scripts/setup_robot_ros2.sh" \
        "${REMOTE}:${REMOTE_ROOT}/scripts/"
fi

# validate_bag.py belongs under scripts/logging on the robot; copy it again to
# the canonical location so bag validation works with both ROS1 and ROS2 bags.
run_scp \
    "${ROOT}/scripts/logging/validate_ros2_bag.py" \
    "${ROOT}/scripts/logging/validate_bag.py" \
    "${REMOTE}:${REMOTE_ROOT}/scripts/logging/"

if [ -f "${ROOT}/docs/ROBOT_DIAGNOSTIC_PIPELINE.md" ]; then
    run_scp \
        "${ROOT}/docs/ROBOT_DIAGNOSTIC_PIPELINE.md" \
        "${REMOTE}:${REMOTE_ROOT}/docs/"
fi

if compgen -G "${ROOT}/configs/robot_doctor*.json" >/dev/null; then
    run_scp \
        "${ROOT}"/configs/robot_doctor*.json \
        "${REMOTE}:${REMOTE_ROOT}/configs/"
fi

run_ssh "${REMOTE}" "chmod +x '${REMOTE_ROOT}/scripts/diagnostics/'*.py '${REMOTE_ROOT}/scripts/diagnostics/'*.sh '${REMOTE_ROOT}/scripts/logging/validate_'*.py '${REMOTE_ROOT}/scripts/setup_robot_ros2.sh' 2>/dev/null || true"

if [ "${RUN_REMOTE_SELFTEST}" = true ]; then
    echo ""
    echo "== remote self-test =="
    run_ssh "${REMOTE}" "cd '${REMOTE_ROOT}' && python3 scripts/diagnostics/robot_doctor_selftest.py"
fi

echo ""
echo "== remote robot_doctor =="
set +e
run_ssh "${REMOTE}" \
    "cd '${REMOTE_ROOT}' && python3 scripts/diagnostics/robot_doctor.py '${ROBOT_ID}' --output-root '${REMOTE_OUTPUT_ROOT}'${REMOTE_DOCTOR_ARGS}"
REMOTE_RC=$?
set -e

echo ""
echo "== collect evidence =="
REMOTE_REPORT_DIR="$(run_ssh "${REMOTE}" "ls -td ${REMOTE_OUTPUT_ROOT}/${ROBOT_ID}_* 2>/dev/null | head -1" | tail -1)"
mkdir -p "${LOCAL_DEST}"
if [ -z "${REMOTE_REPORT_DIR}" ]; then
    echo "WARN: no remote report directory found under ${REMOTE_OUTPUT_ROOT}; synthesizing failed report" >&2
    LOCAL_FAILED_DIR="${LOCAL_DEST}/${ROBOT_ID}_no_remote_report_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "${LOCAL_FAILED_DIR}/logs"
    {
        echo "remote=${REMOTE}"
        echo "remote_output_root=${REMOTE_OUTPUT_ROOT}"
        echo "robot_doctor_rc=${REMOTE_RC}"
        echo "reason=no remote report directory found"
    } > "${LOCAL_FAILED_DIR}/logs/remote_wrapper_failure.txt"
    python3 "${ROOT}/scripts/diagnostics/synthesize_robot_doctor_failure.py" \
        --robot-id "${ROBOT_ID}" \
        --output-dir "${LOCAL_FAILED_DIR}" \
        --remote-report "${REMOTE_OUTPUT_ROOT}/${ROBOT_ID}_*" \
        --remote-rc "${REMOTE_RC}" \
        --profile "${SYNTH_PROFILE}"
    if [ -f "${ROOT}/scripts/diagnostics/validate_robot_doctor_report.py" ]; then
        echo ""
        echo "== validate synthesized report =="
        python3 "${ROOT}/scripts/diagnostics/validate_robot_doctor_report.py" --check-evidence "${LOCAL_FAILED_DIR}/summary.json"
    fi
    if [ "${REMOTE_RC}" -eq 0 ]; then
        exit 1
    fi
    exit "${REMOTE_RC}"
fi

run_scp -r "${REMOTE}:${REMOTE_REPORT_DIR}" "${LOCAL_DEST}/"
REMOTE_REPORT_BASENAME="$(basename "${REMOTE_REPORT_DIR}")"
echo "local_report: ${LOCAL_DEST}/${REMOTE_REPORT_BASENAME}"
echo "remote_report: ${REMOTE_REPORT_DIR}"
echo "robot_doctor_rc: ${REMOTE_RC}"

LOCAL_SUMMARY="${LOCAL_DEST}/${REMOTE_REPORT_BASENAME}/summary.json"
if [ ! -f "${LOCAL_SUMMARY}" ]; then
    echo ""
    echo "== synthesize failed report =="
    mkdir -p "${LOCAL_DEST}/${REMOTE_REPORT_BASENAME}/logs"
    {
        echo "remote=${REMOTE}"
        echo "remote_output_root=${REMOTE_OUTPUT_ROOT}"
        echo "remote_report=${REMOTE_REPORT_DIR}"
        echo "robot_doctor_rc=${REMOTE_RC}"
        echo "reason=remote report directory copied but summary.json was missing"
    } > "${LOCAL_DEST}/${REMOTE_REPORT_BASENAME}/logs/remote_wrapper_failure.txt"
    python3 "${ROOT}/scripts/diagnostics/synthesize_robot_doctor_failure.py" \
        --robot-id "${ROBOT_ID}" \
        --output-dir "${LOCAL_DEST}/${REMOTE_REPORT_BASENAME}" \
        --remote-report "${REMOTE_REPORT_DIR}" \
        --remote-rc "${REMOTE_RC}" \
        --profile "${SYNTH_PROFILE}"
fi

if [ -f "${ROOT}/scripts/diagnostics/validate_robot_doctor_report.py" ] && [ -f "${LOCAL_SUMMARY}" ]; then
    echo ""
    echo "== validate copied report =="
    python3 "${ROOT}/scripts/diagnostics/validate_robot_doctor_report.py" --check-evidence "${LOCAL_SUMMARY}"
fi

exit "${REMOTE_RC}"
