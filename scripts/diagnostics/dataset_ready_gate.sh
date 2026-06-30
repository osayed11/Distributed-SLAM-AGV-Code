#!/usr/bin/env bash
# Read-only pre-run dataset readiness gate for daily robot operation.
#
# This is the normal "can I start recording now?" command. It must not repair,
# reset USB devices, install packages, or mutate the robot. It runs the strict
# configured robot_doctor gate and prints a deterministic READY_TO_RECORD result.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

usage() {
    sed -n '1,6p' "$0"
    cat <<'EOF'

Usage:
  bash scripts/diagnostics/dataset_ready_gate.sh agv110
  bash scripts/diagnostics/dataset_ready_gate.sh agv110 --mocap-topic /optitrack/rigid_bodies/orkar_agv110 --cmd-topic /agv110/cmd_vel

Options:
  --config PATH              Gate config. Default: configs/robot_doctor_dataset_gate.json
  --output-root PATH         Evidence root. Default: ~/agv_data/diagnostics
  --mocap-topic TOPIC        Expected live MoCap rigid-body topic.
  --cmd-topic TOPIC          Expected command topic for this robot.
  --odom-mocap-sanity-json PATH
                             JSON evidence from the 1 m odom-vs-MoCap sanity check.
  --expected-d455-serial SN  Expected assigned D455 serial for this robot.
  --identity-file PATH       Read EXPECTED_D455_SERIAL from this file.
                             Default: ~/agv_data/<robot>_identity.env when present.
  --no-identity-file         Do not read the default identity file.
  --bringup                  Launch standard ROS2 bringup only for this gate
                             using initial_reset:=false and the configured cmd topic.
  --bringup-cmd CMD          Launch this bringup command only for this gate.
  --bringup-wait SECONDS     Max wait for required bringup topics. Default: 90.
  --confirm-mechanical       Operator confirms mounts/chassis/slip checklist.
  --confirm-mocap            Operator confirms rigid body and marker visibility.
  --confirm-anchors          Operator confirms anchors/obstacles were surveyed.
  --strict-ops               Make operator confirmations hard requirements.
  --allow-review             Exit 0 for non-blocking review state.
  --no-report-validation     Skip summary.json schema/evidence validation.

Environment:
  ROBOT_ID                   Used if the robot id argument is omitted.

EOF
}

ROBOT_ID="${1:-${ROBOT_ID:-}}"
if [ -n "${ROBOT_ID}" ] && [[ "${ROBOT_ID}" != --* ]]; then
    shift || true
else
    ROBOT_ID="${ROBOT_ID:-}"
fi

CONFIG="configs/robot_doctor_dataset_gate.json"
OUTPUT_ROOT="${HOME}/agv_data/diagnostics"
ALLOW_REVIEW=false
VALIDATE_REPORT=true
IDENTITY_FILE=""
USE_IDENTITY_FILE=true
EXPECTED_D455_SERIAL_ARG=""
CMD_TOPIC_ARG=""
BRINGUP_CMD=""
BRINGUP_REQUESTED=false
BRINGUP_WAIT="${AGV_READY_BRINGUP_WAIT:-90}"
DOCTOR_ARGS=()

while [ "$#" -gt 0 ]; do
    case "$1" in
        --config)
            shift
            CONFIG="${1:-}"
            [ -n "${CONFIG}" ] || { echo "ERROR: --config requires a value" >&2; exit 2; }
            ;;
        --output-root)
            shift
            OUTPUT_ROOT="${1:-}"
            [ -n "${OUTPUT_ROOT}" ] || { echo "ERROR: --output-root requires a value" >&2; exit 2; }
            ;;
        --mocap-topic)
            opt="$1"
            shift
            value="${1:-}"
            [ -n "${value}" ] || { echo "ERROR: ${opt} requires a value" >&2; exit 2; }
            DOCTOR_ARGS+=("${opt}" "${value}")
            ;;
        --cmd-topic)
            shift
            CMD_TOPIC_ARG="${1:-}"
            [ -n "${CMD_TOPIC_ARG}" ] || { echo "ERROR: --cmd-topic requires a value" >&2; exit 2; }
            DOCTOR_ARGS+=("--cmd-topic" "${CMD_TOPIC_ARG}")
            ;;
        --odom-mocap-sanity-json)
            opt="$1"
            shift
            value="${1:-}"
            [ -n "${value}" ] || { echo "ERROR: ${opt} requires a value" >&2; exit 2; }
            DOCTOR_ARGS+=("${opt}" "${value}")
            ;;
        --expected-d455-serial)
            shift
            EXPECTED_D455_SERIAL_ARG="${1:-}"
            [ -n "${EXPECTED_D455_SERIAL_ARG}" ] || { echo "ERROR: --expected-d455-serial requires a value" >&2; exit 2; }
            ;;
        --identity-file)
            shift
            IDENTITY_FILE="${1:-}"
            [ -n "${IDENTITY_FILE}" ] || { echo "ERROR: --identity-file requires a value" >&2; exit 2; }
            ;;
        --no-identity-file)
            USE_IDENTITY_FILE=false
            ;;
        --bringup)
            BRINGUP_REQUESTED=true
            ;;
        --bringup-cmd)
            shift
            BRINGUP_CMD="${1:-}"
            [ -n "${BRINGUP_CMD}" ] || { echo "ERROR: --bringup-cmd requires a value" >&2; exit 2; }
            ;;
        --bringup-wait)
            shift
            BRINGUP_WAIT="${1:-}"
            [ -n "${BRINGUP_WAIT}" ] || { echo "ERROR: --bringup-wait requires a value" >&2; exit 2; }
            ;;
        --confirm-mechanical|--confirm-mocap|--confirm-anchors|--strict-ops)
            DOCTOR_ARGS+=("$1")
            ;;
        --allow-review)
            ALLOW_REVIEW=true
            ;;
        --no-report-validation)
            VALIDATE_REPORT=false
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

if [ -z "${ROBOT_ID}" ]; then
    echo "ERROR: robot id is required." >&2
    usage >&2
    exit 2
fi

if [ ! -f "${ROOT}/${CONFIG}" ] && [ ! -f "${CONFIG}" ]; then
    echo "ERROR: gate config not found: ${CONFIG}" >&2
    exit 2
fi

if [ -z "${IDENTITY_FILE}" ]; then
    IDENTITY_FILE="${HOME}/agv_data/${ROBOT_ID}_identity.env"
fi
EXPECTED_D455_SERIAL="${EXPECTED_D455_SERIAL_ARG:-${EXPECTED_D455_SERIAL:-}}"
if [ -z "${EXPECTED_D455_SERIAL}" ] && [ "${USE_IDENTITY_FILE}" = true ] && [ -f "${IDENTITY_FILE}" ]; then
    EXPECTED_D455_SERIAL="$(
        awk -F= '
            $1 == "EXPECTED_D455_SERIAL" {
                gsub(/^[ \t"'\''"]+|[ \t"'\''"]+$/, "", $2)
                print $2
                exit
            }
        ' "${IDENTITY_FILE}"
    )"
fi
if [ -n "${EXPECTED_D455_SERIAL}" ]; then
    DOCTOR_ARGS+=("--expected-d455-serial" "${EXPECTED_D455_SERIAL}")
fi
if [ "${BRINGUP_REQUESTED}" = true ] && [ -z "${BRINGUP_CMD}" ]; then
    BRINGUP_CMD="${AGV_READY_BRINGUP_CMD:-}"
    if [ -z "${BRINGUP_CMD}" ]; then
        bringup_cmd_topic="${CMD_TOPIC_ARG:-/${ROBOT_ID}/cmd_vel}"
        BRINGUP_CMD="ros2 launch agv_bringup bringup.launch.py agv_color_profile:=640x480x15 agv_depth_profile:=640x480x15 initial_reset:=false agv_cmd_vel_topic:=${bringup_cmd_topic}"
    fi
fi
if [ -n "${BRINGUP_CMD}" ]; then
    DOCTOR_ARGS+=("--bringup-cmd" "${BRINGUP_CMD}" "--bringup-wait" "${BRINGUP_WAIT}")
fi

mkdir -p "${OUTPUT_ROOT}"
before_reports="$(mktemp)"
after_reports="$(mktemp)"
console_log_tmp="$(mktemp)"
cleanup_tmp() {
    rm -f "${before_reports}" "${after_reports}" "${console_log_tmp}"
}
trap cleanup_tmp EXIT
find "${OUTPUT_ROOT}" -maxdepth 1 -type d -name "${ROBOT_ID}_*" -print 2>/dev/null | sort > "${before_reports}"

echo "========================================================================"
echo "DATASET READINESS GATE"
echo "robot: ${ROBOT_ID}"
echo "config: ${CONFIG}"
echo "output_root: ${OUTPUT_ROOT}"
echo "mode: read-only; no fixes, no USB resets, no package installs"
echo "meaning: READY_TO_RECORD only; run post-run bag audit after recording"
if [ -n "${EXPECTED_D455_SERIAL}" ]; then
    echo "expected_d455_serial: ${EXPECTED_D455_SERIAL}"
else
    echo "expected_d455_serial: not configured"
fi
if [ -n "${BRINGUP_CMD}" ]; then
    echo "bringup_cmd: ${BRINGUP_CMD}"
    echo "bringup_wait: ${BRINGUP_WAIT}"
else
    echo "bringup_cmd: not launched by wrapper"
fi
echo "========================================================================"

if [ -n "${BRINGUP_CMD}" ]; then
    existing_bringup="$(
        pgrep -af 'ros2 launch agv_bringup|realsense2_camera_node|ydlidar_ros2_driver_node|myagv_odometry' 2>/dev/null | \
            grep -Ev 'dataset_ready_gate|robot_doctor.py|grep' || true
    )"
    if [ -n "${existing_bringup}" ]; then
        stale_log="${OUTPUT_ROOT}/${ROBOT_ID}_existing_bringup_$(date +%Y%m%d_%H%M%S).log"
        printf "%s\n" "${existing_bringup}" > "${stale_log}"
        echo "READY_TO_RECORD: false"
        echo "POST_RUN_DATASET_READY: false"
        echo "STATE: blocked"
        echo "FAILED_STAGE: 2.2 Drivers / launch config"
        echo "CAUSE: --bringup was requested but sensor bringup processes are already running"
        echo "EVIDENCE:"
        echo "  - ${stale_log}"
        echo "NEXT_ACTION: stop the existing bringup/session first, or rerun without --bringup if you intentionally want to validate the existing ROS graph"
        echo "BLOCKERS:"
        echo "  - 2.2 Drivers / launch config: existing_bringup_processes: --bringup would create duplicate sensor drivers"
        exit 1
    fi
fi

set +e
python3 "${SCRIPT_DIR}/robot_doctor.py" "${ROBOT_ID}" \
    --config "${CONFIG}" \
    --output-root "${OUTPUT_ROOT}" \
    "${DOCTOR_ARGS[@]}" >"${console_log_tmp}" 2>&1
doctor_rc=$?
set -e

find "${OUTPUT_ROOT}" -maxdepth 1 -type d -name "${ROBOT_ID}_*" -print 2>/dev/null | sort > "${after_reports}"
new_report_dir="$(comm -13 "${before_reports}" "${after_reports}" | tail -1 || true)"
summary_json="${new_report_dir}/summary.json"
if [ -z "${summary_json}" ] || [ ! -f "${summary_json}" ]; then
    echo "BLOCKED: robot_doctor did not produce a summary.json report." >&2
    echo ""
    echo "== robot_doctor console tail =="
    tail -120 "${console_log_tmp}" >&2 || true
    exit 1
fi

mkdir -p "${new_report_dir}/logs"
cp "${console_log_tmp}" "${new_report_dir}/logs/dataset_ready_gate_robot_doctor_console.log"

if [ "${VALIDATE_REPORT}" = true ]; then
    echo ""
    echo "== report validation =="
    python3 "${SCRIPT_DIR}/validate_robot_doctor_report.py" --check-evidence "${summary_json}"
fi

echo ""
echo "== decision =="
python3 - "${summary_json}" <<'PY'
import json
import sys
from pathlib import Path

report = json.loads(Path(sys.argv[1]).read_text())
decision = report.get("decision", {}) if isinstance(report.get("decision"), dict) else {}
failure_tree = report.get("failure_tree", {}) if isinstance(report.get("failure_tree"), dict) else {}
checks = report.get("checks", [])
failures = [item for item in checks if item.get("status") == "FAIL"]
warnings = [item for item in checks if item.get("status") == "WARN"]
blocking_warnings = [item for item in warnings if item.get("check") != "bag_validation_missing"]
blocker = failures[0] if failures else (blocking_warnings[0] if blocking_warnings else None)
ready_to_record = not failures and not blocking_warnings
post_run_dataset_ready = bool(report.get("dataset_ready", False))
state = "ready_to_record" if ready_to_record else str(decision.get("state", "blocked"))
print(f"READY_TO_RECORD: {str(ready_to_record).lower()}")
print(f"POST_RUN_DATASET_READY: {str(post_run_dataset_ready).lower()}")
print(f"STATE: {state}")
if blocker:
    code = str(blocker.get("code", "?"))
    branch = failure_tree.get(code, {}).get("name", "?") if isinstance(failure_tree.get(code), dict) else "?"
    print(f"FAILED_STAGE: {code} {branch}")
    print(f"CAUSE: {blocker.get('summary', '?')}")
    evidence = blocker.get("evidence", [])
    print("EVIDENCE:")
    if evidence:
        for path in evidence:
            print(f"  - {path}")
    else:
        print("  - none")
    print(f"NEXT_ACTION: {blocker.get('next_action', 'fix the blocker and rerun the readiness gate')}")
    print("BLOCKERS:")
    for item in (failures + blocking_warnings)[:12]:
        item_code = str(item.get("code", "?"))
        item_branch = failure_tree.get(item_code, {}).get("name", "?") if isinstance(failure_tree.get(item_code), dict) else "?"
        print(f"  - {item_code} {item_branch}: {item.get('check', '?')}: {item.get('summary', '?')}")
else:
    print("FAILED_STAGE: none")
    print("CAUSE: pre-run gate passed; no blocking failures or pre-run warnings")
    print("EVIDENCE:")
    print(f"  - {sys.argv[1]}")
    print("NEXT_ACTION: collect data, then run the post-run bag audit before calling the run publishable")
print(f"REPORT: {sys.argv[1]}")
print("CONSOLE_LOG: " + str(Path(sys.argv[1]).parent / "logs/dataset_ready_gate_robot_doctor_console.log"))
PY

if python3 - "${summary_json}" <<'PY'
import json
import sys
from pathlib import Path
report = json.loads(Path(sys.argv[1]).read_text())
checks = report.get("checks", [])
failures = [item for item in checks if item.get("status") == "FAIL"]
warnings = [item for item in checks if item.get("status") == "WARN"]
blocking_warnings = [item for item in warnings if item.get("check") != "bag_validation_missing"]
if not failures and not blocking_warnings:
    sys.exit(0)
if report.get("decision", {}).get("state") == "review":
    sys.exit(2)
sys.exit(1)
PY
then
    exit 0
else
    readiness_rc=$?
    if [ "${readiness_rc}" -eq 2 ] && [ "${ALLOW_REVIEW}" = true ]; then
        exit 0
    fi
    if [ "${doctor_rc}" -ne 0 ]; then
        exit "${doctor_rc}"
    fi
    exit 1
fi
