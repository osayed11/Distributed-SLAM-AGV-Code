#!/usr/bin/env bash
# Run robot_doctor over a fleet host list and aggregate copied reports.
#
# Host list format, one robot per line:
#   <robot_id> <robot-ip-or-hostname>
#
# Blank lines and lines starting with # are ignored.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

usage() {
    sed -n '1,12p' "$0"
    cat <<'EOF'

Usage:
  SSH_PASS=ubuntu bash scripts/diagnostics/run_fleet_doctor_remote.sh hosts.txt --strict-fleet -- \
    --config configs/robot_doctor_dataset_gate.json --profile preflight

Environment:
  LOCAL_OUTPUT_ROOT   default: <repo>/diagnostic_reports/fleet_<timestamp>
  FLEET_CONTINUE      default: true; keep going after one robot fails
  FLEET_REMOTE_WRAPPER default: run_robot_doctor_remote.sh; override only for tests

EOF
}

if [ "$#" -lt 1 ]; then
    usage >&2
    exit 2
fi

HOSTS_FILE="$1"
shift
SUMMARY_ARGS=()
while [ "$#" -gt 0 ]; do
    case "$1" in
        --)
            shift
            break
            ;;
        --strict-fleet|--require-dataset-ready|--require-same-gate|--require-same-config|--require-same-commit|--require-clean-repo)
            SUMMARY_ARGS+=("$1")
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            # Backwards-compatible path: remaining args are robot_doctor args.
            break
            ;;
    esac
done

if [ ! -f "${HOSTS_FILE}" ]; then
    echo "ERROR: host list not found: ${HOSTS_FILE}" >&2
    exit 2
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
LOCAL_OUTPUT_ROOT="${LOCAL_OUTPUT_ROOT:-${ROOT}/diagnostic_reports/fleet_${STAMP}}"
FLEET_CONTINUE="${FLEET_CONTINUE:-true}"
REMOTE_WRAPPER="${FLEET_REMOTE_WRAPPER:-${SCRIPT_DIR}/run_robot_doctor_remote.sh}"
mkdir -p "${LOCAL_OUTPUT_ROOT}"

REPORTS_FILE="${LOCAL_OUTPUT_ROOT}/summary_paths.txt"
FAILURES_FILE="${LOCAL_OUTPUT_ROOT}/failures.txt"
SUMMARY_RC_FILE="${LOCAL_OUTPUT_ROOT}/summary_rc.txt"
: > "${REPORTS_FILE}"
: > "${FAILURES_FILE}"
: > "${SUMMARY_RC_FILE}"

run_one() {
    local robot_id="$1"
    local host="$2"
    shift 2
    local before after latest

    echo ""
    echo "========================================================================"
    echo "robot=${robot_id} host=${host}"
    echo "========================================================================"

    before="$(find "${LOCAL_OUTPUT_ROOT}" -name summary.json -print 2>/dev/null | sort || true)"
    set +e
    LOCAL_OUTPUT_ROOT="${LOCAL_OUTPUT_ROOT}" \
        bash "${REMOTE_WRAPPER}" "${robot_id}" "${host}" -- "$@"
    local rc=$?
    set -e
    after="$(find "${LOCAL_OUTPUT_ROOT}" -name summary.json -print 2>/dev/null | sort || true)"
    latest="$(comm -13 <(printf '%s\n' "${before}") <(printf '%s\n' "${after}") | tail -1 || true)"
    if [ -n "${latest}" ]; then
        printf '%s\n' "${latest}" >> "${REPORTS_FILE}"
    fi
    if [ "${rc}" -ne 0 ]; then
        printf '%s %s rc=%s\n' "${robot_id}" "${host}" "${rc}" >> "${FAILURES_FILE}"
    fi
    return "${rc}"
}

while read -r robot_id host rest; do
    case "${robot_id:-}" in
        ""|\#*)
            continue
            ;;
    esac
    if [ -z "${host:-}" ]; then
        echo "WARN: skipping malformed host line for ${robot_id}" >&2
        continue
    fi
    if ! run_one "${robot_id}" "${host}" "$@"; then
        if [ "${FLEET_CONTINUE}" != true ]; then
            break
        fi
    fi
done < "${HOSTS_FILE}"

echo ""
echo "========================================================================"
echo "fleet summary"
echo "========================================================================"
if [ -s "${REPORTS_FILE}" ]; then
    reports=()
    while IFS= read -r report_path; do
        [ -n "${report_path}" ] && reports+=("${report_path}")
    done < "${REPORTS_FILE}"
    set +e
    python3 "${SCRIPT_DIR}/fleet_doctor_summary.py" "${reports[@]}" \
        --json-out "${LOCAL_OUTPUT_ROOT}/fleet_summary.json" \
        "${SUMMARY_ARGS[@]}"
    summary_rc=$?
    set -e
    printf '%s\n' "${summary_rc}" > "${SUMMARY_RC_FILE}"
else
    echo "No reports were copied."
    printf '1\n' > "${SUMMARY_RC_FILE}"
fi

summary_rc="$(cat "${SUMMARY_RC_FILE}")"

if [ -s "${FAILURES_FILE}" ]; then
    echo ""
    echo "Robots needing attention:"
    cat "${FAILURES_FILE}"
    exit 1
fi

if [ "${summary_rc}" -ne 0 ]; then
    echo ""
    echo "Fleet summary failed strictness/report checks."
    exit "${summary_rc}"
fi

echo "local_output_root=${LOCAL_OUTPUT_ROOT}"
