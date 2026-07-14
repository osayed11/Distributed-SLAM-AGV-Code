#!/usr/bin/env bash
# Convenience wrapper for the unified AGV diagnostic pipeline.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
if [ -r "${ROOT}/scripts/network/load_ros_transport_env.sh" ]; then
    # shellcheck disable=SC1091
    source "${ROOT}/scripts/network/load_ros_transport_env.sh"
fi
exec python3 "${SCRIPT_DIR}/robot_doctor.py" "$@"
