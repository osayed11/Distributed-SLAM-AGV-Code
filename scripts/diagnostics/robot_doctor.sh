#!/usr/bin/env bash
# Convenience wrapper for the unified AGV diagnostic pipeline.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "${SCRIPT_DIR}/robot_doctor.py" "$@"
