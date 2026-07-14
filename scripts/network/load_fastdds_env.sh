#!/usr/bin/env bash
# Compatibility wrapper. New code should source load_ros_transport_env.sh.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/load_ros_transport_env.sh"
