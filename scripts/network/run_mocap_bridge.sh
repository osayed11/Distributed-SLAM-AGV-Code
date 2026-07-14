#!/usr/bin/env bash
# Systemd entry point for the direct NatNet-to-ROS 2 fleet bridge.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/load_fastdds_env.sh"

MOCAP_BRIDGE_ENV="${MOCAP_BRIDGE_ENV:-/etc/orkar/mocap_bridge.env}"
if [ ! -r "${MOCAP_BRIDGE_ENV}" ]; then
    echo "ERROR: missing ${MOCAP_BRIDGE_ENV}; run configure_fastdds.sh bridge." >&2
    exit 1
fi
set -a
# shellcheck disable=SC1090
source "${MOCAP_BRIDGE_ENV}"
set +a

: "${NATNET_SERVER:?NATNET_SERVER is required}"
: "${MOCAP_RIGID_BODIES:?MOCAP_RIGID_BODIES is required}"

args=(
    --server "${NATNET_SERVER}"
    --frame-id "${MOCAP_FRAME_ID:-world}"
    --frame-timeout "${MOCAP_FRAME_TIMEOUT_SEC:-5}"
)
if [ -n "${NATNET_LOCAL_IP:-}" ]; then
    args+=(--local "${NATNET_LOCAL_IP}")
fi
case "${NATNET_MULTICAST:-false}" in
    1|true|TRUE|yes|YES) args+=(--multicast) ;;
esac
for mapping in ${MOCAP_RIGID_BODIES}; do
    args+=(--rigid-body "${mapping}")
done

exec python3 "${ROOT}/scripts/mocap/natnet_ros2_pose_publisher.py" "${args[@]}"
