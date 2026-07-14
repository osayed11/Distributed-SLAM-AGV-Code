#!/usr/bin/env bash
# Publish selected NatNet rigid bodies to the global ROS graph for Zenoh export.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROOT}/agv2_ws/install/setup.bash"
set -u

: "${NATNET_SERVER:?NATNET_SERVER is required}"
: "${NATNET_LOCAL_IP:?NATNET_LOCAL_IP is required}"
: "${MOCAP_RIGID_BODIES:?MOCAP_RIGID_BODIES is required}"

# This source must be visible to the laptop-side Zenoh router. Robot sensor
# processes remain on the separate loopback-only graph loaded by login shells.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE ORKAR_ROS_TRANSPORT

args=(
    --server "${NATNET_SERVER}"
    --local "${NATNET_LOCAL_IP}"
    --frame-id "${MOCAP_FRAME_ID:-world}"
    --frame-timeout "${MOCAP_FRAME_TIMEOUT_SEC:-5}"
)
for mapping in ${MOCAP_RIGID_BODIES}; do
    args+=(--rigid-body "${mapping}")
done

exec python3 "${ROOT}/scripts/mocap/natnet_ros2_pose_publisher.py" "${args[@]}"
