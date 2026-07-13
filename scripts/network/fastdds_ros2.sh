#!/usr/bin/env bash
# Run ROS 2 CLI introspection as a Fast DDS Super Client.

set -euo pipefail

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

PROFILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/etc/orkar/fastdds_super_client.xml}"
if [ ! -r "${PROFILE}" ]; then
    echo "ERROR: missing ${PROFILE}; run configure_fastdds.sh client first." >&2
    exit 1
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="${PROFILE}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
unset ROS_DISCOVERY_SERVER

if [ "$#" -eq 0 ]; then
    echo "Usage: bash scripts/network/fastdds_ros2.sh <ros2 arguments...>" >&2
    echo "Example: bash scripts/network/fastdds_ros2.sh topic list --no-daemon" >&2
    exit 2
fi
exec ros2 "$@"
