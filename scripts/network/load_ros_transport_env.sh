#!/usr/bin/env bash
# Load the single robot-local ROS transport configuration, if installed.

ORKAR_ROS_TRANSPORT_ENV="${ORKAR_ROS_TRANSPORT_ENV:-/etc/orkar/ros_transport.env}"

if [ -r "${ORKAR_ROS_TRANSPORT_ENV}" ]; then
    # A Zenoh robot is intentionally isolated from Wi-Fi DDS discovery.
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    set -a
    # shellcheck disable=SC1090
    source "${ORKAR_ROS_TRANSPORT_ENV}"
    set +a
elif [ -r "${ORKAR_FASTDDS_ENV:-/etc/orkar/fastdds.env}" ]; then
    # Compatibility fallback for robots not yet migrated to Zenoh.
    set -a
    # shellcheck disable=SC1090
    source "${ORKAR_FASTDDS_ENV:-/etc/orkar/fastdds.env}"
    set +a
fi
