#!/usr/bin/env bash
# Source the robot-local Fast DDS client configuration, if installed.

ORKAR_FASTDDS_ENV="${ORKAR_FASTDDS_ENV:-/etc/orkar/fastdds.env}"
if [ -r "${ORKAR_FASTDDS_ENV}" ]; then
    set -a
    # shellcheck disable=SC1090
    source "${ORKAR_FASTDDS_ENV}"
    set +a
fi
