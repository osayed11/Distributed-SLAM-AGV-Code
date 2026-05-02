#!/usr/bin/env bash
# Build and prepare the AGV robot-side stack after clone/pull.
#
# Usage:
#   bash scripts/setup_robot.sh
#   bash scripts/setup_robot.sh --skip-system
#
# By default this installs expected OS/ROS packages, then builds both catkin
# workspaces. Use --skip-system only on an already provisioned/offline robot.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
INSTALL_SYSTEM=true

for arg in "$@"; do
    case "$arg" in
        --skip-system)
            INSTALL_SYSTEM=false
            ;;
        -h|--help)
            sed -n '1,16p' "$0"
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg" >&2
            exit 2
            ;;
    esac
done

section() {
    echo ""
    echo "== $1 =="
}

require_file() {
    if [ ! -f "$1" ]; then
        echo "ERROR: missing required file: $1" >&2
        exit 1
    fi
}

section "repo"
echo "root: ${ROOT}"
require_file "${ROOT}/myagv_ros/.catkin_workspace"
require_file "${ROOT}/agv_ws/.catkin_workspace"

if [ "$INSTALL_SYSTEM" = true ]; then
    section "system dependencies"
    sudo apt-get update
    sudo apt-get install -y \
        build-essential \
        chrony \
        cmake \
        git \
        python3-pip \
        python3-yaml \
        ros-noetic-apriltag-ros \
        ros-noetic-diagnostic-msgs \
        ros-noetic-geometry-msgs \
        ros-noetic-image-transport-plugins \
        ros-noetic-nav-msgs \
        ros-noetic-rosbag \
        ros-noetic-sensor-msgs \
        ros-noetic-std-msgs \
        ros-noetic-tf \
        ros-noetic-tf2-msgs

    if apt-cache show librealsense2-dev >/dev/null 2>&1; then
        sudo apt-get install -y librealsense2-dev librealsense2-utils
    else
        echo "WARN: librealsense2-dev not available from configured apt sources."
        echo "      Install Intel RealSense packages separately if this robot is fresh."
    fi

    sudo systemctl enable --now chrony 2>/dev/null || sudo service chrony restart || true
fi

require_file "/opt/ros/noetic/setup.bash"

if ! command -v chronyc >/dev/null 2>&1; then
    echo "ERROR: chronyc not found; install chrony or rerun without --skip-system." >&2
    exit 1
fi

if ! chronyc tracking >/dev/null 2>&1; then
    echo "WARN: chrony is installed but not reporting tracking status yet."
fi

section "data directories"
mkdir -p "${HOME}/agv_data"
echo "bags: ${HOME}/agv_data"

section "build myagv_ros"
source /opt/ros/noetic/setup.bash
cd "${ROOT}/myagv_ros"
catkin_make

section "build agv_ws"
source /opt/ros/noetic/setup.bash
source "${ROOT}/myagv_ros/devel/setup.bash"
cd "${ROOT}/agv_ws"
catkin_make

section "workspace check"
source /opt/ros/noetic/setup.bash
source "${ROOT}/myagv_ros/devel/setup.bash"
source "${ROOT}/agv_ws/devel/setup.bash"
rospack find agv_bringup
rospack find realsense2_camera
rospack find ydlidar_ros_driver
rospack find myagv_odometry

section "script permissions"
chmod +x \
    "${ROOT}/scripts/logging/start_session.sh" \
    "${ROOT}/scripts/logging/drive_straight.py" \
    "${ROOT}/scripts/logging/drive_square.py" \
    "${ROOT}/scripts/logging/drive_forward_back.py" \
    "${ROOT}/scripts/logging/validate_bag.py" \
    "${ROOT}/scripts/logging/audit_bag_fast.py" \
    "${ROOT}/scripts/diagnostics/"*.sh 2>/dev/null || true

section "next commands"
cat <<EOF
source /opt/ros/noetic/setup.bash
source ${ROOT}/myagv_ros/devel/setup.bash
source ${ROOT}/agv_ws/devel/setup.bash

# One-command data run:
bash ${ROOT}/scripts/logging/start_session.sh agv1 square_manual

# Optional manual teleop in another terminal:
rosrun myagv_teleop myagv_teleop.py
EOF
