#!/usr/bin/env bash
# Build and prepare the AGV robot-side stack after clone/pull.
# Optimized for ROS Noetic on Ubuntu 20.04.

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
    echo "========================================"
    echo "== $1"
    echo "========================================"
}

ensure_swap() {
    section "checking swap"
    # Check current swap in Megabytes
    current_swap=$(free -m | grep -i swap | awk '{print $2}')
    
    if [ "$current_swap" -lt 2000 ]; then
        echo "Current swap ($current_swap MB) is too small. Increasing to 2GB..."
        
        # Disable existing swapfile if present to allow resize
        sudo swapoff /swapfile 2>/dev/null || true
        
        # Create 2GB swap file - fallocate is fast, dd is a backup
        sudo fallocate -l 2G /swapfile || sudo dd if=/dev/zero of=/swapfile bs=1M count=2048
        sudo chmod 600 /swapfile
        sudo mkswap /swapfile
        sudo swapon /swapfile || echo "WARN: Could not enable swapfile"
        
        # Persistence
        if ! grep -q "/swapfile" /etc/fstab; then
            echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
        fi
        echo "Swap successfully configured."
    else
        echo "Swap is sufficient ($current_swap MB)."
    fi
}

ensure_file() {
    if [ ! -f "$1" ]; then
        echo "Initializing missing marker: $1"
        mkdir -p "$(dirname "$1")"
        touch "$1"
    fi
}

section "repository initialization"
echo "root: ${ROOT}"
ensure_file "${ROOT}/agv_ws/.catkin_workspace"

# Handle swap before system updates to prevent build crashes later
ensure_swap

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
        python3-rosdep \
        python3-rosinstall \
        ros-noetic-ddynamic-reconfigure \
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
        echo "WARN: librealsense2-dev not found in apt sources."
    fi

    sudo systemctl enable --now chrony 2>/dev/null || sudo service chrony restart || true
fi

# Final Environment Checks
if [ ! -f "/opt/ros/noetic/setup.bash" ]; then
    echo "ERROR: ROS Noetic not found at /opt/ros/noetic/setup.bash" >&2
    exit 1
fi

section "data directories"
mkdir -p "${HOME}/agv_data"
echo "bags directory created at: ${HOME}/agv_data"


section "building agv_ws workspace"
source /opt/ros/noetic/setup.bash
cd "${ROOT}/agv_ws"
rm -rf build devel
catkin_make -j1

section "script permissions"
chmod +x \
    "${ROOT}/scripts/logging/start_session.sh" \
    "${ROOT}/scripts/logging/drive_straight.py" \
    "${ROOT}/scripts/logging/drive_square.py" \
    "${ROOT}/scripts/logging/drive_forward_back.py" \
    "${ROOT}/scripts/logging/validate_bag.py" \
    "${ROOT}/scripts/logging/audit_bag_fast.py" \
    "${ROOT}/scripts/diagnostics/"*.sh 2>/dev/null || true

section "automation complete"
cat <<EOF
To begin using the robot:

1. Load environment:
   source /opt/ros/noetic/setup.bash
   source ${ROOT}/agv_ws/devel/setup.bash

2. Launch full stack:
   bash ${ROOT}/scripts/logging/start_session.sh agv1 square_manual

Everything is now optimized for ROS Noetic.
EOF
