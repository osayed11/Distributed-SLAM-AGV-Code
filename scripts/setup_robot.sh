#!/bin/bash
# Optimized for ROS Noetic on Ubuntu 20.04 (ARM64/myAGV).

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
    current_swap=$(free -m | grep -i swap | awk '{print $2}')

    if [ "$current_swap" -lt 2000 ]; then
        echo "Current swap ($current_swap MB) is too small. Increasing to 2GB..."
        sudo swapoff /swapfile 2>/dev/null || true
        sudo fallocate -l 2G /swapfile || sudo dd if=/dev/zero of=/swapfile bs=1M count=2048
        sudo chmod 600 /swapfile
        sudo mkswap /swapfile
        sudo swapon /swapfile || echo "WARN: Could not enable swapfile"

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

ensure_swap

if [ "$INSTALL_SYSTEM" = true ]; then
    section "system dependencies"
    sudo apt-get update
    sudo apt-get install -y \
        build-essential \
        chrony \
        cmake \
        git \
        gnupg2 \
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

    # --- Intel RealSense SDK Automation ---
    if ! dpkg -s librealsense2-dev >/dev/null 2>&1; then
        echo "Configuring Intel RealSense Repo & SDK..."
        sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
        sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
        
        sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u
        
        # Install ARM64 compatible packages (Skipping DKMS)
        sudo apt-get install -y librealsense2-utils librealsense2-dev librealsense2-dbg

        # Apply udev rules for camera access
        wget -q https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
        sudo mv 99-realsense-libusb.rules /etc/udev/rules.d/
        sudo udevadm control --reload-rules && sudo udevadm trigger
    else
        echo "RealSense SDK already installed."
    fi

    sudo systemctl enable --now chrony 2>/dev/null || sudo service chrony restart || true

    # --- RealSense ROS Wrapper Management ---
    section "checking realsense-ros"
    RS_SRC="${ROOT}/agv_ws/src/realsense-ros"
    if [ ! -d "${RS_SRC}" ]; then
        echo "Cloning realsense-ros..."
        mkdir -p "${ROOT}/agv_ws/src"
        git clone https://github.com/IntelRealSense/realsense-ros.git "${RS_SRC}"
    fi
    
    cd "${RS_SRC}"
    echo "Updating realsense-ros to stable ros1-legacy branch..."
    git remote add upstream https://github.com/IntelRealSense/realsense-ros.git || true
    git fetch upstream
    git checkout upstream/ros1-legacy
fi

if [ ! -f "/opt/ros/noetic/setup.bash" ]; then
    echo "ERROR: ROS Noetic not found at /opt/ros/noetic/setup.bash" >&2
    exit 1
fi

section "data directories"
mkdir -p "${HOME}/agv_data"
echo "bags directory created at: ${HOME}/agv_data"

section "building agv_ws workspace"
# Ensure a clean environment for building
source /opt/ros/noetic/setup.bash
export CMAKE_PREFIX_PATH="/opt/ros/noetic"

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

Everything is now optimized for ROS Noetic and ARM64.
EOF
