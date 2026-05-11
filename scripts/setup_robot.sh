#!/usr/bin/env bash
# Build and prepare the AGV robot-side stack after clone/pull.
#
# Usage:
#   bash scripts/setup_robot.sh
#   bash scripts/setup_robot.sh --skip-system
#
# By default this installs expected OS/ROS packages, then builds the catkin
# workspace. Use --skip-system only on an already provisioned/offline robot.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
INSTALL_SYSTEM=true
REQUIRED_LIBREALSENSE_VERSION="${REQUIRED_LIBREALSENSE_VERSION:-2.57.6}"

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

ensure_swap() {
    section "swap check"
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

check_realsense_version() {
    section "realsense sdk"
    if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists realsense2; then
        local version
        version="$(pkg-config --modversion realsense2)"
        echo "pkg-config realsense2: ${version}"
        if [ "${version}" != "${REQUIRED_LIBREALSENSE_VERSION}" ]; then
            echo "WARN: validated RealSense SDK is ${REQUIRED_LIBREALSENSE_VERSION}, found ${version}."
            echo "      Install librealsense2 ${REQUIRED_LIBREALSENSE_VERSION} runtime/dev headers and rebuild agv_ws before scenario runs."
        fi
    else
        echo "WARN: realsense2 pkg-config metadata not found."
        echo "      Install librealsense2 ${REQUIRED_LIBREALSENSE_VERSION} runtime/dev headers before scenario runs."
    fi
}

ensure_catkin_workspace() {
    if [ ! -d "$1/src" ]; then
        echo "ERROR: missing catkin workspace src directory: $1/src" >&2
        exit 1
    fi
    if [ ! -f "$1/.catkin_workspace" ]; then
        touch "$1/.catkin_workspace"
    fi
}

section "repo"
echo "root: ${ROOT}"
ensure_catkin_workspace "${ROOT}/agv_ws"

ensure_swap

if [ "$INSTALL_SYSTEM" = true ]; then
    section "system dependencies"

    # Refresh ROS GPG key
    echo "Refreshing ROS GPG Key..."
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 || true

    sudo apt-get update
    sudo apt-get install -y \
        build-essential \
        chrony \
        cmake \
        git \
        gnupg2 \
        pkg-config \
        python3-pip \
        python3-yaml \
        ros-noetic-apriltag-ros \
        ros-noetic-cv-bridge \
        ros-noetic-ddynamic-reconfigure \
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
        echo "WARN: librealsense2 packages not available from configured apt sources."
        echo "      Install Intel RealSense packages separately if this robot is fresh."
    fi

    sudo systemctl enable --now chrony 2>/dev/null || sudo service chrony restart || true
fi

# ---------------------------------------------------------------------------
# Hardware udev rules
# ---------------------------------------------------------------------------
section "hardware rules"

# RealSense Camera Rules
if [ ! -f "/etc/udev/rules.d/99-realsense-libusb.rules" ]; then
    echo "Attempting to download RealSense udev rules..."
    if wget -q --timeout=10 https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules; then
        sudo mv 99-realsense-libusb.rules /etc/udev/rules.d/
        echo "[✓] RealSense rules installed."
    else
        echo "[!] WARN: Could not reach GitHub to download camera rules. Skipping..."
    fi
fi

# AGV Base Controller Rule (/dev/myAGV)
echo "Installing AGV base controller rules..."
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", SYMLINK+="myAGV"' | sudo tee /etc/udev/rules.d/99-myagv-base.rules > /dev/null

# YDLidar X2 Rule (/dev/ydlidar)
echo "Installing YDLidar rules..."
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="ydlidar"' | sudo tee /etc/udev/rules.d/99-ydlidar.rules > /dev/null

sudo udevadm control --reload-rules && sudo udevadm trigger

# ---------------------------------------------------------------------------
# Validate ROS and dependencies
# ---------------------------------------------------------------------------
if [ -f /opt/ros/noetic/setup.bash ]; then
    require_file "/opt/ros/noetic/setup.bash"
elif [ -f /opt/ros/melodic/setup.bash ]; then
    require_file "/opt/ros/melodic/setup.bash"
else
    echo "ERROR: no supported ROS setup found under /opt/ros" >&2
    exit 1
fi

if ! command -v chronyc >/dev/null 2>&1; then
    echo "ERROR: chronyc not found; install chrony or rerun without --skip-system." >&2
    exit 1
fi

if ! chronyc tracking >/dev/null 2>&1; then
    echo "WARN: chrony is installed but not reporting tracking status yet."
fi

check_realsense_version

section "data directories"
mkdir -p "${HOME}/agv_data"
echo "bags: ${HOME}/agv_data"

# ---------------------------------------------------------------------------
# Build workspace
# ---------------------------------------------------------------------------
section "build agv_ws"
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
fi
cd "${ROOT}/agv_ws"
catkin_make

section "workspace check"
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
fi
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
source ${ROOT}/agv_ws/devel/setup.bash

# One-command data run:
bash ${ROOT}/scripts/logging/start_session.sh agv1 square_manual

# Optional manual teleop in another terminal:
rosrun myagv_teleop myagv_teleop.py
EOF
