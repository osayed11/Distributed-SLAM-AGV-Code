#!/usr/bin/env bash
# Build and prepare the AGV robot-side stack after clone/pull.
#
# Usage:
#   bash scripts/setup_robot.sh
#   bash scripts/setup_robot.sh --skip-system
#
# By default this installs expected OS/ROS packages, then builds whichever
# workspace matches the ROS version on this robot (agv_ws for ROS1, agv2_ws
# for ROS2). Use --skip-system only on an already provisioned/offline robot.

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

# ---------------------------------------------------------------------------
# Detect which ROS versions are present on this machine
# ---------------------------------------------------------------------------
HAS_ROS1=false
HAS_ROS2=false
ROS1_DISTRO=""
ROS2_DISTRO=""

if   [ -f /opt/ros/noetic/setup.bash ];  then HAS_ROS1=true; ROS1_DISTRO=noetic
elif [ -f /opt/ros/melodic/setup.bash ]; then HAS_ROS1=true; ROS1_DISTRO=melodic
fi

if   [ -f /opt/ros/humble/setup.bash ]; then HAS_ROS2=true; ROS2_DISTRO=humble
elif [ -f /opt/ros/iron/setup.bash ];   then HAS_ROS2=true; ROS2_DISTRO=iron
elif [ -f /opt/ros/jazzy/setup.bash ];  then HAS_ROS2=true; ROS2_DISTRO=jazzy
fi

echo "Detected ROS1: ${HAS_ROS1} (${ROS1_DISTRO:-none})"
echo "Detected ROS2: ${HAS_ROS2} (${ROS2_DISTRO:-none})"

if [ "$HAS_ROS1" = false ] && [ "$HAS_ROS2" = false ]; then
    echo "ERROR: no supported ROS installation found under /opt/ros" >&2
    exit 1
fi

# Detect Ubuntu codename for apt source selection
UBUNTU_CODENAME=$(lsb_release -cs 2>/dev/null || echo "focal")

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
            return 1
        fi
        return 0
    else
        echo "WARN: realsense2 pkg-config metadata not found."
        echo "      Need librealsense2 ${REQUIRED_LIBREALSENSE_VERSION} runtime/dev headers."
        return 1
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
if [ "$HAS_ROS1" = true ]; then
    ensure_catkin_workspace "${ROOT}/agv_ws"
fi

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
        libboost-dev \
        libboost-system-dev \
        lsb-release \
        pkg-config \
        python3-pip \
        python3-yaml

    # --- ROS1 packages ---
    if [ "$HAS_ROS1" = true ]; then
        echo "Installing ROS1 (${ROS1_DISTRO}) packages..."
        # Refresh ROS1 GPG key
        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' \
            --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 || true

        sudo apt-get install -y \
            ros-${ROS1_DISTRO}-apriltag-ros \
            ros-${ROS1_DISTRO}-cv-bridge \
            ros-${ROS1_DISTRO}-ddynamic-reconfigure \
            ros-${ROS1_DISTRO}-diagnostic-msgs \
            ros-${ROS1_DISTRO}-geometry-msgs \
            ros-${ROS1_DISTRO}-image-transport-plugins \
            ros-${ROS1_DISTRO}-nav-msgs \
            ros-${ROS1_DISTRO}-rosbag \
            ros-${ROS1_DISTRO}-sensor-msgs \
            ros-${ROS1_DISTRO}-std-msgs \
            ros-${ROS1_DISTRO}-tf \
            ros-${ROS1_DISTRO}-tf2-msgs
    fi

    # --- ROS2 packages ---
    if [ "$HAS_ROS2" = true ]; then
        echo "Installing ROS2 (${ROS2_DISTRO}) packages..."
        sudo apt-get install -y \
            ros-${ROS2_DISTRO}-rclcpp \
            ros-${ROS2_DISTRO}-rclpy \
            ros-${ROS2_DISTRO}-nav-msgs \
            ros-${ROS2_DISTRO}-sensor-msgs \
            ros-${ROS2_DISTRO}-geometry-msgs \
            ros-${ROS2_DISTRO}-tf2-ros \
            ros-${ROS2_DISTRO}-tf2-geometry-msgs \
            ros-${ROS2_DISTRO}-launch-ros \
            ros-${ROS2_DISTRO}-cv-bridge \
            ros-${ROS2_DISTRO}-image-transport \
            ros-${ROS2_DISTRO}-diagnostic-msgs \
            ros-${ROS2_DISTRO}-apriltag-ros \
            ros-${ROS2_DISTRO}-realsense2-camera \
            python3-colcon-common-extensions \
            python3-colcon-mixin
    fi

    # --- Intel RealSense SDK ---
    RS_OK=true
    check_realsense_version || RS_OK=false
    if [ "$RS_OK" = false ]; then
        echo "Configuring Intel RealSense apt repo..."

        # Clean up any old/broken realsense sources
        sudo rm -f /etc/apt/sources.list.d/realsense* || true
        sudo rm -f /etc/apt/sources.list.d/librealsense* || true
        sudo sed -i '/librealsense.intel.com/d' /etc/apt/sources.list || true

        # Add Intel GPG key
        sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key FB0B24895113F120 || \
        sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key FB0B24895113F120

        # Use the correct apt repo for this Ubuntu version (focal=20.04, jammy=22.04)
        sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo ${UBUNTU_CODENAME} main" -u

        sudo apt-get install -y librealsense2-utils librealsense2-dev librealsense2-dbg
    else
        echo "RealSense SDK already installed."
    fi

    # --- YDLidar SDK (native C library, required by both ROS1 and ROS2 drivers) ---
    if ! command -v ydlidar_test >/dev/null 2>&1; then
        section "ydlidar sdk"
        echo "Building and installing YDLidar-SDK..."
        cd "${ROOT}/drivers/YDLidar-SDK"
        mkdir -p build && cd build
        cmake ..
        make -j$(nproc)
        sudo make install
        cd "${ROOT}"
    else
        echo "YDLidar SDK already installed."
    fi

    # --- Permissions ---
    echo "Ensuring user permissions for hardware..."
    sudo usermod -a -G dialout $USER || true
    sudo usermod -a -G video $USER || true

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

# YDLidar X2 (built-in UART on Pi)
echo "Setting permissions for YDLidar on /dev/ttyAMA0..."
echo 'KERNEL=="ttyAMA0", MODE:="0666"' | sudo tee /etc/udev/rules.d/99-ydlidar.rules > /dev/null

sudo udevadm control --reload-rules && sudo udevadm trigger

# ---------------------------------------------------------------------------
# Validate common dependencies
# ---------------------------------------------------------------------------
if ! command -v chronyc >/dev/null 2>&1; then
    echo "ERROR: chronyc not found; install chrony or rerun without --skip-system." >&2
    exit 1
fi

if ! chronyc tracking >/dev/null 2>&1; then
    echo "WARN: chrony is installed but not reporting tracking status yet."
fi

check_realsense_version || true

section "data directories"
mkdir -p "${HOME}/agv_data"
echo "bags: ${HOME}/agv_data"

# ---------------------------------------------------------------------------
# Build ROS1 workspace (catkin_make)
# ---------------------------------------------------------------------------
if [ "$HAS_ROS1" = true ]; then
    section "build agv_ws (ROS1 / catkin)"
    set +u
    source "/opt/ros/${ROS1_DISTRO}/setup.bash"
    set -u
    cd "${ROOT}/agv_ws"
    catkin_make

    section "ROS1 workspace check"
    set +u
    source "${ROOT}/agv_ws/devel/setup.bash"
    set -u
    rospack find agv_bringup
    rospack find realsense2_camera
    rospack find ydlidar_ros_driver
    rospack find myagv_odometry
fi

# ---------------------------------------------------------------------------
# Build ROS2 workspace (colcon)
# ---------------------------------------------------------------------------
if [ "$HAS_ROS2" = true ] && [ -d "${ROOT}/agv2_ws/src" ]; then
    section "build agv2_ws (ROS2 / colcon)"
    set +u
    source "/opt/ros/${ROS2_DISTRO}/setup.bash"
    set -u
    cd "${ROOT}/agv2_ws"
    colcon build --symlink-install

    section "ROS2 workspace check"
    set +u
    source "${ROOT}/agv2_ws/install/setup.bash"
    set -u
    for pkg in agv_bringup myagv_odometry myagv_teleop realsense2_camera ydlidar_ros2_driver; do
        if ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
            echo "[OK] ${pkg}"
        else
            echo "[MISSING] ${pkg}"
        fi
    done
fi

section "script permissions"
chmod +x \
    "${ROOT}/scripts/logging/start_session.sh" \
    "${ROOT}/scripts/logging/drive_straight.py" \
    "${ROOT}/scripts/logging/drive_square.py" \
    "${ROOT}/scripts/logging/drive_circle.py" \
    "${ROOT}/scripts/logging/drive_forward_back.py" \
    "${ROOT}/scripts/logging/drive_odom_shuttle.py" \
    "${ROOT}/scripts/logging/launch_odom_shuttle_fleet.sh" \
    "${ROOT}/scripts/logging/validate_bag.py" \
    "${ROOT}/scripts/logging/audit_bag_fast.py" \
    "${ROOT}/scripts/diagnostics/"*.sh 2>/dev/null || true

section "next commands"
if [ "$HAS_ROS2" = true ]; then
    cat <<EOF
source /opt/ros/${ROS2_DISTRO}/setup.bash
source ${ROOT}/agv2_ws/install/setup.bash

# One-command data run:
bash ${ROOT}/scripts/logging/start_session.sh agv1 square_manual

# Optional manual teleop in another terminal:
ros2 run myagv_teleop myagv_teleop
EOF
else
    cat <<EOF
source /opt/ros/${ROS1_DISTRO}/setup.bash
source ${ROOT}/agv_ws/devel/setup.bash

# One-command data run:
bash ${ROOT}/scripts/logging/start_session.sh agv1 square_manual

# Optional manual teleop in another terminal:
rosrun myagv_teleop myagv_teleop.py
EOF
fi
