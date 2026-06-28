#!/usr/bin/env bash
# Provision a ROS 2 AGV robot for repeatable dataset collection.
#
# Usage:
#   bash scripts/setup_robot_ros2.sh agv102
#   bash scripts/setup_robot_ros2.sh agv102 --skip-system
#   SUDO_PASSWORD=ubuntu bash scripts/setup_robot_ros2.sh agv102
#
# This is the only supported provisioning script for the ROS 2 dataset robots.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROBOT_ID="${1:-agv_unknown}"
if [ "$#" -gt 0 ] && [[ "${1:-}" != --* ]]; then
    shift
fi

INSTALL_SYSTEM=true
INSTALL_REALSENSE=true
INSTALL_YDLIDAR_SDK=true
BUILD_WS=true
RUN_DOCTOR=true
APPLY_LOW_RISK_FIXES=true

ROS_DISTRO="${ROS_DISTRO:-humble}"
EXPECTED_LIBREALSENSE="${EXPECTED_LIBREALSENSE:-2.58.1}"
EXPECTED_REALSENSE_ROS_DRIVER="${EXPECTED_REALSENSE_ROS_DRIVER:-4.57.7}"
EXPECTED_REALSENSE_ROS_LIBREALSENSE="${EXPECTED_REALSENSE_ROS_LIBREALSENSE:-2.57.7}"
EXPECTED_D455_FIRMWARE="${EXPECTED_D455_FIRMWARE:-5.17.0.10}"
PYREALSENSE2_PIP_VERSION="${PYREALSENSE2_PIP_VERSION:-2.58.1.10581}"
ALLOW_REALSENSE_VERSION_DRIFT="${ALLOW_REALSENSE_VERSION_DRIFT:-false}"
REALSENSE_REPO_TRUST_MODE="${REALSENSE_REPO_TRUST_MODE:-auto}"

usage() {
    sed -n '1,11p' "$0"
    cat <<'EOF'

Options:
  --skip-system            Do not install apt packages.
  --skip-realsense         Do not add/install RealSense apt packages.
  --skip-ydlidar-sdk       Do not build/install the native YDLidar SDK.
  --skip-build             Do not build agv2_ws.
  --no-doctor              Do not run robot_doctor at the end.
  --no-low-risk-fixes      Do not install D455 low-risk udev rules.

Environment:
  SUDO_PASSWORD=ubuntu     Enables non-interactive sudo over SSH.
  PYREALSENSE2_PIP_VERSION=2.58.1.10581
                           User-local pyrealsense2 fallback when apt lacks
                           python3-pyrealsense2 for arm64.
  ALLOW_REALSENSE_VERSION_DRIFT=true
                           Install available RealSense packages even if the
                           expected package version prefix is unavailable.
  REALSENSE_REPO_TRUST_MODE=auto|signed|trusted
                           auto tries signed-by first, then logs and falls
                           back to trusted=yes if Intel's repo key is broken.
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --skip-system)
            INSTALL_SYSTEM=false
            ;;
        --skip-realsense)
            INSTALL_REALSENSE=false
            ;;
        --skip-ydlidar-sdk)
            INSTALL_YDLIDAR_SDK=false
            ;;
        --skip-build)
            BUILD_WS=false
            ;;
        --no-doctor)
            RUN_DOCTOR=false
            ;;
        --no-low-risk-fixes)
            APPLY_LOW_RISK_FIXES=false
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

section() {
    echo ""
    echo "== $1 =="
}

sudo_run() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
    elif [ -n "${SUDO_PASSWORD:-}" ]; then
        printf '%s\n' "${SUDO_PASSWORD}" | sudo -S -p "" "$@"
    else
        sudo "$@"
    fi
}

sudo_write_file() {
    local path="$1"
    local mode="$2"
    local content="$3"
    local tmp
    tmp="$(mktemp)"
    printf '%s\n' "${content}" > "${tmp}"
    sudo_run install -m "${mode}" "${tmp}" "${path}"
    rm -f "${tmp}"
}

source_ros() {
    local setup="/opt/ros/${ROS_DISTRO}/setup.bash"
    if [ ! -f "${setup}" ]; then
        echo "ERROR: missing ${setup}. Install ROS 2 ${ROS_DISTRO} first." >&2
        exit 1
    fi
    set +u
    # ROS setup scripts are not nounset-safe.
    source "${setup}"
    if [ -f "${ROOT}/agv2_ws/install/setup.bash" ]; then
        source "${ROOT}/agv2_ws/install/setup.bash"
    fi
    set -u
}

apt_install() {
    sudo_run apt-get install -y "$@"
}

disable_legacy_realsense_sources() {
    local old_source
    local stamp
    stamp="$(date +%Y%m%d%H%M%S)"
    for old_source in /etc/apt/sources.list.d/*librealsense* /etc/apt/sources.list.d/archive_uri-https_librealsense*; do
        [ -e "${old_source}" ] || continue
        [ "${old_source}" = "/etc/apt/sources.list.d/librealsense.list" ] && continue
        case "${old_source}" in
            *.disabled-by-setup-*) continue ;;
        esac
        sudo_run mv "${old_source}" "${old_source}.disabled-by-setup-${stamp}"
    done
}

write_realsense_source() {
    local mode="$1"
    local source_line
    case "${mode}" in
        signed)
            source_line="deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo jammy main"
            ;;
        trusted)
            source_line="deb [trusted=yes] https://librealsense.intel.com/Debian/apt-repo jammy main"
            ;;
        *)
            echo "ERROR: invalid RealSense repo source mode: ${mode}" >&2
            exit 2
            ;;
    esac
    sudo_write_file /etc/apt/sources.list.d/librealsense.list 0644 "${source_line}"
}

apt_update_logged() {
    local log_file="$1"
    set +e
    sudo_run apt-get update 2>&1 | tee "${log_file}"
    local rc="${PIPESTATUS[0]}"
    set -e
    return "${rc}"
}

apt_candidate_with_prefix() {
    local pkg="$1"
    local prefix="$2"
    apt-cache madison "${pkg}" 2>/dev/null \
        | awk '{print $3}' \
        | awk -v p="${prefix}" 'index($0, p) == 1 {print; exit}'
}

install_expected_version_package() {
    local pkg="$1"
    local expected_prefix="$2"
    local version
    version="$(apt_candidate_with_prefix "${pkg}" "${expected_prefix}")"
    if [ -n "${version}" ]; then
        apt_install "${pkg}=${version}"
        return
    fi
    if [ "${ALLOW_REALSENSE_VERSION_DRIFT}" = "true" ]; then
        echo "WARN: no ${pkg} candidate starts with ${expected_prefix}; installing available candidate."
        apt_install "${pkg}"
        return
    fi
    echo "ERROR: no ${pkg} candidate starts with ${expected_prefix}." >&2
    echo "       Set ALLOW_REALSENSE_VERSION_DRIFT=true only for debugging, not dataset standardization." >&2
    exit 1
}

ensure_realsense_repo() {
    section "realsense apt repo"
    local update_log
    local tmp_key
    case "${REALSENSE_REPO_TRUST_MODE}" in
        auto|signed|trusted) ;;
        *)
            echo "ERROR: REALSENSE_REPO_TRUST_MODE must be auto, signed, or trusted." >&2
            exit 2
            ;;
    esac
    apt_install ca-certificates curl gnupg lsb-release
    sudo_run install -d -m 0755 /etc/apt/keyrings
    tmp_key="$(mktemp)"
    curl -fsSL -o "${tmp_key}" https://librealsense.intel.com/Debian/librealsense.pgp
    sudo_run install -m 0644 "${tmp_key}" /etc/apt/keyrings/librealsense.pgp
    rm -f "${tmp_key}"
    disable_legacy_realsense_sources
    if [ "${REALSENSE_REPO_TRUST_MODE}" = "trusted" ]; then
        echo "WARN: using RealSense apt repo with trusted=yes by operator request."
        write_realsense_source trusted
        sudo_run apt-get update
        return
    fi

    update_log="$(mktemp)"
    write_realsense_source signed
    if apt_update_logged "${update_log}" && ! grep -q "NO_PUBKEY" "${update_log}"; then
        rm -f "${update_log}"
        return
    fi
    if ! grep -q "NO_PUBKEY" "${update_log}"; then
        rm -f "${update_log}"
        echo "ERROR: apt-get update failed for the RealSense repo; see output above." >&2
        exit 1
    fi
    if [ "${REALSENSE_REPO_TRUST_MODE}" = "signed" ]; then
        rm -f "${update_log}"
        echo "ERROR: Intel RealSense repo key verification failed and signed mode was requested." >&2
        echo "       Set REALSENSE_REPO_TRUST_MODE=auto or trusted only if you accept the explicit trusted=yes fallback." >&2
        exit 1
    fi
    echo "WARN: Intel RealSense repo key verification failed; falling back to trusted=yes."
    echo "      Package versions are still checked against ${EXPECTED_LIBREALSENSE} after install."
    write_realsense_source trusted
    rm -f "${update_log}"
    sudo_run apt-get update
}

install_realsense_stack() {
    section "realsense packages"
    install_expected_version_package librealsense2 "${EXPECTED_LIBREALSENSE}"
    install_expected_version_package librealsense2-dev "${EXPECTED_LIBREALSENSE}"
    install_expected_version_package librealsense2-utils "${EXPECTED_LIBREALSENSE}"
    install_expected_version_package librealsense2-udev-rules "${EXPECTED_LIBREALSENSE}"
    if apt-cache show python3-pyrealsense2 >/dev/null 2>&1; then
        install_expected_version_package python3-pyrealsense2 "${EXPECTED_LIBREALSENSE}"
    else
        echo "WARN: python3-pyrealsense2 is not available from apt."
        echo "      Installing user-local pyrealsense2==${PYREALSENSE2_PIP_VERSION} from pip."
        python3 -m pip install --user "pyrealsense2==${PYREALSENSE2_PIP_VERSION}"
    fi

    for pkg in librealsense2 librealsense2-dev librealsense2-utils librealsense2-udev-rules python3-pyrealsense2; do
        if dpkg-query -W -f='${Status}' "${pkg}" 2>/dev/null | grep -q "ok installed"; then
            sudo_run apt-mark hold "${pkg}" >/dev/null || true
        fi
    done
}

install_d455_boot_quirk() {
    section "d455 usb boot quirk"
    local cmdline_file="/boot/firmware/cmdline.txt"
    if [ ! -f "${cmdline_file}" ]; then
        cmdline_file="/boot/cmdline.txt"
    fi
    if [ ! -f "${cmdline_file}" ]; then
        echo "WARN: boot cmdline file not found; cannot install usbcore D455 quirk."
        return
    fi
    if grep -qw "usbcore.quirks=8086:0b5c:kn" "${cmdline_file}"; then
        echo "usbcore.quirks=8086:0b5c:kn already present in ${cmdline_file}"
        return
    fi
    sudo_run cp "${cmdline_file}" "${cmdline_file}.bak.$(date +%Y%m%d_%H%M%S)"
    python_script='from pathlib import Path
import sys
path = Path(sys.argv[1])
parts = [item for item in path.read_text().strip().split() if not item.startswith("usbcore.quirks=")]
parts.append("usbcore.quirks=8086:0b5c:kn")
path.write_text(" ".join(parts) + "\n")'
    printf '%s\n' "${python_script}" | sudo_run python3 - "${cmdline_file}"
    echo "installed usbcore.quirks=8086:0b5c:kn in ${cmdline_file}; reboot required"
}

install_d455_power_rule() {
    section "d455 usb power rule"
    local rule_file="/etc/udev/rules.d/99-realsense-d455-power.rules"
    local rule_content
    rule_content='ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", RUN+="/usr/local/sbin/orkar-d455-power.sh"
ACTION=="change", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", RUN+="/usr/local/sbin/orkar-d455-power.sh"
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", TEST=="power/control", ATTR{power/control}:="on"
ACTION=="change", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", TEST=="power/control", ATTR{power/control}:="on"
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", TEST=="power/autosuspend_delay_ms", ATTR{power/autosuspend_delay_ms}:="-1"
ACTION=="change", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", TEST=="power/autosuspend_delay_ms", ATTR{power/autosuspend_delay_ms}:="-1"'
    sudo_write_file "${rule_file}" 0644 "${rule_content}"
    sudo_run udevadm control --reload-rules
    sudo_run udevadm trigger --subsystem-match=usb --attr-match=idVendor=8086 --attr-match=idProduct=0b5c || true
    echo "installed ${rule_file}"
}

install_d455_power_service() {
    section "d455 usb power service"
    local script_file="/usr/local/sbin/orkar-d455-power.sh"
    local service_file="/etc/systemd/system/orkar-d455-power.service"
    local script_content
    script_content='#!/usr/bin/env bash
set -euo pipefail
for _ in $(seq 1 30); do
  found=false
  for d in /sys/bus/usb/devices/*; do
    [ -f "${d}/idVendor" ] || continue
    [ -f "${d}/idProduct" ] || continue
    [ "$(cat "${d}/idVendor" 2>/dev/null)" = "8086" ] || continue
    [ "$(cat "${d}/idProduct" 2>/dev/null)" = "0b5c" ] || continue
    found=true
    [ -f "${d}/power/control" ] && echo on > "${d}/power/control" || true
    [ -f "${d}/power/autosuspend_delay_ms" ] && echo -1 > "${d}/power/autosuspend_delay_ms" || true
  done
  "${found}" && exit 0
  sleep 1
done
exit 0'
    local service_content
    service_content='[Unit]
Description=Disable runtime autosuspend for Intel RealSense D455
After=systemd-udev-settle.service
Wants=systemd-udev-settle.service

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/orkar-d455-power.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target'
    sudo_write_file "${script_file}" 0755 "${script_content}"
    sudo_write_file "${service_file}" 0644 "${service_content}"
    sudo_run systemctl daemon-reload
    sudo_run systemctl enable --now orkar-d455-power.service >/dev/null || true
    echo "installed ${service_file}"
}

install_d455_uvc_bind_rule() {
    section "d455 uvc bind rule"
    local rule_file="/etc/udev/rules.d/99-realsense-d455-uvc-bind.rules"
    local rule_content
    rule_content='# Force-bind Intel RealSense D455 video control interfaces to uvcvideo when the kernel leaves them unbound.
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_interface", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", ATTR{bInterfaceClass}=="0e", ATTR{bInterfaceSubClass}=="01", DRIVER=="", RUN+="/bin/sh -c '\''/sbin/modprobe uvcvideo; echo -n %k > /sys/bus/usb/drivers/uvcvideo/bind || true'\''"'
    sudo_write_file "${rule_file}" 0644 "${rule_content}"
    sudo_run modprobe uvcvideo || true
    sudo_run udevadm control --reload-rules
    sudo_run udevadm trigger --subsystem-match=usb --attr-match=idVendor=8086 --attr-match=idProduct=0b5c || true
    echo "installed ${rule_file}"
}

check_python_binding() {
    section "pyrealsense2 import"
    if python3 - <<'PY'
import pyrealsense2 as rs
print(rs.__version__ if hasattr(rs, "__version__") else "pyrealsense2 import ok")
PY
    then
        return
    fi
    echo "ERROR: pyrealsense2 import failed; dataset stream gate cannot run." >&2
    echo "       Install python3-pyrealsense2 for this architecture and rerun setup." >&2
    exit 1
}

install_ydlidar_sdk() {
    section "ydlidar sdk"
    local sdk_dir="${ROOT}/drivers/YDLidar-SDK"
    local jobs

    if [ ! -d "${sdk_dir}" ]; then
        echo "ERROR: missing vendored YDLidar SDK at ${sdk_dir}" >&2
        echo "       agv2_ws/src/ydlidar_ros2_driver requires ydlidar_sdk." >&2
        exit 1
    fi

    jobs="$(nproc 2>/dev/null || echo 2)"
    cmake -S "${sdk_dir}" -B "${sdk_dir}/build" -DCMAKE_BUILD_TYPE=Release
    cmake --build "${sdk_dir}/build" -j "${jobs}"
    sudo_run cmake --install "${sdk_dir}/build"
    sudo_run ldconfig || true

    if pkg-config --exists ydlidar_sdk; then
        echo "pkg-config ydlidar_sdk: $(pkg-config --modversion ydlidar_sdk)"
    else
        echo "WARN: ydlidar_sdk installed, but pkg-config did not find ydlidar_sdk.pc"
    fi
}

section "repo"
echo "root: ${ROOT}"
echo "robot: ${ROBOT_ID}"
echo "ros: ${ROS_DISTRO}"

if [ "${INSTALL_SYSTEM}" = "true" ]; then
    section "system dependencies"
    disable_legacy_realsense_sources
    if [ "${INSTALL_REALSENSE}" = "true" ]; then
        sudo_run rm -f /etc/apt/sources.list.d/librealsense.list
    fi
    sudo_run apt-get update
    apt_install \
        build-essential \
        chrony \
        cmake \
        git \
        i2c-tools \
        network-manager \
        pkg-config \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-yaml \
        usbutils \
        v4l-utils \
        "${ROS_DISTRO:+ros-${ROS_DISTRO}-cv-bridge}" \
        "${ROS_DISTRO:+ros-${ROS_DISTRO}-diagnostic-msgs}" \
        "${ROS_DISTRO:+ros-${ROS_DISTRO}-geometry-msgs}" \
        "${ROS_DISTRO:+ros-${ROS_DISTRO}-image-transport}" \
        "${ROS_DISTRO:+ros-${ROS_DISTRO}-nav-msgs}" \
        "${ROS_DISTRO:+ros-${ROS_DISTRO}-robot-state-publisher}" \
        "${ROS_DISTRO:+ros-${ROS_DISTRO}-rosbag2-storage-mcap}" \
        "${ROS_DISTRO:+ros-${ROS_DISTRO}-sensor-msgs}" \
        "${ROS_DISTRO:+ros-${ROS_DISTRO}-std-msgs}" \
        "${ROS_DISTRO:+ros-${ROS_DISTRO}-std-srvs}" \
        "${ROS_DISTRO:+ros-${ROS_DISTRO}-tf2-msgs}" \
        "${ROS_DISTRO:+ros-${ROS_DISTRO}-tf2-ros}" \
        "${ROS_DISTRO:+ros-${ROS_DISTRO}-xacro}"

    python3 -m pip install --user "mcap>=1.2,<2"

    if [ "${INSTALL_REALSENSE}" = "true" ]; then
        ensure_realsense_repo
        install_realsense_stack
        apt_install \
            "ros-${ROS_DISTRO}-realsense2-camera" \
            "ros-${ROS_DISTRO}-realsense2-description"
        check_python_binding
    fi
    sudo_run systemctl enable --now chrony >/dev/null 2>&1 || true
fi

if [ "${APPLY_LOW_RISK_FIXES}" = "true" ]; then
    install_d455_boot_quirk
    install_d455_power_service
    install_d455_power_rule
    install_d455_uvc_bind_rule
fi

if [ "${INSTALL_YDLIDAR_SDK}" = "true" ]; then
    install_ydlidar_sdk
fi

if [ "${BUILD_WS}" = "true" ]; then
    section "build agv2_ws"
    if [ ! -d "${ROOT}/agv2_ws/src" ]; then
        echo "ERROR: missing ROS 2 workspace: ${ROOT}/agv2_ws/src" >&2
        exit 1
    fi
    source_ros
    cd "${ROOT}/agv2_ws"
    colcon build --symlink-install
fi

section "script permissions"
chmod +x \
    "${ROOT}/scripts/diagnostics/"*.sh \
    "${ROOT}/scripts/diagnostics/"*.py \
    "${ROOT}/scripts/logging/"*.sh \
    "${ROOT}/scripts/logging/"*.py 2>/dev/null || true

if [ "${RUN_DOCTOR}" = "true" ]; then
    section "robot doctor static gate"
    cd "${ROOT}"
    python3 scripts/diagnostics/robot_doctor.py "${ROBOT_ID}" \
        --config configs/robot_doctor_dataset_gate.json \
        --profile static \
        --expected-d455-firmware "${EXPECTED_D455_FIRMWARE}" \
        --expected-librealsense "${EXPECTED_LIBREALSENSE}" \
        --expected-realsense-ros-driver "${EXPECTED_REALSENSE_ROS_DRIVER}" \
        --expected-realsense-ros-librealsense "${EXPECTED_REALSENSE_ROS_LIBREALSENSE}"
fi

section "next"
cat <<EOF
# Run a real sensor gate once the camera is connected and bringup is available:
cd ${ROOT}
bash scripts/diagnostics/robot_doctor.sh ${ROBOT_ID} \\
  --config configs/robot_doctor_dataset_gate.json \\
  --profile dataset \\
  --bringup-cmd "ros2 launch agv_bringup bringup.launch.py" \\
  --bringup-wait 90
EOF
