#!/usr/bin/env bash
# Apply narrowly-scoped fixes for robot_doctor findings.
#
# Default mode is dry-run. Use --apply to change the robot.
#
# Supported fixes:
#   d455-autosuspend            Persistently disables USB autosuspend for Intel RealSense D455.
#   d455-uvc-bind               Force-binds unbound D455 video interfaces to uvcvideo.
#   d455-usb-reset              Sends USBDEVFS_RESET to the connected D455.
#   d455-authorize-cycle        Deauthorizes/reauthorizes the D455 USB device to recover wedged UVC binding.
#   realsense-standalone-tools  Installs/pins standalone Intel librealsense tools.

set -euo pipefail

APPLY=false
FIXES=()

usage() {
    sed -n '1,12p' "$0"
    cat <<'EOF'

Usage:
  bash scripts/diagnostics/apply_robot_doctor_fix.sh --fix d455-autosuspend
  bash scripts/diagnostics/apply_robot_doctor_fix.sh --fix d455-uvc-bind
  bash scripts/diagnostics/apply_robot_doctor_fix.sh --fix d455-usb-reset
  bash scripts/diagnostics/apply_robot_doctor_fix.sh --fix d455-authorize-cycle
  bash scripts/diagnostics/apply_robot_doctor_fix.sh --fix realsense-standalone-tools
  bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix d455-autosuspend
  bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix d455-uvc-bind
  bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix d455-usb-reset
  bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix d455-authorize-cycle
  bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix realsense-standalone-tools

For non-interactive SSH automation:
  SUDO_PASSWORD="$ROBOT_SUDO_PASSWORD" bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix <fix-name>

EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --apply)
            APPLY=true
            ;;
        --fix)
            shift
            if [ "${1:-}" = "" ]; then
                echo "ERROR: --fix requires a value" >&2
                exit 2
            fi
            FIXES+=("$1")
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

if [ "${#FIXES[@]}" -eq 0 ]; then
    echo "ERROR: no fix selected" >&2
    usage >&2
    exit 2
fi

log() {
    echo "[$(date --iso-8601=seconds 2>/dev/null || date)] $*"
}

run_or_show() {
    if [ "${APPLY}" = true ]; then
        log "RUN: $*"
        bash -lc "$*"
    else
        log "DRY-RUN: $*"
    fi
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
    path="$1"
    mode="$2"
    content="$3"
    tmp="$(mktemp)"
    printf '%s\n' "${content}" > "${tmp}"
    sudo_run install -m "${mode}" "${tmp}" "${path}"
    rm -f "${tmp}"
}

sudo_write_sysfs() {
    path="$1"
    value="$2"
    if [ "${APPLY}" = true ]; then
        log "RUN: write '${value}' to ${path}"
        if [ "$(id -u)" -eq 0 ]; then
            printf '%s\n' "${value}" > "${path}"
        else
            sudo_run bash -c 'printf "%s\n" "$1" > "$2"' _ "${value}" "${path}"
        fi
    else
        log "DRY-RUN: write '${value}' to ${path}"
    fi
}

install_d455_boot_quirk() {
    cmdline_file="/boot/firmware/cmdline.txt"
    if [ ! -f "${cmdline_file}" ]; then
        cmdline_file="/boot/cmdline.txt"
    fi
    if [ ! -f "${cmdline_file}" ]; then
        log "WARN: boot cmdline file not found; cannot install usbcore D455 quirk"
        return
    fi
    if grep -qw "usbcore.quirks=8086:0b5c:kn" "${cmdline_file}"; then
        log "usbcore.quirks=8086:0b5c:kn already present in ${cmdline_file}"
        return
    fi
    if [ "${APPLY}" = true ]; then
        sudo_run cp "${cmdline_file}" "${cmdline_file}.bak.$(date +%Y%m%d_%H%M%S)"
        sudo_run python3 -c '
from pathlib import Path
import sys
path = Path(sys.argv[1])
parts = [item for item in path.read_text().strip().split() if not item.startswith("usbcore.quirks=")]
parts.append("usbcore.quirks=8086:0b5c:kn")
path.write_text(" ".join(parts) + "\n")
' "${cmdline_file}"
        log "installed usbcore.quirks=8086:0b5c:kn in ${cmdline_file}; reboot required"
    else
        log "DRY-RUN: add usbcore.quirks=8086:0b5c:kn to ${cmdline_file}"
    fi
}

install_d455_power_service() {
    script_file="/usr/local/sbin/orkar-d455-power.sh"
    service_file="/etc/systemd/system/orkar-d455-power.service"
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
    if [ "${APPLY}" = true ]; then
        sudo_write_file "${script_file}" 0755 "${script_content}"
        sudo_write_file "${service_file}" 0644 "${service_content}"
        sudo_run systemctl daemon-reload
        sudo_run systemctl enable --now orkar-d455-power.service >/dev/null || true
        log "installed ${service_file}"
    else
        log "DRY-RUN: install ${script_file} and ${service_file}"
    fi
}

find_d455_sysfs() {
    for d in /sys/bus/usb/devices/*; do
        [ -f "${d}/idVendor" ] || continue
        [ -f "${d}/idProduct" ] || continue
        if [ "$(cat "${d}/idVendor" 2>/dev/null)" = "8086" ] && \
           [ "$(cat "${d}/idProduct" 2>/dev/null)" = "0b5c" ]; then
            printf '%s\n' "${d}"
        fi
    done
}

fix_d455_autosuspend() {
    log "Fix: d455-autosuspend"
    install_d455_boot_quirk
    mapfile -t d455_devices < <(find_d455_sysfs)
    if [ "${#d455_devices[@]}" -eq 0 ]; then
        echo "ERROR: no D455 found in /sys/bus/usb/devices (8086:0b5c)." >&2
        echo "       Run robot_doctor first; if d455_enumeration fails, fix cable/port/power before autosuspend." >&2
        exit 1
    fi

    rule_file="/etc/udev/rules.d/99-realsense-d455-power.rules"
    rule_content='ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", RUN+="/usr/local/sbin/orkar-d455-power.sh"
ACTION=="change", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", RUN+="/usr/local/sbin/orkar-d455-power.sh"
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", TEST=="power/control", ATTR{power/control}:="on"
ACTION=="change", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", TEST=="power/control", ATTR{power/control}:="on"
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", TEST=="power/autosuspend_delay_ms", ATTR{power/autosuspend_delay_ms}:="-1"
ACTION=="change", SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", TEST=="power/autosuspend_delay_ms", ATTR{power/autosuspend_delay_ms}:="-1"'

    if [ "${APPLY}" = true ]; then
        sudo_write_file "${rule_file}" 0644 "${rule_content}"
        sudo_run udevadm control --reload-rules
        install_d455_power_service
    else
        log "Would write ${rule_file}:"
        printf '%s\n' "${rule_content}"
        install_d455_power_service
    fi

    for d in "${d455_devices[@]}"; do
        if [ -f "${d}/power/control" ]; then
            sudo_write_sysfs "${d}/power/control" "on"
            current="$(cat "${d}/power/control" 2>/dev/null || true)"
            log "D455 ${d} power/control=${current:-unknown}"
        else
            log "D455 ${d} has no power/control file; skipping live power policy update"
        fi
        if [ -f "${d}/power/autosuspend_delay_ms" ]; then
            sudo_write_sysfs "${d}/power/autosuspend_delay_ms" "-1"
            delay="$(cat "${d}/power/autosuspend_delay_ms" 2>/dev/null || true)"
            log "D455 ${d} power/autosuspend_delay_ms=${delay:-unknown}"
        else
            log "D455 ${d} has no power/autosuspend_delay_ms file; skipping live delay update"
        fi
    done

    if [ "${APPLY}" = true ]; then
        log "RUN: sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=8086 --attr-match=idProduct=0b5c"
        sudo_run udevadm trigger --subsystem-match=usb --attr-match=idVendor=8086 --attr-match=idProduct=0b5c || true
    else
        log "DRY-RUN: sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=8086 --attr-match=idProduct=0b5c || true"
    fi

    log "Done. Re-run robot_doctor and expect PASS for 2.1 d455_usb_autosuspend."
}

fix_d455_uvc_bind() {
    log "Fix: d455-uvc-bind"
    mapfile -t d455_devices < <(find_d455_sysfs)
    if [ "${#d455_devices[@]}" -eq 0 ]; then
        echo "ERROR: no D455 found in /sys/bus/usb/devices (8086:0b5c)." >&2
        echo "       If lsusb also cannot see the camera, fix cable/port/power first." >&2
        exit 1
    fi

    rule_file="/etc/udev/rules.d/99-realsense-d455-uvc-bind.rules"
    rule_content='# Force-bind Intel RealSense D455 video control interfaces to uvcvideo when the kernel leaves them unbound.
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_interface", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", ATTR{bInterfaceClass}=="0e", ATTR{bInterfaceSubClass}=="01", DRIVER=="", RUN+="/bin/sh -c '\''/sbin/modprobe uvcvideo; echo -n %k > /sys/bus/usb/drivers/uvcvideo/bind || true'\''"'

    if [ "${APPLY}" = true ]; then
        sudo_write_file "${rule_file}" 0644 "${rule_content}"
        sudo_run modprobe uvcvideo || true
        sudo_run udevadm control --reload-rules
    else
        log "Would write ${rule_file}:"
        printf '%s\n' "${rule_content}"
    fi

    for d in "${d455_devices[@]}"; do
        for iface in "${d}":*; do
            [ -d "${iface}" ] || continue
            [ -f "${iface}/bInterfaceClass" ] || continue
            [ "$(cat "${iface}/bInterfaceClass" 2>/dev/null)" = "0e" ] || continue
            [ "$(cat "${iface}/bInterfaceSubClass" 2>/dev/null)" = "01" ] || continue
            iface_name="$(basename "${iface}")"
            if [ -L "${iface}/driver" ]; then
                log "${iface_name} already bound to $(basename "$(readlink "${iface}/driver")")"
                continue
            fi
            if [ "${APPLY}" = true ]; then
                log "RUN: bind ${iface_name} to uvcvideo"
                sudo_run bash -c 'printf "%s" "$1" > /sys/bus/usb/drivers/uvcvideo/bind' _ "${iface_name}" || true
            else
                log "DRY-RUN: bind ${iface_name} to uvcvideo"
            fi
        done
    done

    if [ "${APPLY}" = true ]; then
        sleep 2
        log "D455 USB tree after bind:"
        lsusb -t | sed -n '1,80p'
        if command -v rs-enumerate-devices >/dev/null 2>&1; then
            timeout 20 rs-enumerate-devices -s 2>&1 || true
        fi
    fi

    log "Done. Re-run robot_doctor and expect PASS for 2.1 d455_uvc_binding and 2.1 d455_rs_enumerate."
}

fix_d455_usb_reset() {
    log "Fix: d455-usb-reset"
    mapfile -t d455_devices < <(find_d455_sysfs)
    if [ "${#d455_devices[@]}" -eq 0 ]; then
        echo "ERROR: no D455 found in /sys/bus/usb/devices (8086:0b5c)." >&2
        echo "       If lsusb also cannot see the camera, fix cable/port/power first." >&2
        exit 1
    fi

    reset_script="$(mktemp)"
    cat > "${reset_script}" <<'PY'
import fcntl
import os
import sys

USBDEVFS_RESET = 0x5514
path = sys.argv[1]
fd = os.open(path, os.O_WRONLY)
try:
    fcntl.ioctl(fd, USBDEVFS_RESET, 0)
finally:
    os.close(fd)
print(f"reset {path}")
PY

    for d in "${d455_devices[@]}"; do
        bus="$(cat "${d}/busnum")"
        dev="$(cat "${d}/devnum")"
        devfile="$(printf "/dev/bus/usb/%03d/%03d" "${bus}" "${dev}")"
        if [ "${APPLY}" = true ]; then
            log "RUN: USBDEVFS_RESET ${devfile} (timeout 20s)"
            if ! sudo_run timeout 20 python3 "${reset_script}" "${devfile}"; then
                rm -f "${reset_script}"
                echo "ERROR: USBDEVFS_RESET failed or timed out for ${devfile}." >&2
                echo "       Treat this as USB/kernel/control-path evidence; power-cycle or move to cable/port/camera A/B swap." >&2
                exit 1
            fi
        else
            log "DRY-RUN: USBDEVFS_RESET ${devfile}"
        fi
    done
    rm -f "${reset_script}"

    if [ "${APPLY}" = true ]; then
        sleep 8
        mapfile -t d455_devices_after < <(find_d455_sysfs)
        for d in "${d455_devices_after[@]}"; do
            if [ -f "${d}/power/control" ]; then
                sudo_write_sysfs "${d}/power/control" "on"
                log "D455 ${d} power/control=$(cat "${d}/power/control" 2>/dev/null || true)"
            fi
            if [ -f "${d}/power/autosuspend_delay_ms" ]; then
                sudo_write_sysfs "${d}/power/autosuspend_delay_ms" "-1"
                log "D455 ${d} power/autosuspend_delay_ms=$(cat "${d}/power/autosuspend_delay_ms" 2>/dev/null || true)"
            fi
        done
    fi

    log "Done. Re-run robot_doctor and expect rs-enumerate/control-query evidence to improve if the fault was transient."
}

fix_d455_authorize_cycle() {
    log "Fix: d455-authorize-cycle"
    mapfile -t d455_devices < <(find_d455_sysfs)
    if [ "${#d455_devices[@]}" -eq 0 ]; then
        echo "ERROR: no D455 found in /sys/bus/usb/devices (8086:0b5c)." >&2
        echo "       If lsusb also cannot see the camera, fix cable/port/power first." >&2
        exit 1
    fi

    for d in "${d455_devices[@]}"; do
        if [ ! -f "${d}/authorized" ]; then
            echo "ERROR: ${d}/authorized is missing; cannot cycle USB authorization." >&2
            exit 1
        fi
        if [ "${APPLY}" = true ]; then
            log "RUN: deauthorize ${d}"
            sudo_write_sysfs "${d}/authorized" "0"
            sleep 4
            log "RUN: reauthorize ${d}"
            sudo_write_sysfs "${d}/authorized" "1"
        else
            log "DRY-RUN: write 0 then 1 to ${d}/authorized"
        fi
    done

    if [ "${APPLY}" = true ]; then
        sleep 8
        mapfile -t d455_devices_after < <(find_d455_sysfs)
        for d in "${d455_devices_after[@]}"; do
            if [ -f "${d}/power/control" ]; then
                sudo_write_sysfs "${d}/power/control" "on"
                log "D455 ${d} power/control=$(cat "${d}/power/control" 2>/dev/null || true)"
            fi
            if [ -f "${d}/power/autosuspend_delay_ms" ]; then
                sudo_write_sysfs "${d}/power/autosuspend_delay_ms" "-1"
                log "D455 ${d} power/autosuspend_delay_ms=$(cat "${d}/power/autosuspend_delay_ms" 2>/dev/null || true)"
            fi
        done
        log "D455 USB tree after authorize cycle:"
        lsusb -t | sed -n '1,80p'
        if command -v rs-enumerate-devices >/dev/null 2>&1; then
            timeout 20 rs-enumerate-devices -s 2>&1 || true
        fi
    fi

    log "Done. Re-run robot_doctor and expect rs-enumerate and d455_uvc_binding to recover if the fault was a wedged USB authorization state."
}

fix_realsense_standalone_tools() {
    log "Fix: realsense-standalone-tools"
    version="${REALSENSE_APT_VERSION:-2.58.1-0~realsense.8235}"
    pyrealsense_version="${PYREALSENSE2_PIP_VERSION:-2.58.1.10581}"
    packages=(
        librealsense2
        librealsense2-utils
        librealsense2-dev
        librealsense2-udev-rules
        librealsense2-gl
    )

    if ! command -v apt-cache >/dev/null 2>&1; then
        echo "ERROR: apt-cache not found; this fix supports Debian/Ubuntu robots only." >&2
        exit 1
    fi

    if [ "${APPLY}" = true ]; then
        log "RUN: sudo apt-get update"
        sudo_run apt-get update
    else
        log "DRY-RUN: sudo apt-get update"
    fi

    missing=()
    for pkg in "${packages[@]}"; do
        if ! apt-cache policy "${pkg}" 2>/dev/null | grep -Fq "${version}"; then
            missing+=("${pkg}")
        fi
    done
    if [ "${#missing[@]}" -gt 0 ]; then
        echo "ERROR: expected RealSense apt version ${version} is not available for: ${missing[*]}" >&2
        echo "       Ensure the Intel RealSense apt source is configured, then rerun this fix." >&2
        exit 1
    fi

    install_args=()
    for pkg in "${packages[@]}"; do
        install_args+=("${pkg}=${version}")
    done

    if [ "${APPLY}" = true ]; then
        log "RUN: sudo apt-get install pinned RealSense packages (${version})"
        sudo_run env DEBIAN_FRONTEND=noninteractive apt-get install -y --allow-downgrades "${install_args[@]}"
        log "RUN: sudo apt-mark hold ${packages[*]}"
        sudo_run apt-mark hold "${packages[@]}"
    else
        log "DRY-RUN: sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y --allow-downgrades ${install_args[*]}"
        log "DRY-RUN: sudo apt-mark hold ${packages[*]}"
    fi

    if python3 - <<'PY' >/dev/null 2>&1
import pyrealsense2
PY
    then
        log "pyrealsense2 import already works"
    elif python3 -m pip --version >/dev/null 2>&1; then
        if [ "${APPLY}" = true ]; then
            log "RUN: python3 -m pip install --user pyrealsense2==${pyrealsense_version}"
            python3 -m pip install --user "pyrealsense2==${pyrealsense_version}"
        else
            log "DRY-RUN: python3 -m pip install --user pyrealsense2==${pyrealsense_version}"
        fi
    else
        echo "ERROR: pyrealsense2 is missing and python3 -m pip is unavailable." >&2
        echo "       Install python3-pip, then rerun this fix before using the dataset stream gate." >&2
        exit 1
    fi

    if command -v rs-enumerate-devices >/dev/null 2>&1; then
        log "rs-enumerate-devices: $(command -v rs-enumerate-devices)"
    else
        log "rs-enumerate-devices is not on PATH yet"
    fi
    if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists realsense2; then
        log "pkg-config realsense2: $(pkg-config --modversion realsense2)"
    else
        log "realsense2 pkg-config metadata not visible yet"
    fi
    if python3 - <<'PY'
import pyrealsense2 as rs
print(getattr(rs, "__version__", "pyrealsense2 import ok"))
PY
    then
        :
    elif [ "${APPLY}" = true ]; then
        echo "ERROR: pyrealsense2 import still fails after install." >&2
        exit 1
    fi

    log "Done. Re-run robot_doctor and expect PASS for 2.2 realsense_tools and 2.2 librealsense_version."
}

for fix in "${FIXES[@]}"; do
    case "${fix}" in
        d455-autosuspend)
            fix_d455_autosuspend
            ;;
        d455-uvc-bind)
            fix_d455_uvc_bind
            ;;
        d455-usb-reset)
            fix_d455_usb_reset
            ;;
        d455-authorize-cycle)
            fix_d455_authorize_cycle
            ;;
        realsense-standalone-tools)
            fix_realsense_standalone_tools
            ;;
        *)
            echo "ERROR: unsupported fix: ${fix}" >&2
            exit 2
            ;;
    esac
done
