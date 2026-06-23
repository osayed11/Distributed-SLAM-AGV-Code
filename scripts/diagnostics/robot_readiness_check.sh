#!/usr/bin/env bash
# Run on the robot. Starts a bounded bringup test, samples rates/TF, then stops
# only the nodes it launched. This is a preflight gate before recording a bag.

set -u

ROOT="${SLAM_PROJECT_ROOT:-${HOME}/slam_project}"
LOG="/tmp/agv_bringup_check_$(date +%Y%m%d_%H%M%S).log"
ROS_MAJOR=0
ROS_DISTRO_USED=""
FAILURES=0

REQUIRED_REALSENSE_FIRMWARE="${REQUIRED_REALSENSE_FIRMWARE:-5.17.0.10}"
REQUIRED_ROS_REALSENSE_CAMERA_VERSION="${REQUIRED_ROS_REALSENSE_CAMERA_VERSION:-4.57.7}"
REQUIRED_ROS_LIBREALSENSE_VERSION="${REQUIRED_ROS_LIBREALSENSE_VERSION:-2.57.7}"
REQUIRED_STANDALONE_LIBREALSENSE_VERSION="${REQUIRED_STANDALONE_LIBREALSENSE_VERSION:-2.58.1}"
MIN_CAMERA_IMU_HZ="${MIN_CAMERA_IMU_HZ:-150}"
MIN_CAMERA_GYRO_HZ="${MIN_CAMERA_GYRO_HZ:-150}"
MIN_CAMERA_ACCEL_HZ="${MIN_CAMERA_ACCEL_HZ:-90}"
MIN_RGBD_HZ="${MIN_RGBD_HZ:-12}"
MIN_REALSENSE_FPS="${MIN_REALSENSE_FPS:-15}"
RUN_RS_ENUMERATE="${RUN_RS_ENUMERATE:-false}"

source_ros_setup() {
    set +u
    source "$1"
    set -u
}

if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source_ros_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
    ROS_DISTRO_USED="${ROS_DISTRO}"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source_ros_setup /opt/ros/humble/setup.bash
    ROS_DISTRO_USED="humble"
elif [ -f /opt/ros/iron/setup.bash ]; then
    source_ros_setup /opt/ros/iron/setup.bash
    ROS_DISTRO_USED="iron"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source_ros_setup /opt/ros/jazzy/setup.bash
    ROS_DISTRO_USED="jazzy"
elif [ -f /opt/ros/noetic/setup.bash ]; then
    source_ros_setup /opt/ros/noetic/setup.bash
    ROS_DISTRO_USED="noetic"
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source_ros_setup /opt/ros/melodic/setup.bash
    ROS_DISTRO_USED="melodic"
else
    echo "ERROR: no supported ROS setup found under /opt/ros" >&2
    exit 1
fi

case "${ROS_DISTRO_USED}" in
    humble|iron|jazzy)
        ROS_MAJOR=2
        if [ -f "${ROOT}/agv2_ws/install/setup.bash" ]; then
            source_ros_setup "${ROOT}/agv2_ws/install/setup.bash"
        fi
        ;;
    noetic|melodic)
        ROS_MAJOR=1
        if [ -f "${ROOT}/myagv_ros/devel/setup.bash" ]; then
            source_ros_setup "${ROOT}/myagv_ros/devel/setup.bash"
        fi
        if [ -f "${ROOT}/agv_ws/devel/setup.bash" ]; then
            source_ros_setup "${ROOT}/agv_ws/devel/setup.bash"
        else
            echo "ERROR: missing ROS1 workspace setup: ${ROOT}/agv_ws/devel/setup.bash" >&2
            exit 1
        fi
        ;;
esac

print_section() {
    echo ""
    echo "== $1 =="
}

pass_gate() {
    echo "PASS $1: $2"
}

fail_gate() {
    echo "FAIL $1: $2"
    FAILURES=$((FAILURES + 1))
}

warn_gate() {
    echo "WARN $1: $2"
}

dpkg_version() {
    dpkg-query -W -f='${Version}' "$1" 2>/dev/null || true
}

check_dpkg_prefix() {
    local package="$1"
    local expected="$2"
    local label="$3"
    local version

    version="$(dpkg_version "${package}")"
    if [ -z "${version}" ]; then
        fail_gate "${label}" "${package} is not installed"
    elif [ "${version#${expected}}" != "${version}" ]; then
        pass_gate "${label}" "${package} ${version}"
    else
        fail_gate "${label}" "expected ${package} ${expected}*, found ${version}"
    fi
}

check_realsense_gate() {
    local usb_tree
    local rs_info
    local standalone_version

    print_section "realsense gate"

    usb_tree="$(lsusb -t 2>/dev/null || true)"
    if printf "%s\n" "${usb_tree}" | grep -q "Driver=uvcvideo, 5000M"; then
        pass_gate "RealSense USB" "D455 video interfaces are on USB 3.x / 5000M"
    else
        fail_gate "RealSense USB" "D455 is not visible as uvcvideo at 5000M"
    fi

    if [ "${RUN_RS_ENUMERATE}" != "true" ]; then
        warn_gate "rs-enumerate details" "skipped before live bringup; set RUN_RS_ENUMERATE=true only for control-path diagnostics"
    elif command -v rs-enumerate-devices >/dev/null 2>&1; then
        rs_info="$(timeout 15 rs-enumerate-devices 2>&1 || true)"
        printf "%s\n" "${rs_info}" | grep -Ei "Device Name|Firmware Version|Usb Type Descriptor|Imu Type|BMI085" || true

        if ! printf "%s\n" "${rs_info}" | grep -Eq "Firmware Version|Usb Type Descriptor|Imu Type"; then
            warn_gate "rs-enumerate details" "control-path details unavailable; live stream gates decide readiness"
        else
            if printf "%s\n" "${rs_info}" | grep -q "Firmware Version.*${REQUIRED_REALSENSE_FIRMWARE}"; then
                pass_gate "RealSense firmware" "${REQUIRED_REALSENSE_FIRMWARE}"
            else
                fail_gate "RealSense firmware" "expected ${REQUIRED_REALSENSE_FIRMWARE}"
            fi

            if printf "%s\n" "${rs_info}" | grep -Eq "Usb Type Descriptor.*3\\."; then
                pass_gate "RealSense USB descriptor" "USB 3.x"
            else
                fail_gate "RealSense USB descriptor" "expected USB 3.x from rs-enumerate-devices"
            fi

            if printf "%s\n" "${rs_info}" | grep -q "BMI085"; then
                pass_gate "RealSense IMU" "BMI085"
            else
                fail_gate "RealSense IMU" "expected BMI085"
            fi
        fi
    else
        fail_gate "RealSense tools" "rs-enumerate-devices is not installed"
    fi

    if [ "${ROS_MAJOR}" = "2" ]; then
        check_dpkg_prefix "ros-${ROS_DISTRO_USED}-realsense2-camera" \
            "${REQUIRED_ROS_REALSENSE_CAMERA_VERSION}" "ROS driver"
        check_dpkg_prefix "ros-${ROS_DISTRO_USED}-librealsense2" \
            "${REQUIRED_ROS_LIBREALSENSE_VERSION}" "ROS node LibRealSense package"
        if ldd /opt/ros/"${ROS_DISTRO_USED}"/lib/librealsense2_camera.so 2>/dev/null | \
            grep -q "/opt/ros/${ROS_DISTRO_USED}/lib.*/librealsense2.so.2.57"; then
            pass_gate "ROS node LibRealSense link" "using ROS ${REQUIRED_ROS_LIBREALSENSE_VERSION} library path"
        else
            fail_gate "ROS node LibRealSense link" "realsense2_camera is not linked to the ROS 2.57.x librealsense library"
        fi
    fi

    check_dpkg_prefix "librealsense2-utils" \
        "${REQUIRED_STANDALONE_LIBREALSENSE_VERSION}" "standalone librealsense tools"
    if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists realsense2; then
        standalone_version="$(pkg-config --modversion realsense2)"
        if [ "${standalone_version}" = "${REQUIRED_STANDALONE_LIBREALSENSE_VERSION}" ]; then
            pass_gate "standalone librealsense pkg-config" "${standalone_version}"
        else
            fail_gate "standalone librealsense pkg-config" "expected ${REQUIRED_STANDALONE_LIBREALSENSE_VERSION}, found ${standalone_version}"
        fi
    else
        fail_gate "standalone librealsense pkg-config" "realsense2 metadata missing"
    fi
}

write_sysfs_best_effort() {
    local path="$1"
    local value="$2"

    if [ -w "${path}" ]; then
        echo "${value}" > "${path}" 2>/dev/null || true
    elif sudo -n true >/dev/null 2>&1; then
        echo "${value}" | sudo -n tee "${path}" > /dev/null 2>&1 || true
    fi
}

reset_d455() {
    local rs_sysfs
    local bus
    local dev
    local devfile
    local rc

    print_section "D455 USB reset"

    rs_sysfs="$(for p in /sys/bus/usb/devices/*/idProduct; do
        [ "$(cat "$p" 2>/dev/null)" = "0b5c" ] && dirname "$p" && break
    done)"
    if [ -z "${rs_sysfs}" ]; then
        fail_gate "D455 USB reset" "D455 not found in sysfs"
        return
    fi

    bus="$(cat "${rs_sysfs}/busnum")"
    dev="$(cat "${rs_sysfs}/devnum")"
    devfile="$(printf "/dev/bus/usb/%03d/%03d" "${bus}" "${dev}")"
    if [ -w "${devfile}" ]; then
        python3 - "${devfile}" <<'PY'
import fcntl
import os
import sys
USBDEVFS_RESET = 0x5514
fd = os.open(sys.argv[1], os.O_WRONLY)
try:
    fcntl.ioctl(fd, USBDEVFS_RESET, 0)
finally:
    os.close(fd)
PY
    elif sudo -n true >/dev/null 2>&1; then
        sudo -n python3 - "${devfile}" <<'PY'
import fcntl
import os
import sys
USBDEVFS_RESET = 0x5514
fd = os.open(sys.argv[1], os.O_WRONLY)
try:
    fcntl.ioctl(fd, USBDEVFS_RESET, 0)
finally:
    os.close(fd)
PY
    else
        fail_gate "D455 USB reset" "no permission to reset ${devfile}"
        return
    fi
    rc=$?
    if [ "${rc}" -ne 0 ]; then
        fail_gate "D455 USB reset" "USBDEVFS_RESET failed for ${devfile}"
        return
    fi

    sleep 8
    rs_sysfs="$(for p in /sys/bus/usb/devices/*/idProduct; do
        [ "$(cat "$p" 2>/dev/null)" = "0b5c" ] && dirname "$p" && break
    done)"
    if [ -n "${rs_sysfs}" ]; then
        write_sysfs_best_effort "${rs_sysfs}/power/control" on
        write_sysfs_best_effort "${rs_sysfs}/power/autosuspend" -1
    fi
    pass_gate "D455 USB reset" "USB reset sent before live readiness bringup"
}

check_power_hardening() {
    local usb_autosuspend
    local d455_found=false
    local d
    local control
    local autosuspend
    local wifi_save

    print_section "power hardening"

    if [ -r /sys/module/usbcore/parameters/autosuspend ]; then
        usb_autosuspend="$(cat /sys/module/usbcore/parameters/autosuspend)"
        if [ "${usb_autosuspend}" = "-1" ]; then
            pass_gate "usbcore autosuspend" "disabled globally"
        else
            fail_gate "usbcore autosuspend" "current value ${usb_autosuspend}; run setup_robot.sh and reboot"
        fi
    else
        warn_gate "usbcore autosuspend" "could not read /sys/module/usbcore/parameters/autosuspend"
    fi

    if grep -Rqs "usbcore.autosuspend=-1" /boot/firmware/cmdline.txt /boot/cmdline.txt 2>/dev/null; then
        pass_gate "boot usb autosuspend" "usbcore.autosuspend=-1 present"
    else
        fail_gate "boot usb autosuspend" "usbcore.autosuspend=-1 missing from Pi boot cmdline"
    fi

    if systemctl is-enabled agv-realsense-power.service >/dev/null 2>&1; then
        pass_gate "D455 power service" "agv-realsense-power.service enabled"
    else
        fail_gate "D455 power service" "agv-realsense-power.service is not enabled"
    fi

    for p in /sys/bus/usb/devices/*/idProduct; do
        if [ "$(cat "$p" 2>/dev/null)" = "0b5c" ]; then
            d455_found=true
            d="$(dirname "$p")"
            echo "D455 sysfs: ${d}"
            for f in serial speed busnum devnum power/control power/autosuspend; do
                [ -e "${d}/${f}" ] && printf "  %s=" "${f}" && cat "${d}/${f}"
            done
            control="$(cat "${d}/power/control" 2>/dev/null || true)"
            autosuspend="$(cat "${d}/power/autosuspend" 2>/dev/null || true)"
            if [ "${control}" = "on" ]; then
                pass_gate "D455 power/control" "on"
            else
                fail_gate "D455 power/control" "expected on, found ${control:-unknown}"
            fi
            if [ "${autosuspend}" = "-1" ]; then
                pass_gate "D455 power/autosuspend" "-1"
            else
                fail_gate "D455 power/autosuspend" "expected -1, found ${autosuspend:-unknown}"
            fi
        fi
    done
    if [ "${d455_found}" = false ]; then
        fail_gate "D455 sysfs" "camera not found as USB product 0b5c"
    fi

    if [ -f /etc/NetworkManager/conf.d/90-agv-wifi-powersave-off.conf ] && \
       grep -Eq 'wifi\.powersave *= *2' /etc/NetworkManager/conf.d/90-agv-wifi-powersave-off.conf; then
        pass_gate "WiFi powersave config" "NetworkManager default disables powersave"
    else
        warn_gate "WiFi powersave config" "AGV NetworkManager powersave-off config missing"
    fi

    if command -v iw >/dev/null 2>&1 && iw dev wlan0 info >/dev/null 2>&1; then
        wifi_save="$(iw dev wlan0 get power_save 2>/dev/null || true)"
        if printf "%s\n" "${wifi_save}" | grep -qi "off"; then
            pass_gate "wlan0 power_save" "off"
        else
            fail_gate "wlan0 power_save" "${wifi_save:-unknown}; expected off"
        fi
    else
        warn_gate "wlan0 power_save" "iw missing or wlan0 unavailable"
    fi
}

validate_realsense_profile_floor() {
    local profile="$1"
    local label="$2"
    local fps

    fps="$(printf "%s" "${profile}" | awk -Fx '{print $3}')"
    if ! printf "%s" "${fps}" | grep -Eq '^[0-9]+([.][0-9]+)?$'; then
        fail_gate "${label} profile" "invalid profile '${profile}'"
        return
    fi
    if awk -v fps="${fps}" -v min="${MIN_REALSENSE_FPS}" 'BEGIN { exit(fps >= min ? 0 : 1) }'; then
        pass_gate "${label} profile" "${profile}"
    else
        fail_gate "${label} profile" "${profile}; hardware stream FPS must be >= ${MIN_REALSENSE_FPS}"
    fi
}

topic_list() {
    if [ "${ROS_MAJOR}" = "2" ]; then
        ros2 topic list 2>/dev/null
    else
        rostopic list 2>/dev/null
    fi
}

node_list() {
    if [ "${ROS_MAJOR}" = "2" ]; then
        ros2 node list 2>/dev/null
    else
        rosnode list 2>/dev/null
    fi
}

check_hz() {
    local topic="$1"
    local timeout_sec="${2:-12}"
    local window="${3:-20}"
    local min_rate="${4:-}"
    local out
    local line
    local rate

    if [ "${ROS_MAJOR}" = "2" ]; then
        out=$(timeout "${timeout_sec}" ros2 topic hz --window "${window}" "${topic}" 2>&1 || true)
    else
        out=$(timeout "${timeout_sec}" rostopic hz "${topic}" --window "${window}" 2>&1 || true)
    fi
    line=$(printf "%s\n" "${out}" | grep "average rate" | tail -1 || true)
    if [ -n "${line}" ]; then
        rate=$(printf "%s\n" "${line}" | awk -F': ' '{print $2}' | awk '{print $1}')
        if [ -n "${min_rate}" ] && ! awk -v rate="${rate}" -v min="${min_rate}" 'BEGIN { exit(rate >= min ? 0 : 1) }'; then
            fail_gate "${topic}" "${line}; expected >= ${min_rate} Hz"
        else
            pass_gate "${topic}" "${line}"
        fi
    else
        fail_gate "${topic}" "no average rate within ${timeout_sec}s"
        printf "%s\n" "${out}" | tail -5
    fi
}

check_topic_registered() {
    local topic="$1"
    if topic_list | grep -qx "${topic}"; then
        pass_gate "${topic}" "registered"
        if [ "${ROS_MAJOR}" = "2" ]; then
            ros2 topic info "${topic}" 2>/dev/null | sed -n '1,8p'
        else
            rostopic info "${topic}" 2>/dev/null | sed -n '1,8p'
        fi
    else
        fail_gate "${topic}" "not registered"
    fi
}

check_optional_hz() {
    local topic="$1"
    local timeout_sec="${2:-8}"
    local window="${3:-20}"
    local out
    local line

    if ! topic_list | grep -qx "${topic}"; then
        echo "INFO optional ${topic}: not registered"
        return
    fi

    if [ "${ROS_MAJOR}" = "2" ]; then
        out=$(timeout "${timeout_sec}" ros2 topic hz --window "${window}" "${topic}" 2>&1 || true)
    else
        out=$(timeout "${timeout_sec}" rostopic hz "${topic}" --window "${window}" 2>&1 || true)
    fi
    line=$(printf "%s\n" "${out}" | grep "average rate" | tail -1 || true)
    if [ -n "${line}" ]; then
        echo "PASS optional ${topic}: ${line}"
    else
        echo "INFO optional ${topic}: registered but no average rate within ${timeout_sec}s"
        printf "%s\n" "${out}" | tail -3
    fi
}

tf_echo() {
    local parent="$1"
    local child="$2"
    if [ "${ROS_MAJOR}" = "2" ]; then
        timeout 8 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>&1 | sed -n '1,12p' || true
    else
        timeout 8 rosrun tf tf_echo "${parent}" "${child}" 2>&1 | sed -n '1,12p' || true
    fi
}

print_section "ros"
echo "ros_distro=${ROS_DISTRO_USED}"
echo "ros_major=${ROS_MAJOR}"
echo "root=${ROOT}"

print_section "devices"
ls -l /dev/ttyACM0 /dev/myAGV /dev/ttyS0 /dev/ydlidar 2>/dev/null || true

print_section "usb"
lsusb -t || true
if command -v rs-enumerate-devices >/dev/null 2>&1; then
    echo ""
    timeout 15 rs-enumerate-devices 2>/dev/null | grep -E "Firmware Version|Usb Type Descriptor|Imu Type" || true
fi

check_realsense_gate
check_power_hardening

print_section "configured camera profiles"
validate_realsense_profile_floor "640x480x15" "default color"
validate_realsense_profile_floor "640x480x15" "default depth"

print_section "packages"
if [ "${ROS_MAJOR}" = "2" ]; then
    for pkg in agv_bringup realsense2_camera ydlidar_ros2_driver myagv_odometry; do
        ros2 pkg prefix "${pkg}" >/dev/null 2>&1 && echo "PASS ${pkg}" || echo "FAIL ${pkg}"
    done
    dpkg -l | grep -E "librealsense2|ros-${ROS_DISTRO_USED}-realsense2-camera" || true
else
    rospack find agv_bringup || true
    rospack find realsense2_camera || true
    rospack find apriltag_ros || true
fi

print_section "stale ros before test"
pgrep -fal "roslaunch|rosmaster|roscore|realsense|ydlidar|myagv|rosbag|apriltag|ros2 launch" || true

print_section "start bringup"
reset_d455
if [ "${ROS_MAJOR}" = "2" ]; then
    setsid ros2 launch agv_bringup bringup.launch.py > "${LOG}" 2>&1 &
else
    setsid roslaunch agv_bringup bringup.launch > "${LOG}" 2>&1 &
fi
BRINGUP_PID=$!
echo "bringup_pid=${BRINGUP_PID}"
echo "bringup_log=${LOG}"

cleanup() {
    print_section "cleanup"
    if [ -n "${BRINGUP_PID:-}" ]; then
        kill -INT "-${BRINGUP_PID}" 2>/dev/null || kill -INT "${BRINGUP_PID}" 2>/dev/null || true
    fi
    sleep 3
    if [ "${ROS_MAJOR}" = "1" ]; then
        rosnode kill /apriltag_ros_continuous_node 2>/dev/null || true
        rosnode kill /camera/realsense2_camera /camera/realsense2_camera_manager \
            /ydlidar_lidar_publisher /myagv_odometry_node \
            /base_to_camera_link /base_footprint_to_base_link 2>/dev/null || true
    fi
    if [ -n "${BRINGUP_PID:-}" ]; then
        kill -TERM "-${BRINGUP_PID}" 2>/dev/null || kill -TERM "${BRINGUP_PID}" 2>/dev/null || true
    fi
    for _ in 1 2 3 4 5 6 7 8; do
        if [ -z "${BRINGUP_PID:-}" ] || ! kill -0 "${BRINGUP_PID}" 2>/dev/null; then
            break
        fi
        sleep 1
    done
    if [ -n "${BRINGUP_PID:-}" ] && kill -0 "${BRINGUP_PID}" 2>/dev/null; then
        kill -KILL "-${BRINGUP_PID}" 2>/dev/null || kill -KILL "${BRINGUP_PID}" 2>/dev/null || true
        sleep 1
    fi
    pkill -TERM -f "realsense2_camera_node|ydlidar_ros2_driver_node|myagv_odometry_node" 2>/dev/null || true
    sleep 2
    pkill -KILL -f "realsense2_camera_node|ydlidar_ros2_driver_node|myagv_odometry_node" 2>/dev/null || true
    echo "remaining_ros:"
    pgrep -fal "roslaunch|rosmaster|roscore|realsense|ydlidar|myagv|apriltag|ros2 launch" || true
}
trap cleanup EXIT

sleep 35

print_section "ros nodes"
node_list | sort || true

print_section "registered topics"
check_topic_registered /scan
check_topic_registered /odom
check_topic_registered /tf
check_topic_registered /camera/color/image_raw
check_topic_registered /camera/aligned_depth_to_color/image_raw
check_topic_registered /camera/imu
check_topic_registered /camera/accel/sample
check_topic_registered /camera/gyro/sample

print_section "core topic rates"
check_hz /scan 30 20
check_hz /odom 12 20
check_hz /tf 12 30
check_hz /camera/color/image_raw 12 20 "${MIN_RGBD_HZ}"
check_hz /camera/aligned_depth_to_color/image_raw 12 20 "${MIN_RGBD_HZ}"
check_optional_hz /imu 8 10
check_hz /camera/imu 12 40 "${MIN_CAMERA_IMU_HZ}"
check_hz /camera/accel/sample 8 30 "${MIN_CAMERA_ACCEL_HZ}"
check_hz /camera/gyro/sample 8 30 "${MIN_CAMERA_GYRO_HZ}"

print_section "tf checks"
tf_echo base_footprint base_link
tf_echo base_footprint imu_link
tf_echo base_footprint camera_link
tf_echo base_footprint laser_frame

print_section "mocap topics"
topic_list | grep -Ei 'optitrack|mocap|ground|vrpn' || true

print_section "apriltag live pipeline"
echo "SKIP: AprilTag detector is optional and not launched by this readiness check."

print_section "bringup log tail"
tail -80 "${LOG}" || true

if grep -Eiq "The device has been disconnected|USB disconnect|No such device|device removed" "${LOG}" 2>/dev/null; then
    warn_gate "RealSense runtime log" "camera disconnect/device-drop text observed; topic-rate gates decide readiness"
elif grep -Eiq "UVCIOC_CTRL_QUERY|control_transfer.*failed|Connection timed out|Failed to create device|set_xu|Frames didn't arrived" "${LOG}" 2>/dev/null; then
    warn_gate "RealSense runtime log" "UVC/control timeout text observed; topic-rate gates decide readiness"
else
    pass_gate "RealSense runtime log" "no UVC/control timeout text in bringup log"
fi

print_section "readiness summary"
if [ "${FAILURES}" -eq 0 ]; then
    echo "READINESS PASS"
else
    echo "READINESS FAIL: ${FAILURES} gate failure(s)"
    exit 1
fi
