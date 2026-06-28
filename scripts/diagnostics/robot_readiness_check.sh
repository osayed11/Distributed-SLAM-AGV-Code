#!/usr/bin/env bash
# Run on the robot. Starts a bounded bringup test, samples rates/TF, then stops
# only the nodes it launched. This is a preflight gate before recording a bag.

ROOT="${HOME}/slam_project"
LOG="/tmp/agv_bringup_check_$(date +%Y%m%d_%H%M%S).log"

if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
else
    echo "ERROR: no supported ROS setup found under /opt/ros" >&2
    exit 1
fi

if [ -f "${ROOT}/myagv_ros/devel/setup.bash" ]; then
    source "${ROOT}/myagv_ros/devel/setup.bash"
fi
source "${ROOT}/agv_ws/devel/setup.bash"

set -u

print_section() {
    echo ""
    echo "== $1 =="
}

check_hz() {
    local topic="$1"
    local timeout_sec="${2:-12}"
    local window="${3:-20}"
    local out
    local line

    out=$(timeout "${timeout_sec}" rostopic hz "${topic}" --window "${window}" 2>&1 || true)
    line=$(printf "%s\n" "${out}" | grep "average rate" | tail -1 || true)
    if [ -n "${line}" ]; then
        echo "PASS ${topic}: ${line}"
    else
        echo "FAIL ${topic}: no average rate within ${timeout_sec}s"
        printf "%s\n" "${out}" | tail -5
    fi
}

check_topic_registered() {
    local topic="$1"
    if rostopic list 2>/dev/null | grep -qx "${topic}"; then
        echo "PASS ${topic}: registered"
        rostopic info "${topic}" 2>/dev/null | sed -n '1,8p'
    else
        echo "FAIL ${topic}: not registered"
    fi
}

check_optional_hz() {
    local topic="$1"
    local timeout_sec="${2:-8}"
    local window="${3:-20}"
    local out
    local line

    if ! rostopic list 2>/dev/null | grep -qx "${topic}"; then
        echo "INFO optional ${topic}: not registered"
        return
    fi

    out=$(timeout "${timeout_sec}" rostopic hz "${topic}" --window "${window}" 2>&1 || true)
    line=$(printf "%s\n" "${out}" | grep "average rate" | tail -1 || true)
    if [ -n "${line}" ]; then
        echo "PASS optional ${topic}: ${line}"
    else
        echo "INFO optional ${topic}: registered but no average rate within ${timeout_sec}s"
        printf "%s\n" "${out}" | tail -3
    fi
}

print_section "devices"
ls -l /dev/ttyACM0 /dev/ttyAMA0 2>/dev/null || true

print_section "usb"
lsusb -t || true

print_section "packages"
rospack find agv_bringup || true
rospack find realsense2_camera || true
rospack find apriltag_ros || true

print_section "stale ros before test"
pgrep -fal "roslaunch|rosmaster|roscore|realsense|ydlidar|myagv|rosbag|apriltag" || true

print_section "start bringup"
setsid roslaunch agv_bringup bringup.launch > "${LOG}" 2>&1 &
BRINGUP_PID=$!
echo "bringup_pid=${BRINGUP_PID}"
echo "bringup_log=${LOG}"

cleanup() {
    print_section "cleanup"
    rosnode kill /apriltag_ros_continuous_node 2>/dev/null || true
    if [ -n "${APRILTAG_PID:-}" ]; then
        kill "${APRILTAG_PID}" 2>/dev/null || true
    fi
    if [ -n "${BRINGUP_PID:-}" ]; then
        kill -INT "-${BRINGUP_PID}" 2>/dev/null || kill -INT "${BRINGUP_PID}" 2>/dev/null || true
    fi
    sleep 2
    rosnode kill /camera/realsense2_camera /camera/realsense2_camera_manager \
        /ydlidar_lidar_publisher /myagv_odometry_node \
        /base_to_camera_link /base_footprint_to_base_link 2>/dev/null || true
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
    echo "remaining_ros:"
    pgrep -fal "roslaunch|realsense|ydlidar|myagv|apriltag" || true
}
trap cleanup EXIT

sleep 35

print_section "ros nodes"
rosnode list 2>/dev/null | sort || true

print_section "core topic rates"
check_hz /scan 30 20
check_hz /odom 12 20
check_hz /tf 12 30
check_hz /camera/color/image_raw 12 20
check_hz /camera/aligned_depth_to_color/image_raw 12 20
    check_optional_hz /imu 8 10
    check_optional_hz /camera/imu 8 20
    check_optional_hz /camera/accel/sample 8 20
    check_optional_hz /camera/gyro/sample 8 20

print_section "tf checks"
timeout 8 rosrun tf tf_echo /base_footprint /base_link 2>&1 | sed -n '1,12p' || true
timeout 8 rosrun tf tf_echo /base_footprint /imu_link 2>&1 | sed -n '1,12p' || true
timeout 8 rosrun tf tf_echo /base_footprint /camera_link 2>&1 | sed -n '1,12p' || true
timeout 8 rosrun tf tf_echo /base_footprint /laser_frame 2>&1 | sed -n '1,12p' || true

print_section "mocap topics"
rostopic list 2>/dev/null | grep -Ei 'phase|mocap|ground|vrpn' || true
if rostopic list 2>/dev/null | grep -qx "/phasespace/rigids"; then
    check_hz /phasespace/rigids 10 20
fi
if rostopic list 2>/dev/null | grep -qx "/mocap"; then
    check_hz /mocap 10 20
fi

if [ "${CHECK_APRILTAG:-false}" = true ]; then
    print_section "apriltag live pipeline"
    roslaunch agv_bringup apriltag.launch > /tmp/agv_apriltag_check.log 2>&1 &
    APRILTAG_PID=$!
    sleep 8
    rosnode list 2>/dev/null | grep apriltag || true
    check_topic_registered /tag_detections
    check_hz /tag_detections 10 10
    echo "apriltag_log=/tmp/agv_apriltag_check.log"
else
    print_section "apriltag live pipeline"
    echo "SKIP live AprilTag detector. Set CHECK_APRILTAG=true for a separate detector smoke test."
fi

print_section "bringup log tail"
tail -80 "${LOG}" || true
