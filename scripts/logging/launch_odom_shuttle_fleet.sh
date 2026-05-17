#!/usr/bin/env bash
# Launch a fleet straight-line shuttle recording with odom-only controllers.
#
# Edit ROBOTS for the current lab network. The script starts robot-side logging
# with start_session.sh, waits until rosbag is live, then starts the odom-only
# shuttle controller on each robot.

set -euo pipefail

ROBOTS=("10.23.117.240" "10.23.31.157" "10.23.22.246" "10.23.74.155")
ROBOT_NAMES=("agv0" "agv1" "agv2" "agv3")

SCENARIO="${SCENARIO:-odom_shuttle}"
DISTANCE="${DISTANCE:-1.0}"
CYCLES="${CYCLES:-10}"
MAX_SPEED="${MAX_SPEED:-0.08}"
MAX_LATERAL="${MAX_LATERAL:-0.06}"
MAX_XY_COMMAND="${MAX_XY_COMMAND:-0.10}"
START_STAGGER="${START_STAGGER:-0}"
ARM_LEAD_SEC="${ARM_LEAD_SEC:-45}"
POST_ROLL_SEC="${POST_ROLL_SEC:-5}"
RECORD_WAIT_SEC="${RECORD_WAIT_SEC:-150}"
REQUIRE_CHRONY="${REQUIRE_CHRONY:-false}"
DISABLE_LATERAL="${DISABLE_LATERAL:-false}"

if [ "${#ROBOTS[@]}" -ne "${#ROBOT_NAMES[@]}" ]; then
    echo "ERROR: ROBOTS and ROBOT_NAMES must have the same length." >&2
    exit 2
fi

read -rsp "Password: " PASS
echo ""

BASE_START_EPOCH="$(($(date +%s) + ARM_LEAD_SEC))"
echo "=== Odom shuttle fleet plan ==="
echo "scenario:          ${SCENARIO}"
echo "robots:            ${#ROBOTS[@]}"
echo "distance_m:        ${DISTANCE}"
echo "cycles:            ${CYCLES}"
echo "max_speed:         ${MAX_SPEED}"
echo "max_lateral:       ${MAX_LATERAL}"
echo "base_start_epoch:  ${BASE_START_EPOCH}"
echo "start_stagger_s:   ${START_STAGGER}"
echo ""

remote_source_ros='
if [ -f /opt/ros/noetic/setup.bash ]; then source /opt/ros/noetic/setup.bash; fi
if [ -f /opt/ros/melodic/setup.bash ]; then source /opt/ros/melodic/setup.bash; fi
if [ -f ~/slam_project/agv_ws/devel/setup.bash ]; then source ~/slam_project/agv_ws/devel/setup.bash; fi
'

echo "=== Phase 1: start logging ==="
for i in "${!ROBOTS[@]}"; do
    ip="${ROBOTS[$i]}"
    name="${ROBOT_NAMES[$i]}"
    echo "[${name}] connecting to ${ip}"
    sshpass -p "${PASS}" ssh -n -o StrictHostKeyChecking=no -o ConnectTimeout=15 "ubuntu@${ip}" "
        set -e
        ${remote_source_ros}
        if command -v chronyc >/dev/null 2>&1; then
            chronyc tracking || true
            chronyc sources -v || true
            if [ '${REQUIRE_CHRONY}' = 'true' ]; then
                chronyc tracking | grep -q 'Leap status.*Normal'
                chronyc sources -v | grep -q '^\^\*'
            fi
        elif [ '${REQUIRE_CHRONY}' = 'true' ]; then
            echo 'ERROR: chronyc not installed' >&2
            exit 1
        fi
        export REQUIRE_IMU=false
        export REQUIRE_GT=false
        export SCENARIO='${SCENARIO}'
        log_file='/tmp/odom_shuttle_${name}.log'
        : > \"\${log_file}\"
        nohup bash ~/slam_project/scripts/logging/start_session.sh '${name}' '${SCENARIO}' > \"\${log_file}\" 2>&1 &
    " &
done
wait

echo "=== Phase 1.5: wait for bags ==="
HEALTHY=()
for i in "${!ROBOTS[@]}"; do
    ip="${ROBOTS[$i]}"
    name="${ROBOT_NAMES[$i]}"
    echo "[${name}] waiting for rosbag"
    if sshpass -p "${PASS}" ssh -n -o StrictHostKeyChecking=no -o ConnectTimeout=15 "ubuntu@${ip}" "
        timeout=${RECORD_WAIT_SEC}
        elapsed=0
        while ! pgrep -f 'rosbag record' >/dev/null 2>&1; do
            sleep 2
            elapsed=\$((elapsed + 2))
            if [ \${elapsed} -ge \${timeout} ]; then
                echo 'ERROR: timed out waiting for rosbag'
                tail -20 /tmp/odom_shuttle_${name}.log 2>/dev/null || true
                exit 1
            fi
        done
    "; then
        echo "[${name}] rosbag live"
        HEALTHY+=("${i}")
    else
        echo "[${name}] not ready; skipping drive"
    fi
done

if [ "${#HEALTHY[@]}" -eq 0 ]; then
    echo "ERROR: no robots are logging." >&2
    exit 1
fi

echo "=== Phase 2: start odom shuttle drives ==="
for i in "${HEALTHY[@]}"; do
    ip="${ROBOTS[$i]}"
    name="${ROBOT_NAMES[$i]}"
    start_delay="$((i * START_STAGGER))"
    lateral_flag=""
    if [ "${DISABLE_LATERAL}" = "true" ]; then
        lateral_flag="--disable-lateral"
    fi

    echo "[${name}] scheduled start delay ${start_delay}s"
    sshpass -p "${PASS}" ssh -n -t -t -o StrictHostKeyChecking=no "ubuntu@${ip}" "
        set -e
        ${remote_source_ros}
        python3 -u ~/slam_project/scripts/logging/drive_odom_shuttle.py \
            --distance '${DISTANCE}' \
            --cycles '${CYCLES}' \
            --max-speed '${MAX_SPEED}' \
            --max-lateral '${MAX_LATERAL}' \
            --max-xy-command '${MAX_XY_COMMAND}' \
            --start-at-epoch '${BASE_START_EPOCH}' \
            --start-delay '${start_delay}' \
            --no-prompt \
            --verbose \
            ${lateral_flag}
    " &
done
wait

echo "=== Phase 3: stop logging ==="
sleep "${POST_ROLL_SEC}"
for i in "${HEALTHY[@]}"; do
    ip="${ROBOTS[$i]}"
    name="${ROBOT_NAMES[$i]}"
    echo "[${name}] stopping rosbag"
    sshpass -p "${PASS}" ssh -n -t -t -o StrictHostKeyChecking=no "ubuntu@${ip}" "
        pkill -INT -f 'rosbag record' || true
        while pgrep -f '[s]tart_session.sh' >/dev/null 2>&1; do sleep 2; done
        sleep 2
    " &
done
wait

echo "Odom shuttle fleet run complete."
