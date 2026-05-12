#!/bin/bash
# Fixed: Synchronized start and stop for consistent bag durations
ROBOTS=("10.23.118.99" "10.23.16.229" "10.23.22.246" "10.23.37.117" "10.23.33.237")
RADII=("0.25" "0.5" "0.75" "1" "1.25")
LINEAR="0.20"
DURATION="100.0"
STAGGER=15

read -sp "Password: " PASS
echo -e "\n"

echo "⚙️ PHASE 1: Initializing Hardware & Logging on all robots..."
for i in "${!ROBOTS[@]}"; do
    IP="${ROBOTS[$i]}"
    echo "🚀 [AGV$i] Connecting to $IP..."
    sshpass -p "$PASS" ssh -n -o StrictHostKeyChecking=no -o ConnectTimeout=15 ubuntu@$IP "
        [ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
        [ -f ~/slam_project/agv_ws/devel/setup.bash ] && source ~/slam_project/agv_ws/devel/setup.bash
        export REQUIRE_IMU=false; export REQUIRE_GT=false;
        LOG_FILE=\"/tmp/bringup_agv$i.log\"; > \$LOG_FILE
        nohup bash ~/slam_project/scripts/logging/start_session.sh agv$i concentric > \$LOG_FILE 2>&1 &

        # Wait for rosbag (up to 120s)
        TIMEOUT=120; ELAPSED=0
        until pgrep -f \"rosbag record\" > /dev/null 2>&1; do
            sleep 2; ELAPSED=\$((ELAPSED + 2))
            if [ \$ELAPSED -ge \$TIMEOUT ]; then
                echo '[AGV$i] TIMEOUT waiting for rosbag. Last log lines:'
                tail -5 \$LOG_FILE
                exit 1
            fi
            # Show progress every 20s
            if [ \$((ELAPSED % 20)) -eq 0 ]; then
                echo '[AGV$i] Still waiting... (\${ELAPSED}s) last log:'
                tail -1 \$LOG_FILE
            fi
        done
        echo '[AGV$i] rosbag is live.'
    " &
done
wait

echo ""
echo "🔍 PHASE 1.5: Verifying fleet health..."
HEALTHY=()
FAILED=()
for i in "${!ROBOTS[@]}"; do
    IP="${ROBOTS[$i]}"
    if sshpass -p "$PASS" ssh -n -o StrictHostKeyChecking=no -o ConnectTimeout=10 ubuntu@$IP \
        "pgrep -f 'rosbag record' > /dev/null 2>&1" 2>/dev/null; then
        echo "  ✅ [AGV$i] rosbag running"
        HEALTHY+=($i)
    else
        echo "  ❌ [AGV$i] rosbag NOT running — skipping in Phase 2"
        # Show last few log lines for debugging
        sshpass -p "$PASS" ssh -n -o StrictHostKeyChecking=no -o ConnectTimeout=5 ubuntu@$IP \
            "tail -5 /tmp/bringup_agv$i.log 2>/dev/null" 2>/dev/null || true
        FAILED+=($i)
    fi
done

if [ ${#HEALTHY[@]} -eq 0 ]; then
    echo "❌ No healthy robots. Aborting."
    exit 1
fi
echo "  Fleet: ${#HEALTHY[@]}/${#ROBOTS[@]} robots ready."
echo ""

echo "🎯 PHASE 2: Commencing staggered drives..."
for i in "${HEALTHY[@]}"; do
    IP="${ROBOTS[$i]}"
    RADIUS="${RADII[$i]}"
    DELAY=$((i * STAGGER))
    (
        [ $DELAY -gt 0 ] && echo "⏱️ [$(date +%T)] [AGV$i] Holding for ${DELAY}s..." && sleep $DELAY
        
        echo "🏎️ [$(date +%T)] [AGV$i] Starting Drive!"
        sshpass -p "$PASS" ssh -n -t -t -o StrictHostKeyChecking=no ubuntu@$IP "
            [ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
            [ -f ~/slam_project/agv_ws/devel/setup.bash ] && source ~/slam_project/agv_ws/devel/setup.bash
            
            python3 -u ~/slam_project/scripts/logging/drive_circle.py --radius $RADIUS --linear $LINEAR --duration $DURATION --counter-clockwise --no-prompt
            echo '🏁 [AGV$i] Drive complete. Standing by for synchronized stop...'
        "
        echo "🔒 [$(date +%T)] [AGV$i] Drive finished."
    ) &
done

wait

echo "⏸️ All drives complete. Allowing 5s for static loop closure baseline..."
sleep 5

echo "⚙️ PHASE 3: Synchronized Fleet Shutdown..."
for i in "${HEALTHY[@]}"; do
    IP="${ROBOTS[$i]}"
    (
        echo "🛑 [AGV$i] Triggering graceful save..."
        sshpass -p "$PASS" ssh -n -t -t -o StrictHostKeyChecking=no ubuntu@$IP "
            LOG_FILE=\"/tmp/bringup_agv$i.log\"
            tail -n0 -f \$LOG_FILE | while read line; do echo \"[AGV$i SHUTDOWN] \$line\"; done &
            TAIL_PID=\$!
            pkill -INT -f \"rosbag record\" || true
            while pgrep -f \"[s]tart_session.sh\" > /dev/null; do sleep 2; done
            sleep 2
            kill \$TAIL_PID 2>/dev/null || true
        "
        echo "✅ [AGV$i] Logging successfully saved!"
    ) &
done

wait
echo "🏁 Mission Complete for the entire fleet."