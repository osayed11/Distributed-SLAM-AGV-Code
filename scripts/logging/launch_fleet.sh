#!/bin/bash
# Fixed: Synchronized start and stop for consistent bag durations
ROBOTS=("10.23.118.99" "10.23.16.229" "10.23.37.117" "10.23.22.246")
RADII=("0.2" "0.4" "0.6" "0.8")
LINEAR="0.20"
DURATION="60.0"
STAGGER=30

read -sp "Password: " PASS
echo -e "\n"

echo "⚙️ PHASE 1: Initializing Hardware & Logging on all robots..."
for i in "${!ROBOTS[@]}"; do
    IP="${ROBOTS[$i]}"
    echo "🚀 [AGV$i] Connecting to $IP..."
    sshpass -p "$PASS" ssh -n -o StrictHostKeyChecking=no ubuntu@$IP "
        [ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
        [ -f /opt/ros/melodic/setup.bash ] && source /opt/ros/melodic/setup.bash
        [ -f ~/slam_project/myagv_ros/devel/setup.bash ] && source ~/slam_project/myagv_ros/devel/setup.bash
        [ -f ~/slam_project/agv_ws/devel/setup.bash ] && source ~/slam_project/agv_ws/devel/setup.bash
        export REQUIRE_IMU=true; export REQUIRE_GT=false;
        LOG_FILE=\"/tmp/bringup_agv$i.log\"; > \$LOG_FILE
        nohup bash ~/slam_project/scripts/logging/start_session.sh agv$i concentric > \$LOG_FILE 2>&1 &
        
        # Wait until the 'rosbag record' process is actually running
        until pgrep -f \"rosbag record\" > /dev/null; do sleep 1; done
    " &
done
wait

echo "🎯 PHASE 2: Commencing staggered drives..."
for i in "${!ROBOTS[@]}"; do
    IP="${ROBOTS[$i]}"
    RADIUS="${RADII[$i]}"
    DELAY=$((i * STAGGER))
    (
        [ $DELAY -gt 0 ] && echo "⏱️ [$(date +%T)] [AGV$i] Holding for ${DELAY}s..." && sleep $DELAY
        
        echo "🏎️ [$(date +%T)] [AGV$i] Starting Drive!"
        sshpass -p "$PASS" ssh -n -t -t -o StrictHostKeyChecking=no ubuntu@$IP "
            [ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
            [ -f /opt/ros/melodic/setup.bash ] && source /opt/ros/melodic/setup.bash
            [ -f ~/slam_project/myagv_ros/devel/setup.bash ] && source ~/slam_project/myagv_ros/devel/setup.bash
            [ -f ~/slam_project/agv_ws/devel/setup.bash ] && source ~/slam_project/agv_ws/devel/setup.bash
            
            python3 -u ~/slam_project/scripts/logging/drive_circle.py --radius $RADIUS --linear $LINEAR --duration $DURATION --no-prompt
            echo '🏁 [AGV$i] Drive complete. Standing by for synchronized stop...'
        "
        echo "🔒 [$(date +%T)] [AGV$i] Drive finished."
    ) &
done

wait

echo "⏸️ All drives complete. Allowing 5s for static loop closure baseline..."
sleep 5

echo "⚙️ PHASE 3: Synchronized Fleet Shutdown..."
for i in "${!ROBOTS[@]}"; do
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