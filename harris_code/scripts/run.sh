#!/usr/bin/env bash
# Single-command bring-up: starts the laptop GUI and SSHes into each robot
# in network.yaml to launch myagv_ros + drive_runner.
#
# Run from the harris_code/ directory (or pass --config explicitly):
#   ./scripts/run.sh
#   ./scripts/run.sh --mocap                      # also start mocap_pub
#   ./scripts/run.sh --push                       # rsync harris_code/ to each robot first
#   ./scripts/run.sh --no-robot                   # laptop side only
#   ./scripts/run.sh --stop                       # tear everything down
#
# Laptop tmux session  : drive-laptop
#   windows: control_panel, [mocap]
# Robot tmux session   : drive-robot
#   windows: ros, drive
#
# Attach laptop : tmux attach -t drive-laptop
# Attach robot  : ssh ubuntu@<ip> -t tmux attach -t drive-robot

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
HARRIS_DIR="$(cd "$HERE/.." && pwd)"
CONFIG="${CONFIG:-$HARRIS_DIR/config/network.yaml}"
PYTHON="${PYTHON:-python3}"
MOCAP_SERVER="${MOCAP_SERVER:-192.168.1.25}"
REMOTE_USER="${REMOTE_USER:-ubuntu}"
REMOTE_DIR="${REMOTE_DIR:-/home/ubuntu/harris_code}"
REMOTE_PYTHON="${REMOTE_PYTHON:-python3}"
LAPTOP_SESSION="drive-laptop"
ROBOT_SESSION="drive-robot"

DO_MOCAP=false
DO_PUSH=false
DO_ROBOT=true
DO_STOP=false

while [[ $# -gt 0 ]]; do
  case $1 in
    --mocap)     DO_MOCAP=true; shift ;;
    --push)      DO_PUSH=true; shift ;;
    --no-robot)  DO_ROBOT=false; shift ;;
    --stop)      DO_STOP=true; shift ;;
    --config)    CONFIG="$2"; shift 2 ;;
    *) echo "unknown arg: $1"; exit 1 ;;
  esac
done

# Parse robot ids + IPs from yaml.
mapfile -t ROBOT_IDS < <(python3 -c "
import yaml
with open('$CONFIG') as f:
    cfg = yaml.safe_load(f)
for r in cfg['robots']:
    print(r['id'])
")
mapfile -t ROBOT_IPS < <(python3 -c "
import yaml
with open('$CONFIG') as f:
    cfg = yaml.safe_load(f)
for r in cfg['robots']:
    print(r['ip'])
")

# ----------------------------------------------------------------------------
# Stop everything and exit
# ----------------------------------------------------------------------------
if $DO_STOP; then
  echo "stopping laptop session '$LAPTOP_SESSION'…"
  tmux kill-session -t "$LAPTOP_SESSION" 2>/dev/null || echo "  (not running)"
  if $DO_ROBOT; then
    for i in "${!ROBOT_IDS[@]}"; do
      IP="${ROBOT_IPS[$i]}"
      echo "stopping robot ${ROBOT_IDS[$i]} @ $IP…"
      ssh "$REMOTE_USER@$IP" "tmux kill-session -t $ROBOT_SESSION 2>/dev/null && echo '  stopped' || echo '  (not running)'"
    done
  fi
  exit 0
fi

# ----------------------------------------------------------------------------
# Robot side: push + launch
# ----------------------------------------------------------------------------
if $DO_ROBOT; then
  for i in "${!ROBOT_IDS[@]}"; do
    ID="${ROBOT_IDS[$i]}"
    IP="${ROBOT_IPS[$i]}"
    echo "==> robot $ID @ $IP"

    if $DO_PUSH; then
      echo "  [push] rsync $HARRIS_DIR -> $REMOTE_USER@$IP:$REMOTE_DIR"
      ssh "$REMOTE_USER@$IP" "mkdir -p $REMOTE_DIR"
      rsync -az --delete \
        --exclude="__pycache__" --exclude="scripts" --exclude="config" \
        "$HARRIS_DIR/" "$REMOTE_USER@$IP:$REMOTE_DIR/"
      scp "$CONFIG" "$REMOTE_USER@$IP:$REMOTE_DIR/network.yaml"
    fi

    echo "  [launch] starting tmux session '$ROBOT_SESSION'"
    ssh "$REMOTE_USER@$IP" bash <<EOF
      tmux kill-session -t $ROBOT_SESSION 2>/dev/null || true
      sudo -n pkill -f 'drive_runner' 2>/dev/null || true
      sleep 0.5
      tmux new-session -d -s $ROBOT_SESSION -n ros
      tmux send-keys -t $ROBOT_SESSION:ros \
        'source /home/ubuntu/myagv_ros/devel/setup.bash && roslaunch myagv_odometry myagv_active.launch' Enter
      sleep 2
      tmux new-window -t $ROBOT_SESSION -n drive
      tmux send-keys -t $ROBOT_SESSION:drive \
        'source /home/ubuntu/myagv_ros/devel/setup.bash && cd $REMOTE_DIR && $REMOTE_PYTHON drive_runner.py --config network.yaml --id $ID' Enter
EOF
    echo "  attach: ssh $REMOTE_USER@$IP -t tmux attach -t $ROBOT_SESSION"
  done
fi

# ----------------------------------------------------------------------------
# Laptop side: GUI (+ optional mocap)
# ----------------------------------------------------------------------------
tmux kill-session -t "$LAPTOP_SESSION" 2>/dev/null || true

tmux new-session -d -s "$LAPTOP_SESSION" -n control_panel
tmux send-keys -t "$LAPTOP_SESSION:control_panel" \
  "cd $HARRIS_DIR && $PYTHON control_panel.py --config $CONFIG" Enter

if $DO_MOCAP; then
  tmux new-window -t "$LAPTOP_SESSION" -n mocap
  tmux send-keys -t "$LAPTOP_SESSION:mocap" \
    "cd $HARRIS_DIR && $PYTHON mocap_pub.py --config $CONFIG --server $MOCAP_SERVER" Enter
fi

tmux select-window -t "$LAPTOP_SESSION:control_panel"

echo ""
echo "laptop session '$LAPTOP_SESSION' started."
echo "  control_panel: $PYTHON control_panel.py"
if $DO_MOCAP; then
  echo "  mocap:         $PYTHON mocap_pub.py --server $MOCAP_SERVER"
fi
echo ""
echo "attach: tmux attach -t $LAPTOP_SESSION"
echo "stop everything: $0 --stop"
