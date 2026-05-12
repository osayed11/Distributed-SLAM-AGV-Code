#!/usr/bin/env bash
# Install Python dependencies on the laptop and on every robot listed in
# network.yaml. Run from the harris_code/ directory:
#
#   ./scripts/install_deps.sh                 # laptop + all robots
#   ./scripts/install_deps.sh --laptop-only
#   ./scripts/install_deps.sh --robots-only
#   ./scripts/install_deps.sh --config path/to/network.yaml
#
# rospy / nav_msgs / geometry_msgs come from the existing ROS Noetic install
# on the myAGV (sourced via setup.bash at run time) — not installed here.

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
HARRIS_DIR="$(cd "$HERE/.." && pwd)"
CONFIG="${CONFIG:-$HARRIS_DIR/config/network.yaml}"
PYTHON="${PYTHON:-python3}"
REMOTE_USER="${REMOTE_USER:-ubuntu}"
REMOTE_PYTHON="${REMOTE_PYTHON:-python3}"

LAPTOP_PKGS=(pyyaml pyzmq numpy matplotlib msgpack)
ROBOT_PKGS=(pyyaml pyzmq numpy msgpack)

DO_LAPTOP=true
DO_ROBOTS=true

while [[ $# -gt 0 ]]; do
  case $1 in
    --laptop-only) DO_ROBOTS=false; shift ;;
    --robots-only) DO_LAPTOP=false; shift ;;
    --config)      CONFIG="$2"; shift 2 ;;
    *) echo "unknown arg: $1"; exit 1 ;;
  esac
done

# ----------------------------------------------------------------------------
# Laptop
# ----------------------------------------------------------------------------
if $DO_LAPTOP; then
  echo "==> laptop: installing ${LAPTOP_PKGS[*]}"
  "$PYTHON" -m pip install --upgrade "${LAPTOP_PKGS[@]}"

  echo "    checking tkinter for matplotlib TkAgg backend…"
  if "$PYTHON" -c "import tkinter" 2>/dev/null; then
    echo "    tkinter OK"
  else
    echo "    tkinter MISSING — control_panel will fail to open."
    if [[ "$(uname)" == "Darwin" ]]; then
      PY_VER=$("$PYTHON" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
      echo "    fix on macOS:  brew install python-tk@$PY_VER"
    else
      echo "    fix on Ubuntu: sudo apt install python3-tk"
    fi
  fi
fi

# ----------------------------------------------------------------------------
# Robots
# ----------------------------------------------------------------------------
if $DO_ROBOTS; then
  mapfile -t ROBOT_IPS < <(python3 -c "
import yaml
with open('$CONFIG') as f:
    cfg = yaml.safe_load(f)
for r in cfg['robots']:
    print(r['ip'])
")

  for IP in "${ROBOT_IPS[@]}"; do
    echo ""
    echo "==> robot @ $IP: installing ${ROBOT_PKGS[*]}"
    ssh "$REMOTE_USER@$IP" "$REMOTE_PYTHON -m pip install --user --upgrade ${ROBOT_PKGS[*]}"
  done
fi

echo ""
echo "done."
