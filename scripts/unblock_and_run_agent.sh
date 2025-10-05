#!/bin/bash
set -euo pipefail

DEVICE=${1:-/dev/ttyACM0}
KILL=false
START_AGENT=false

usage(){
  cat <<EOF
Usage: $0 [device] [--kill] [--start-agent]

Examples:
  $0                    # shows processes holding /dev/ttyACM0
  $0 --kill             # kills processes holding /dev/ttyACM0 (uses sudo)
  $0 /dev/ttyACM1 --kill --start-agent  # acts on /dev/ttyACM1 then starts agent
EOF
}

for arg in "$@"; do
  case "$arg" in
    --kill) KILL=true ;;
    --start-agent) START_AGENT=true ;;
    -h|--help) usage; exit 0 ;;
    /dev/*) DEVICE="$arg" ;;
    *) ;; 
  esac
done

echo "Device: $DEVICE"

if [ ! -e "$DEVICE" ]; then
  echo "$DEVICE does not exist"
  exit 1
fi

echo "Processes holding $DEVICE (lsof):"
lsof "$DEVICE" || true

echo "Processes holding $DEVICE (fuser -v):"
fuser -v "$DEVICE" || true

if [ "$KILL" = true ]; then
  echo "Killing processes holding $DEVICE (requires sudo)"
  sudo fuser -k "$DEVICE" || true
  sleep 1
  echo "Post-kill processes (lsof):"
  lsof "$DEVICE" || true
fi

if [ "$START_AGENT" = true ]; then
  echo "Attempting to start micro-ROS Agent using ros2 run ..."
  # Try to source local install if present
  if [ -f "install/setup.bash" ]; then
    echo "Sourcing install/setup.bash"
    # shellcheck disable=SC1091
    . "install/setup.bash"
  else
    echo "install/setup.bash not found in repo root; ensure ROS2 environment is sourced"
  fi

  echo "Starting: ros2 run micro_ros_agent micro_ros_agent serial --dev ${DEVICE} -b 115200 -v --discovery-timeout 10"
  ros2 run micro_ros_agent micro_ros_agent serial --dev "${DEVICE}" -b 115200 -v --discovery-timeout 10 || true
fi
