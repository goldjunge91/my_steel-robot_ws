#!/usr/bin/env bash
set -euo pipefail

DEVICE=/dev/ttyACM0
KILL=true
START_AGENT=true
VERBOSE=true
WAIT_SECS=0
PREFER_BY_ID=false

usage(){
  cat <<EOF
Usage: $0 [device] [--kill] [--start-agent] [--verbose]

Examples:
  $0                      # checks /dev/ttyUSB0
  $0 /dev/ttyACM0 --kill   # free /dev/ttyACM0 (uses sudo)
  $0 --start-agent         # attempt to start micro-ROS Agent after checks

This script performs safe diagnostics for a Pi Pico connected as a micro-ROS
device on the given serial node (default: /dev/ttyUSB0). It does NOT flash
firmware.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --kill)
      KILL=true; shift ;;
    --start-agent)
      START_AGENT=true; shift ;;
    --verbose)
      VERBOSE=true; shift ;;
    --wait)
      shift; WAIT_SECS=${1:-0}; shift ;;
    --wait=*)
      WAIT_SECS="${1#*=}"; shift ;;
    --prefer-by-id)
      PREFER_BY_ID=true; shift ;;
    -h|--help)
      usage; exit 0 ;;
    /dev/*)
      DEVICE="$1"; shift ;;
    *)
      echo "Unknown arg: $1"; shift ;;
  esac
done

# If prefer-by-id given, try to find a stable by-id path for Pico-like devices
choose_by_id(){
  if [ -d /dev/serial/by-id ]; then
    for f in /dev/serial/by-id/*; do
      [ -e "$f" ] || continue
      name=$(basename -- "$f")
      case "$name" in
        *pico*|*raspberry*|*micro*|*ttyACM*|*ttyUSB*)
          echo "$f"
          return 0
          ;;
      esac
    done
    # fallback: first entry
    for f in /dev/serial/by-id/*; do
      [ -e "$f" ] || continue
      echo "$f"
      return 0
    done
  fi
  return 1
}

if [ "$PREFER_BY_ID" = true ]; then
  byid=$(choose_by_id || true)
  if [ -n "$byid" ]; then
    echo "Prefer by-id device: $byid"
    DEVICE="$byid"
  else
    echo "No /dev/serial/by-id entry found to prefer; keeping $DEVICE"
  fi
fi

if [ "$VERBOSE" = true ]; then
  echo "VERBOSE: enable shell trace"
  set -x
fi

wait_for_device(){
  local dev="$1"
  local timeout="$2"
  local waited=0
  while [ ! -e "$dev" ] && [ "$waited" -lt "$timeout" ]; do
    sleep 1
    waited=$((waited+1))
  done
  if [ -e "$dev" ]; then
    return 0
  else
    return 1
  fi
}

echo "=== micro-ROS + Pi Pico quick check ==="
echo "Device: $DEVICE"

if [ ! -e "$DEVICE" ]; then
  echo "ERROR: Device $DEVICE does not exist"
  echo "Run 'lsusb' and 'dmesg | tail' after plugging the Pico in to diagnose." 
  exit 2
fi

echo "\n--- System info ---"
date
whoami

echo "\n--- dmesg (last 40 lines) ---"
# dmesg may require capabilities; if it fails, print a helpful hint
if dmesg | tail -n 40 >/dev/null 2>&1; then
  dmesg | tail -n 40 || true
else
  echo "dmesg: unable to read kernel buffer (permission denied). Try 'sudo dmesg | tail' or check 'journalctl -k'."
fi

echo "\n--- lsusb ---"
lsusb || true

echo "\n--- /dev listing for common devices ---"
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/pico_micro_ros 2>/dev/null || true

echo "\n--- udev info for $DEVICE (truncated) ---"
if command -v udevadm >/dev/null 2>&1; then
  udevadm info -a -n "$DEVICE" | sed -n '1,160p' || true
else
  echo "udevadm not available"
fi

echo "\n--- Processes holding $DEVICE ---"
if command -v lsof >/dev/null 2>&1; then
  lsof "$DEVICE" || true
fi
if command -v fuser >/dev/null 2>&1; then
  fuser -v "$DEVICE" || true
fi

if [ "$KILL" = true ]; then
  echo "\n--- Killing processes holding $DEVICE (requires sudo) ---"
  sudo fuser -k "$DEVICE" || true
  sleep 1
  echo "Post-kill (lsof):"
  lsof "$DEVICE" || true
fi

echo "\n--- micro-ROS Agent status (process) ---"
if pgrep -f "micro_ros_agent" >/dev/null; then
  AGENT_PIDS=$(pgrep -af "micro_ros_agent")
  echo "micro-ROS Agent processes found:"
  echo "$AGENT_PIDS"
else
  echo "micro-ROS Agent does not appear to be running"
fi

echo "\n--- ROS2 environment check (sourcing /opt/ros/humble if available) ---"
# Some setup scripts reference variables that are not set; temporarily disable
# 'set -u' to avoid exiting when sourcing them.
if [ -f "/opt/ros/humble/setup.bash" ]; then
  set +u || true
  # shellcheck disable=SC1090
  . /opt/ros/humble/setup.bash
  set -u
  echo "Sourced /opt/ros/humble/setup.bash"
fi
if [ -f "install/setup.bash" ]; then
  set +u || true
  # shellcheck disable=SC1090
  . "install/setup.bash"
  set -u
  echo "Sourced install/setup.bash"
fi

echo "\n--- ROS2 topic check (quick) ---"
if command -v ros2 >/dev/null 2>&1; then
  ros2 topic list | sed -n '1,200p' || true
  echo "\nChecking for likely micro-ROS topics (e.g. /ddd/odom, /imu, /joint_states):"
  ros2 topic list | grep -E "(ddd|imu|joint|pico)" || echo "None of those topics found (this may be OK)"
else
  echo "ros2 CLI not found in PATH — ensure ROS2 is installed and environment sourced"
fi

echo "\n--- Serial connection quick test (attempt to open and close) ---"
if command -v timeout >/dev/null 2>&1 && command -v od >/dev/null 2>&1; then
  echo "Attempting a short read of $DEVICE (may require permissions)..."
  sudo timeout 2 cat "$DEVICE" || true
else
  # fallback to cat
  echo "Attempting a short read of $DEVICE (may require permissions)..."
  sudo timeout 2 cat "$DEVICE" || true
fi

if [ "$START_AGENT" = true ]; then
  echo "\n--- Attempting to start micro-ROS Agent via 'ros2 run' ---"
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 not available in PATH — cannot start agent"
    exit 3
  fi
  # If device is missing and user requested waiting, wait up to WAIT_SECS
  if [ ! -e "$DEVICE" ] && [ "$WAIT_SECS" -gt 0 ]; then
    echo "Device $DEVICE not present — waiting up to ${WAIT_SECS}s for it to appear..."
    if wait_for_device "$DEVICE" "$WAIT_SECS"; then
      echo "Device appeared: $DEVICE"
    else
      echo "Timed out waiting for $DEVICE — aborting agent start"
      exit 4
    fi
  fi

  echo "Starting: ros2 run micro_ros_agent micro_ros_agent serial --dev ${DEVICE} -b 115200 -v --discovery-timeout 10"
  # Start in foreground (user can ctrl-c) — do not detach automatically
  ros2 run micro_ros_agent micro_ros_agent serial --dev "${DEVICE}" -b 115200 -v --discovery-timeout 10 || true
fi

echo "\n--- Done. Summary: ---"
echo "Device checked: $DEVICE"
echo "If you still have issues:"
echo " - Verify Pico shows up in 'dmesg' when plugged in"
echo " - Try different cable/port, check USB adapter (some adapters present as /dev/ttyUSBx)"
echo " - Use './scripts/unblock_and_run_agent.sh /dev/ttyUSB0 --kill --start-agent' for an automated unblock/start flow"

exit 0
