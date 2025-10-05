#!/bin/bash
set -euo pipefail

# Ensure we're in the workspace directory
cd /github/workspace || { echo "Failed to cd to /github/workspace"; exit 1; }

ROS_DISTRO=${ROS_DISTRO:-humble}
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [ ! -f "$ROS_SETUP" ]; then
  echo "Unable to locate ROS setup script at $ROS_SETUP" >&2
  exit 1
fi

# Set AMENT_TRACE_SETUP_FILES to avoid unset variable error with set -u
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-0}

# shellcheck disable=SC1090
source "$ROS_SETUP"

echo "Running setup script..."
bash ./setup.sh

if [ -f "install/setup.bash" ]; then
  echo "Sourcing workspace setup..."
  # shellcheck disable=SC1090
  source install/setup.bash
fi

if [ -z "${LINTER:-}" ]; then
  echo "LINTER environment variable is not set." >&2
  exit 1
fi

echo "Running linter: ament_${LINTER}..."
ament_"${LINTER}" src/
