#!/bin/bash
set -euo pipefail

ROS_DISTRO=${ROS_DISTRO:-humble}
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [ ! -f "$ROS_SETUP" ]; then
  echo "Unable to locate ROS setup script at $ROS_SETUP" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"

./setup.sh

if [ -f "install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source install/setup.bash
fi

if [ -z "${LINTER:-}" ]; then
  echo "LINTER environment variable is not set." >&2
  exit 1
fi

ament_"${LINTER}" src/
