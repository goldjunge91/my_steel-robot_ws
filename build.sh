#!/bin/bash
set -euo pipefail

ROS_DISTRO=${ROS_DISTRO:-humble}
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f "$ROS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
else
  echo "ROS setup script not found at $ROS_SETUP — continuing without sourcing ROS." >&2
fi

if [ -f "install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source install/setup.bash
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon is not available; skipping build step." >&2
  exit 0
fi

# Set the default build type
BUILD_TYPE=${BUILD_TYPE:-RelWithDebInfo}
colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic || true
