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

if [ -f "install/setup.bash" ]; then
  echo "Sourcing existing workspace setup..."
  # shellcheck disable=SC1090
  source install/setup.bash
fi

# Set the default build type
BUILD_TYPE=${BUILD_TYPE:-RelWithDebInfo}
echo "Building workspace with build type: $BUILD_TYPE"
colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic
