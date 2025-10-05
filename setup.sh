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

vcs import < src/ros2.repos src

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  libboost1.74-dev \
  "ros-${ROS_DISTRO}-ament-cppcheck" \
  "ros-${ROS_DISTRO}-ament-cpplint" || true

rosdep update --rosdistro="$ROS_DISTRO"
rosdep install --from-paths src --ignore-src -y --rosdistro="$ROS_DISTRO"
