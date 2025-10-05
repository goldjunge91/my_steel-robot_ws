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

# Import repositories if ros2.repos exists and has actual repositories
if [ -f "src/ros2.repos" ]; then
  if command -v vcs >/dev/null 2>&1; then
    echo "Importing repositories from src/ros2.repos..."
    vcs import src < src/ros2.repos || echo "Warning: vcs import failed or no repositories to import"
  else
    echo "Warning: vcs tool not found, skipping repository import"
  fi
fi

# Detect if we need sudo (not needed in Docker containers where we're already root)
SUDO=""
if [ "$(id -u)" -ne 0 ] && command -v sudo >/dev/null 2>&1; then
  SUDO="sudo"
fi

# Update package lists and install dependencies
echo "Updating apt package lists..."
$SUDO apt-get update || echo "Warning: apt-get update failed"

echo "Installing required packages..."
$SUDO apt-get install -y --no-install-recommends \
  libboost1.74-dev \
  "ros-${ROS_DISTRO}-ament-cppcheck" \
  "ros-${ROS_DISTRO}-ament-cpplint" 2>/dev/null || echo "Warning: Some apt packages could not be installed"

# Update rosdep
echo "Updating rosdep..."
rosdep update --rosdistro="$ROS_DISTRO" || echo "Warning: rosdep update failed"

# Install dependencies from package.xml files
echo "Installing ROS dependencies..."
rosdep install --from-paths src --ignore-src -y --rosdistro="$ROS_DISTRO" || echo "Warning: Some rosdep dependencies could not be installed"
