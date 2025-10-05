#!/bin/bash
set -euo pipefail

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

# Import repositories if ros2.repos exists and has actual repositories
if [ -f "src/ros2.repos" ]; then
  if command -v vcs >/dev/null 2>&1; then
    echo "Importing repositories from src/ros2.repos..."
    set +e
    vcs import src < src/ros2.repos
    VCS_EXIT=$?
    set -e
    if [ $VCS_EXIT -ne 0 ]; then
      echo "Warning: vcs import failed or no repositories to import"
    fi
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
set +e
$SUDO apt-get update
APT_UPDATE_EXIT=$?
set -e
if [ $APT_UPDATE_EXIT -ne 0 ]; then
  echo "Warning: apt-get update failed"
fi

echo "Installing required packages..."
set +e
$SUDO apt-get install -y --no-install-recommends \
  libboost1.74-dev \
  "ros-${ROS_DISTRO}-ament-cppcheck" \
  "ros-${ROS_DISTRO}-ament-cpplint" \
  "ros-${ROS_DISTRO}-ament-uncrustify" \
  "ros-${ROS_DISTRO}-ament-lint-cmake" \
  "ros-${ROS_DISTRO}-ament-xmllint" \
  "ros-${ROS_DISTRO}-ament-flake8" \
  "ros-${ROS_DISTRO}-ament-pep257" 2>/dev/null
APT_INSTALL_EXIT=$?
set -e
if [ $APT_INSTALL_EXIT -ne 0 ]; then
  echo "Warning: Some apt packages could not be installed"
fi

# Update rosdep
echo "Updating rosdep..."
# Initialize rosdep if not already initialized (safe to run multiple times)
if [ ! -d "/etc/ros/rosdep/sources.list.d" ] || [ -z "$(ls -A /etc/ros/rosdep/sources.list.d 2>/dev/null)" ]; then
  echo "Initializing rosdep..."
  set +e
  $SUDO rosdep init
  ROSDEP_INIT_EXIT=$?
  set -e
  if [ $ROSDEP_INIT_EXIT -ne 0 ]; then
    echo "Warning: rosdep init failed (may already be initialized)"
  fi
fi
set +e
rosdep update --rosdistro="$ROS_DISTRO"
ROSDEP_UPDATE_EXIT=$?
set -e
if [ $ROSDEP_UPDATE_EXIT -ne 0 ]; then
  echo "Warning: rosdep update failed"
fi

# Install dependencies from package.xml files
echo "Installing ROS dependencies..."
set +e
rosdep install --from-paths src --ignore-src -y --rosdistro="$ROS_DISTRO"
ROSDEP_INSTALL_EXIT=$?
set -e
if [ $ROSDEP_INSTALL_EXIT -ne 0 ]; then
  echo "Warning: Some rosdep dependencies could not be installed"
fi

echo "Setup completed successfully!"
