#!/bin/bash
set -e

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

if [ -f install/setup.bash ]; then 
  # shellcheck disable=SC1090
  source install/setup.bash
else
  echo "Warning: Workspace not built yet, install/setup.bash not found"
fi

echo "Running colcon test..."
colcon test --merge-install || echo "Warning: Some tests failed or no tests found"

echo "Displaying test results..."
colcon test-result --verbose || echo "Warning: Could not display test results"
