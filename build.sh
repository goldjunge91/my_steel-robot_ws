#!/bin/bash
# -----------------------------------------------------------------------------
# build.sh
#
# This script builds the ROS2 workspace using colcon with optimized settings.
# - Sets the build type to RelWithDebInfo for optimized builds with debug info.

# Argument explanations:
# | Argument                                                    | Description                                      |
# |-------------------------------------------------------------|--------------------------------------------------|
# | --symlink-install                                           | Use symlinks for install (faster iteration)      |
# | --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE"               | Set CMake build type (RelWithDebInfo)            |
# | --cmake-args "-DCMAKE_EXPORT_COMPILE_COMMANDS=On"           | Export compile_commands.json for tooling support  |
# | -Wall -Wextra -Wpedantic                                    | Enable strict compiler warnings                  |
#
# Usage:
#   ./build.sh
#
# Requirements:
#   - Must be run from the workspace root.
#   - Assumes ROS2 and colcon are installed and sourced.
# -----------------------------------------------------------------------------

set -e

# Set the default build type
BUILD_TYPE=RelWithDebInfo

colcon build \
        --symlink-install \  # Use symlinks for install (faster iteration)
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \  # Set build type and export compile commands
        -Wall -Wextra -Wpedantic  # Enable strict compiler warnings
