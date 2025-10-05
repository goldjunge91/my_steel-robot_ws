#!/bin/bash

# Quick start script for robot hardware
# Run this on the Raspberry Pi to start everything

set -e

# Source environment
source "$(dirname "$0")/setup_robot_env.sh"

echo ""
echo "========================================"
echo "Starting Robot Hardware"
echo "========================================"
echo ""

# Check if Pico is connected
if [ ! -e /dev/ttyACM0 ] && [ ! -e /dev/ttyACM1 ]; then
    echo "WARNING: Pico not found on /dev/ttyACM0 or /dev/ttyACM1"
    echo "Please check USB connection"
fi

# Check if camera is connected
if [ ! -e /dev/video0 ]; then
    echo "WARNING: Camera not found on /dev/video0"
    echo "Camera node may fail to start"
fi

echo ""
echo "Starting robot_bringup..."
echo ""

# Launch hardware bringup
ros2 launch robot_bringup hardware_bringup.launch.py
