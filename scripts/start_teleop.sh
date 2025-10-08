#!/bin/bash
# Teleoperation script for remote PC
# This script starts joy node and teleop_twist_joy for Xbox controller

set -e

echo "=========================================="
echo "Starting Robot Teleoperation"
echo "=========================================="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

# Set ROS_DOMAIN_ID (must match robot)
export ROS_DOMAIN_ID=0

# Check if controller is connected
if ! ls /dev/input/js* > /dev/null 2>&1; then
    echo "ERROR: No joystick found at /dev/input/js*"
    echo "Please connect your Xbox controller via USB"
    exit 1
fi

CONTROLLER=$(ls /dev/input/js* | head -1)
echo "Controller found: $CONTROLLER"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down teleoperation..."
    pkill -P $$
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start joy node
echo "[1/2] Starting joy node..."
ros2 run joy joy_node &
JOY_PID=$!
echo "  ✓ Joy node started (PID: $JOY_PID)"
sleep 1

# Start teleop_twist_joy
echo "[2/2] Starting teleop_twist_joy..."
ros2 run teleop_twist_joy teleop_node &
TELEOP_PID=$!
echo "  ✓ Teleop node started (PID: $TELEOP_PID)"

echo ""
echo "=========================================="
echo "Teleoperation Started Successfully!"
echo "=========================================="
echo "Joy node:    PID $JOY_PID"
echo "Teleop node: PID $TELEOP_PID"
echo ""
echo "Controls:"
echo "  - Hold LB (Left Bumper) to enable"
echo "  - Left Stick: Forward/Backward + Strafe"
echo "  - Right Stick: Rotate"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="

# Wait for all background processes
wait
