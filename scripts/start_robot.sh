#!/bin/bash
# Start script for robot on Raspberry Pi
# This script starts all necessary components for the robot

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "Starting Robot System"
echo "=========================================="
echo "Workspace: $WORKSPACE_DIR"
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

# Set ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0

# Log directory
LOG_DIR="$WORKSPACE_DIR/logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

echo "Logs will be saved to: $LOG_DIR"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down robot system..."
    
    # Kill all child processes
    pkill -P $$
    
    # Kill specific ROS2 processes
    pkill -f "micro_ros_agent" 2>/dev/null || true
    pkill -f "foxglove_bridge" 2>/dev/null || true
    pkill -f "ros2_control_node" 2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    pkill -f "usb_cam" 2>/dev/null || true
    pkill -f "spawner" 2>/dev/null || true
    
    echo "All processes stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start micro-ROS agent
echo "[1/3] Starting micro-ROS agent..."
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 > "$LOG_DIR/microros_agent.log" 2>&1 &
MICROROS_PID=$!
echo "  ✓ micro-ROS agent started (PID: $MICROROS_PID)"
sleep 2

# Start robot bringup
echo "[2/3] Starting robot bringup..."
ros2 launch robot_bringup bringup.launch.py \
    robot_model:=robot_xl \
    mecanum:=True \
    camera:=True \
    microros:=False \
    > "$LOG_DIR/robot_bringup.log" 2>&1 &
BRINGUP_PID=$!
echo "  ✓ Robot bringup started (PID: $BRINGUP_PID)"
sleep 5

# Start Foxglove Bridge
echo "[3/3] Starting Foxglove Bridge..."
ros2 run foxglove_bridge foxglove_bridge --port 8765 > "$LOG_DIR/foxglove_bridge.log" 2>&1 &
FOXGLOVE_PID=$!
echo "  ✓ Foxglove Bridge started (PID: $FOXGLOVE_PID)"

echo ""
echo "=========================================="
echo "Robot System Started Successfully!"
echo "=========================================="
echo "micro-ROS agent:  PID $MICROROS_PID"
echo "Robot bringup:    PID $BRINGUP_PID"
echo "Foxglove Bridge:  PID $FOXGLOVE_PID"
echo ""
echo "Logs: $LOG_DIR"
echo "Foxglove: ws://$(hostname -I | awk '{print $1}'):8765"
echo ""
echo "Press Ctrl+C to stop all services"
echo "=========================================="

# Wait for all background processes
wait
