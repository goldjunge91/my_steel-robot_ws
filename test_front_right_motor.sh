#!/bin/bash
# Comprehensive Front Right Motor Test Script
# Tests Motor 1 (front_right_wheel_joint) with firmware diagnostics

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Front Right Motor Test (Motor 1)${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Function to check if a topic exists
check_topic() {
    local topic=$1
    if ros2 topic list | grep -q "^${topic}$"; then
        echo -e "${GREEN}✓${NC} Topic ${topic} exists"
        return 0
    else
        echo -e "${RED}✗${NC} Topic ${topic} NOT found"
        return 1
    fi
}

# Function to get joint state value
get_joint_value() {
    local field=$1
    local index=$2
    ros2 topic echo /joint_states --once --field "${field}[${index}]" 2>/dev/null || echo "N/A"
}

# 1. Check ROS2 Environment
echo -e "${YELLOW}[1/8] Checking ROS2 Environment...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗${NC} ROS2 not sourced!"
    echo "Run: source /opt/ros/humble/setup.bash"
    exit 1
fi
echo -e "${GREEN}✓${NC} ROS2 ${ROS_DISTRO} environment active"
echo ""

# 2. Check micro-ROS Agent
echo -e "${YELLOW}[2/8] Checking micro-ROS Agent...${NC}"
if pgrep -f "micro_ros_agent" > /dev/null; then
    echo -e "${GREEN}✓${NC} micro-ROS agent is running"
else
    echo -e "${RED}✗${NC} micro-ROS agent NOT running!"
    echo "Start it with: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0"
    exit 1
fi
echo ""

# 3. Check Firmware Topics
echo -e "${YELLOW}[3/8] Checking Firmware Topics...${NC}"
check_topic "/joint_states" || echo -e "${RED}  → Firmware may not be publishing!${NC}"
check_topic "/cmd_vel" || echo -e "${YELLOW}  → cmd_vel topic not found (will be created)${NC}"
check_topic "/odom" || echo -e "${YELLOW}  → Odometry not available${NC}"

echo ""
echo "Available topics from firmware:"
ros2 topic list | grep -E "(joint_states|cmd_vel|odom|imu)" || echo "  No firmware topics found!"
echo ""

# 4. Check Joint States Publishing Rate
echo -e "${YELLOW}[4/8] Checking Joint States Publishing Rate...${NC}"
echo "Measuring for 3 seconds..."
RATE=$(timeout 3 ros2 topic hz /joint_states 2>&1 | grep "average rate" | tail -1 | awk '{print $3}')
if [ -z "$RATE" ]; then
    echo -e "${RED}✗${NC} No joint states being published!"
    exit 1
elif (( $(echo "$RATE < 4.0" | bc -l) )); then
    echo -e "${RED}✗${NC} Publishing rate too low: ${RATE} Hz (expected ~10 Hz)"
    echo -e "${YELLOW}  → Firmware may be running slow${NC}"
elif (( $(echo "$RATE < 8.0" | bc -l) )); then
    echo -e "${YELLOW}⚠${NC} Publishing rate acceptable but low: ${RATE} Hz"
else
    echo -e "${GREEN}✓${NC} Publishing rate: ${RATE} Hz"
fi
echo ""

# 5. Check Controller Manager
echo -e "${YELLOW}[5/8] Checking Controller Manager...${NC}"
if ros2 node list | grep -q "controller_manager"; then
    echo -e "${GREEN}✓${NC} Controller Manager is running"
    
    # Check controllers
    echo ""
    echo "Active controllers:"
    ros2 control list_controllers 2>/dev/null || echo "  Could not list controllers"
else
    echo -e "${RED}✗${NC} Controller Manager NOT running!"
    echo "Start it with: ros2 launch robot_controller controller.launch.py"
    exit 1
fi
echo ""

# 6. Analyze Current Joint States
echo -e "${YELLOW}[6/8] Analyzing Front Right Motor State...${NC}"
echo "Reading current joint states..."

JOINT_DATA=$(timeout 2 ros2 topic echo /joint_states --once 2>/dev/null)
if [ -z "$JOINT_DATA" ]; then
    echo -e "${RED}✗${NC} Could not read joint states!"
    exit 1
fi

# Extract front_right_wheel_joint data (index 1)
echo "$JOINT_DATA" > /tmp/joint_states.txt

FR_POS=$(echo "$JOINT_DATA" | grep -A 10 "position:" | sed -n '3p' | tr -d ' -')
FR_VEL=$(echo "$JOINT_DATA" | grep -A 10 "velocity:" | sed -n '3p' | tr -d ' -')

echo ""
echo "Front Right Motor (front_right_wheel_joint):"
echo "  Position: ${FR_POS} rad"
echo "  Velocity: ${FR_VEL} rad/s"
echo ""

if [ "$FR_VEL" == "0.0" ]; then
    echo -e "${GREEN}✓${NC} Motor is stationary (as expected)"
else
    echo -e "${YELLOW}⚠${NC} Motor showing velocity: ${FR_VEL} rad/s"
fi
echo ""

# 7. Test Motor Movement
echo -e "${YELLOW}[7/8] Testing Motor Movement...${NC}"
echo ""
echo -e "${BLUE}Test 1: Forward (0.1 m/s for 3 seconds)${NC}"
echo "The front right motor should spin..."

# Get initial position
INITIAL_POS=$(get_joint_value "position" 1)
echo "Initial position: ${INITIAL_POS}"

# Send forward command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {z: 0.0}}" \
  --rate 10 &
CMD_PID=$!

sleep 3

# Get final position
FINAL_POS=$(get_joint_value "position" 1)
echo "Final position: ${FINAL_POS}"

# Stop motor
kill $CMD_PID 2>/dev/null || true
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" \
  --once

# Check if position changed
if [ "$INITIAL_POS" != "$FINAL_POS" ] && [ "$INITIAL_POS" != "N/A" ] && [ "$FINAL_POS" != "N/A" ]; then
    echo -e "${GREEN}✓${NC} Motor moved! Position changed from ${INITIAL_POS} to ${FINAL_POS}"
    
    # Calculate delta
    DELTA=$(echo "$FINAL_POS - $INITIAL_POS" | bc -l)
    echo "  Delta: ${DELTA} rad"
else
    echo -e "${RED}✗${NC} Motor did NOT move!"
    echo -e "${RED}  → Check motor connections (Pins 4,5 for PWM, Pins 8,9 for encoder)${NC}"
    echo -e "${RED}  → Check if firmware is receiving cmd_vel${NC}"
fi

echo ""
sleep 1

# 8. Monitor velocity during movement
echo -e "${YELLOW}[8/8] Monitoring Velocity Feedback...${NC}"
echo ""
echo -e "${BLUE}Test 2: Slow forward (0.05 m/s for 5 seconds)${NC}"
echo "Watching velocity values..."
echo ""

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {z: 0.0}}" \
  --rate 10 &
CMD_PID=$!

# Monitor for 5 seconds
for i in {1..5}; do
    VEL=$(get_joint_value "velocity" 1)
    POS=$(get_joint_value "position" 1)
    echo "[$i/5] Position: ${POS} rad, Velocity: ${VEL} rad/s"
    sleep 1
done

# Stop motor
kill $CMD_PID 2>/dev/null || true
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" \
  --once

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Test Summary${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "Motor: Front Right (front_right_wheel_joint)"
echo "Pins: PWM CW=4, PWM CCW=5, Encoder A=8, Encoder B=9"
echo ""
echo "If the motor moved and encoder values changed:"
echo -e "  ${GREEN}✓ Hardware is working correctly${NC}"
echo ""
echo "If the motor did NOT move:"
echo "  1. Check physical connections"
echo "  2. Check firmware is flashed correctly"
echo "  3. Check cmd_vel is reaching firmware:"
echo "     ros2 topic echo /cmd_vel"
echo "  4. Check firmware logs via serial monitor"
echo ""
echo "Next steps:"
echo "  - Connect remaining motors (FL, RL, RR)"
echo "  - Test full mecanum drive"
echo "  - Calibrate wheel parameters"
echo ""
