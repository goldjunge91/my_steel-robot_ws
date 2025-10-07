#!/bin/bash
# Firmware Diagnostics Script
# Checks if firmware is properly configured and communicating

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Firmware Diagnostics${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 1. Check USB Connection
echo -e "${YELLOW}[1/6] Checking USB Connection...${NC}"
if ls /dev/ttyACM* 2>/dev/null; then
    echo -e "${GREEN}✓${NC} Pico found on USB"
    DEVICE=$(ls /dev/ttyACM* | head -1)
    echo "  Device: ${DEVICE}"
else
    echo -e "${RED}✗${NC} No Pico found on USB!"
    echo "  Check USB cable and connection"
    exit 1
fi
echo ""

# 2. Check micro-ROS Agent
echo -e "${YELLOW}[2/6] Checking micro-ROS Agent...${NC}"
if pgrep -f "micro_ros_agent" > /dev/null; then
    AGENT_PID=$(pgrep -f "micro_ros_agent")
    echo -e "${GREEN}✓${NC} micro-ROS agent running (PID: ${AGENT_PID})"
    
    # Check which device it's using
    AGENT_CMD=$(ps -p $AGENT_PID -o cmd=)
    echo "  Command: ${AGENT_CMD}"
else
    echo -e "${RED}✗${NC} micro-ROS agent NOT running!"
    echo ""
    echo "Start it with:"
    echo "  ros2 run micro_ros_agent micro_ros_agent serial --dev ${DEVICE}"
    exit 1
fi
echo ""

# 3. Check Firmware Topics
echo -e "${YELLOW}[3/6] Checking Firmware Topics...${NC}"
echo "Topics that should exist:"

EXPECTED_TOPICS=("/joint_states" "/odom" "/cmd_vel")
ALL_GOOD=true

for topic in "${EXPECTED_TOPICS[@]}"; do
    if ros2 topic list | grep -q "^${topic}$"; then
        echo -e "  ${GREEN}✓${NC} ${topic}"
    else
        echo -e "  ${RED}✗${NC} ${topic} - NOT FOUND"
        ALL_GOOD=false
    fi
done

if [ "$ALL_GOOD" = false ]; then
    echo ""
    echo -e "${RED}Missing topics! Firmware may not be running correctly.${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check if firmware is flashed:"
    echo "     cd firmware && make flash"
    echo "  2. Restart micro-ROS agent"
    echo "  3. Check serial monitor for errors:"
    echo "     screen ${DEVICE} 115200"
fi
echo ""

# 4. Check Topic Publishing Rates
echo -e "${YELLOW}[4/6] Checking Publishing Rates...${NC}"
echo "Measuring /joint_states rate (3 seconds)..."

RATE=$(timeout 3 ros2 topic hz /joint_states 2>&1 | grep "average rate" | tail -1 | awk '{print $3}')

if [ -z "$RATE" ]; then
    echo -e "${RED}✗${NC} No data being published!"
    echo "  Firmware is not sending joint states"
else
    echo "  Rate: ${RATE} Hz"
    
    if (( $(echo "$RATE < 4.0" | bc -l) )); then
        echo -e "${RED}✗${NC} Rate too low! (expected ~10 Hz)"
        echo "  Problem: Firmware loop is running too slow"
        echo "  Check: vTaskDelay(100) in firmware/src/DDD.cpp"
    elif (( $(echo "$RATE < 8.0" | bc -l) )); then
        echo -e "${YELLOW}⚠${NC} Rate acceptable but could be better"
    else
        echo -e "${GREEN}✓${NC} Publishing rate is good"
    fi
fi
echo ""

# 5. Check Joint State Content
echo -e "${YELLOW}[5/6] Checking Joint State Content...${NC}"
echo "Reading one message..."

JOINT_MSG=$(timeout 2 ros2 topic echo /joint_states --once 2>/dev/null)

if [ -z "$JOINT_MSG" ]; then
    echo -e "${RED}✗${NC} Could not read joint states!"
else
    # Check joint names
    echo ""
    echo "Joint names found:"
    echo "$JOINT_MSG" | grep -A 5 "name:" | grep "  -" | while read -r line; do
        echo "  ${line}"
    done
    
    # Check if front_right_wheel_joint exists
    if echo "$JOINT_MSG" | grep -q "front_right_wheel_joint"; then
        echo -e "${GREEN}✓${NC} front_right_wheel_joint found"
    else
        echo -e "${RED}✗${NC} front_right_wheel_joint NOT found!"
        echo "  Check firmware joint name configuration"
    fi
    
    # Check for position data
    if echo "$JOINT_MSG" | grep -q "position:"; then
        echo -e "${GREEN}✓${NC} Position data present"
    else
        echo -e "${RED}✗${NC} No position data!"
    fi
    
    # Check for velocity data
    if echo "$JOINT_MSG" | grep -q "velocity:"; then
        echo -e "${GREEN}✓${NC} Velocity data present"
    else
        echo -e "${RED}✗${NC} No velocity data!"
    fi
fi
echo ""

# 6. Test cmd_vel Reception
echo -e "${YELLOW}[6/6] Testing cmd_vel Reception...${NC}"
echo "Sending test command..."

# Send a test command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.01, y: 0.0, z: 0.0}}" \
  --once

sleep 1

# Check if firmware is echoing it back (some firmwares do this)
echo "Checking if firmware received command..."
echo "(Monitor serial output for confirmation)"
echo ""

# Summary
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Diagnostic Summary${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

if [ "$ALL_GOOD" = true ] && [ -n "$RATE" ]; then
    echo -e "${GREEN}✓ Firmware appears to be working correctly${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Run motor test: bash test_front_right_motor.sh"
    echo "  2. Check encoder feedback during movement"
    echo "  3. Verify motor responds to cmd_vel"
else
    echo -e "${RED}✗ Firmware has issues${NC}"
    echo ""
    echo "Recommended actions:"
    echo "  1. Reflash firmware: cd firmware && make flash"
    echo "  2. Check serial monitor: screen ${DEVICE} 115200"
    echo "  3. Restart micro-ROS agent"
    echo "  4. Check firmware logs for errors"
fi
echo ""

echo "Firmware source: firmware/src/DDD.cpp"
echo "Motor config: firmware/src/main.cpp (Pins 4,5,8,9)"
echo "Publishing loop: DDD::run() with vTaskDelay(100)"
echo ""
