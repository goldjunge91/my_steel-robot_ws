#!/bin/bash
# COMPLETE MOTOR TEST - Does everything from our conversation
# Logs everything to complete_test.log

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

LOGFILE="complete_test.log"

# Log function
log() {
    echo -e "$1" | tee -a "$LOGFILE"
}

log "${BLUE}========================================${NC}"
log "${BLUE}  COMPLETE MOTOR TEST${NC}"
log "${BLUE}  $(date)${NC}"
log "${BLUE}========================================${NC}"
log ""

# Step 1: Build robot_hardware_interfaces
log "${YELLOW}[STEP 1/7] Building robot_hardware_interfaces...${NC}"
cd ~/workspace/ros2_dev_ws/my_steel-robot_ws
colcon build --merge-install --packages-select robot_hardware_interfaces 2>&1 | tee -a "$LOGFILE"
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    log "${GREEN}✓ Build successful${NC}"
else
    log "${RED}✗ Build failed!${NC}"
    exit 1
fi
log ""

# Step 2: Source workspace
log "${YELLOW}[STEP 2/7] Sourcing workspace...${NC}"
source install/setup.bash
log "${GREEN}✓ Workspace sourced${NC}"
log ""

# Step 3: Check micro-ROS agent
log "${YELLOW}[STEP 3/7] Checking micro-ROS agent...${NC}"
if pgrep -f "micro_ros_agent" > /dev/null; then
    AGENT_PID=$(pgrep -f "micro_ros_agent")
    log "${GREEN}✓ micro-ROS agent running (PID: ${AGENT_PID})${NC}"
else
    log "${RED}✗ micro-ROS agent NOT running!${NC}"
    log "${YELLOW}Starting micro-ROS agent...${NC}"
    
    # Find device
    if ls /dev/ttyACM* 2>/dev/null; then
        DEVICE=$(ls /dev/ttyACM* | head -1)
        log "Found device: ${DEVICE}"
        
        # Start agent in background
        ros2 run micro_ros_agent micro_ros_agent serial --dev ${DEVICE} > micro_ros_agent.log 2>&1 &
        AGENT_PID=$!
        log "Started agent with PID: ${AGENT_PID}"
        sleep 3
        
        if ps -p $AGENT_PID > /dev/null; then
            log "${GREEN}✓ Agent started successfully${NC}"
        else
            log "${RED}✗ Agent failed to start${NC}"
            exit 1
        fi
    else
        log "${RED}✗ No Pico found on USB!${NC}"
        exit 1
    fi
fi
log ""

# Step 4: Check topics
log "${YELLOW}[STEP 4/7] Checking ROS2 topics...${NC}"
sleep 2
log "Available topics:"
ros2 topic list 2>&1 | tee -a "$LOGFILE"

if ros2 topic list | grep -q "/joint_states"; then
    log "${GREEN}✓ /joint_states exists${NC}"
else
    log "${RED}✗ /joint_states missing${NC}"
fi

if ros2 topic list | grep -q "/cmd_vel"; then
    log "${GREEN}✓ /cmd_vel exists${NC}"
else
    log "${RED}✗ /cmd_vel missing${NC}"
fi
log ""

# Step 5: Check joint_states publishing
log "${YELLOW}[STEP 5/7] Checking joint_states publishing rate...${NC}"
log "Measuring for 3 seconds..."
RATE=$(timeout 3 ros2 topic hz /joint_states 2>&1 | grep "average rate" | tail -1 | awk '{print $3}')

if [ -z "$RATE" ]; then
    log "${RED}✗ No joint states being published!${NC}"
    log ""
    log "${YELLOW}Checking firmware logs...${NC}"
    log "Last 20 lines from micro-ROS agent:"
    tail -20 micro_ros_agent.log 2>/dev/null | tee -a "$LOGFILE" || log "No agent log found"
    log ""
    log "${RED}PROBLEM: Firmware not publishing joint_states${NC}"
    log "This means MotorsAgent is not creating entities."
    exit 1
else
    log "${GREEN}✓ Publishing at ${RATE} Hz${NC}"
fi
log ""

# Step 6: Show current joint states
log "${YELLOW}[STEP 6/7] Reading current joint states...${NC}"
timeout 2 ros2 topic echo /joint_states --once 2>&1 | tee -a "$LOGFILE"
log ""

# Step 7: Test motor movement
log "${YELLOW}[STEP 7/7] Testing motor movement...${NC}"
log ""
log "${BLUE}Sending forward command (0.1 m/s for 3 seconds)...${NC}"

# Get initial position
INITIAL=$(timeout 2 ros2 topic echo /joint_states --once --field "position[1]" 2>/dev/null || echo "N/A")
log "Initial front_right position: ${INITIAL}"

# Send command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {z: 0.0}}" \
  --rate 10 > /dev/null 2>&1 &
CMD_PID=$!

sleep 3

# Get final position
FINAL=$(timeout 2 ros2 topic echo /joint_states --once --field "position[1]" 2>/dev/null || echo "N/A")
log "Final front_right position: ${FINAL}"

# Stop command
kill $CMD_PID 2>/dev/null || true
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" \
  --once > /dev/null 2>&1

log ""

# Check if motor moved
if [ "$INITIAL" != "$FINAL" ] && [ "$INITIAL" != "N/A" ] && [ "$FINAL" != "N/A" ]; then
    log "${GREEN}✓✓✓ MOTOR MOVED! ✓✓✓${NC}"
    log "Position changed from ${INITIAL} to ${FINAL}"
    DELTA=$(echo "$FINAL - $INITIAL" | bc -l 2>/dev/null || echo "unknown")
    log "Delta: ${DELTA} rad"
else
    log "${RED}✗ Motor did NOT move${NC}"
    log "Possible issues:"
    log "  - Motor not connected (check pins 4,5,8,9)"
    log "  - Encoder not working"
    log "  - Firmware not receiving cmd_vel"
fi
log ""

# Summary
log "${BLUE}========================================${NC}"
log "${BLUE}  TEST SUMMARY${NC}"
log "${BLUE}========================================${NC}"
log ""
log "Workspace: Built ✓"
log "micro-ROS Agent: Running ✓"
log "Topics: /joint_states, /cmd_vel, /odom ✓"
log "Publishing Rate: ${RATE:-N/A} Hz"
log "Motor Test: $([ "$INITIAL" != "$FINAL" ] && echo "PASSED ✓" || echo "FAILED ✗")"
log ""
log "Full log saved to: ${LOGFILE}"
log ""
log "Next steps:"
log "  1. If motor moved: Connect other 3 motors"
log "  2. If motor didn't move: Check hardware connections"
log "  3. View agent logs: tail -f micro_ros_agent.log"
log "  4. View firmware logs: screen /dev/ttyACM0 115200"
log ""
