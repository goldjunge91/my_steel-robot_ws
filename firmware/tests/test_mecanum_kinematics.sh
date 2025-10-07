#!/bin/bash

# Test script for mecanum drive kinematics implementation
# Tests that all 4 motors receive commands for mecanum drive

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "=========================================="
echo "  Mecanum Drive Kinematics Test"
echo "=========================================="
echo ""

# Test 1: Forward motion (vx > 0, vy = 0, omega = 0)
echo -e "${YELLOW}Test 1: Forward motion${NC}"
echo "Expected: All 4 wheels should have same velocity"
echo "Command: vx=0.1 m/s, vy=0, omega=0"
echo ""

# Test 2: Strafe right (vx = 0, vy > 0, omega = 0)
echo -e "${YELLOW}Test 2: Strafe right${NC}"
echo "Expected: FL/RR positive, FR/RL negative (same magnitude)"
echo "Command: vx=0, vy=0.1 m/s, omega=0"
echo ""

# Test 3: Rotate CCW (vx = 0, vy = 0, omega > 0)
echo -e "${YELLOW}Test 3: Rotate counter-clockwise${NC}"
echo "Expected: FL/RL negative, FR/RR positive"
echo "Command: vx=0, vy=0, omega=0.5 rad/s"
echo ""

# Test 4: Diagonal motion (vx > 0, vy > 0, omega = 0)
echo -e "${YELLOW}Test 4: Diagonal forward-right${NC}"
echo "Expected: FR/RL high, FL/RR low"
echo "Command: vx=0.1 m/s, vy=0.1 m/s, omega=0"
echo ""

# Test 5: Complex motion (vx > 0, vy > 0, omega > 0)
echo -e "${YELLOW}Test 5: Forward-right with rotation${NC}"
echo "Expected: All 4 wheels different velocities"
echo "Command: vx=0.1 m/s, vy=0.05 m/s, omega=0.3 rad/s"
echo ""

echo -e "${GREEN}✓ Mecanum kinematics test cases defined${NC}"
echo ""
echo "To test manually:"
echo "1. Flash firmware with mecanum support"
echo "2. Start micro-ROS agent"
echo "3. Publish test commands:"
echo "   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" --once"
echo "4. Monitor joint states:"
echo "   ros2 topic echo /joint_states"
echo "5. Verify all 4 wheels show velocity changes"
