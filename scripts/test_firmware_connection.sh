#!/bin/bash

# Test script for my_steel firmware and micro-ROS connection
# Validates hardware communication and ROS topic publishing

set -e

# Configuration
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyACM0}"
UART_PORT="${UART_PORT:-/dev/ttyAMA0}"
TIMEOUT=30

echo "=== my_steel Firmware Connection Test ==="
echo "Serial Port: $SERIAL_PORT"
echo "UART Debug: $UART_PORT"
echo "Timeout: ${TIMEOUT}s"
echo

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test functions
test_serial_device() {
    echo -n "Testing serial device $SERIAL_PORT... "
    if [ -e "$SERIAL_PORT" ]; then
        echo -e "${GREEN}FOUND${NC}"
        return 0
    else
        echo -e "${RED}NOT FOUND${NC}"
        return 1
    fi
}

test_uart_debug() {
    echo -n "Testing UART debug output... "
    if [ -e "$UART_PORT" ]; then
        # Try to read some debug output
        timeout 5 cat "$UART_PORT" 2>/dev/null | head -3 | grep -q "." && \
        echo -e "${GREEN}ACTIVE${NC}" || echo -e "${YELLOW}NO DATA${NC}"
    else
        echo -e "${RED}PORT NOT FOUND${NC}"
    fi
}

test_microros_agent() {
    echo -n "Testing micro-ROS agent connection... "
    
    # Start micro-ROS agent in background
    ros2 run micro_ros_agent micro_ros_agent serial --dev "$SERIAL_PORT" -b 115200 > /tmp/microros_test.log 2>&1 &
    AGENT_PID=$!
    
    # Wait a bit for connection
    sleep 3
    
    # Check if agent is running and connected
    if kill -0 $AGENT_PID 2>/dev/null; then
        echo -e "${GREEN}CONNECTED${NC}"
        kill $AGENT_PID 2>/dev/null || true
        return 0
    else
        echo -e "${RED}FAILED${NC}"
        return 1
    fi
}

test_pico_topics() {
    echo "Testing Pico ROS topics..."
    
    # Start micro-ROS agent
    echo "Starting micro-ROS agent..."
    ros2 run micro_ros_agent micro_ros_agent serial --dev "$SERIAL_PORT" -b 115200 > /tmp/microros_agent.log 2>&1 &
    AGENT_PID=$!
    
    # Wait for connection
    sleep 5
    
    # Test for expected topics
    echo -n "  Checking for pico_count topic... "
    if timeout 10 ros2 topic echo /pico_count --once > /dev/null 2>&1; then
        echo -e "${GREEN}OK${NC}"
    else
        echo -e "${RED}MISSING${NC}"
    fi
    
    echo -n "  Checking for odometry topic... "
    if timeout 10 ros2 topic echo /odom --once > /dev/null 2>&1; then
        echo -e "${GREEN}OK${NC}"
    else
        echo -e "${RED}MISSING${NC}"
    fi
    
    echo -n "  Testing cmd_vel subscription... "
    if timeout 5 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once > /dev/null 2>&1; then
        echo -e "${GREEN}OK${NC}"
    else
        echo -e "${RED}FAILED${NC}"
    fi
    
    # Cleanup
    kill $AGENT_PID 2>/dev/null || true
    sleep 2
}

test_sensor_data() {
    echo "Testing sensor data..."
    
    # Start micro-ROS agent
    ros2 run micro_ros_agent micro_ros_agent serial --dev "$SERIAL_PORT" -b 115200 > /tmp/microros_sensor.log 2>&1 &
    AGENT_PID=$!
    
    sleep 5
    
    # Check for sensor topics
    echo -n "  IMU data... "
    if timeout 10 ros2 topic echo /imu/data --once > /dev/null 2>&1; then
        echo -e "${GREEN}OK${NC}"
    else
        echo -e "${YELLOW}NO DATA${NC}"
    fi
    
    echo -n "  Range sensors... "
    if timeout 10 ros2 topic list | grep -q range; then
        echo -e "${GREEN}DETECTED${NC}"
    else
        echo -e "${YELLOW}NO TOPICS${NC}"
    fi
    
    # Cleanup
    kill $AGENT_PID 2>/dev/null || true
}

# Main test sequence
echo "Starting firmware connection tests..."
echo

# Basic hardware tests
test_serial_device || exit 1
test_uart_debug

echo

# micro-ROS tests
if test_microros_agent; then
    echo
    test_pico_topics
    echo
    test_sensor_data
else
    echo -e "${RED}micro-ROS agent test failed - skipping topic tests${NC}"
    exit 1
fi

echo
echo -e "${GREEN}=== Firmware connection test completed ===${NC}"
echo "Check /tmp/microros_*.log for detailed agent logs"