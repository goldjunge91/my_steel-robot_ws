#!/bin/bash
# Test script to verify firmware topic names after standardization
# This script checks that all topics use standard ROS2 naming conventions
# Requirements: 1.1, 1.2, 1.3, 1.4, 1.5

set +e  # Don't exit on errors, we want to count them

echo "=========================================="
echo "  Firmware Topic Standardization Test"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test results
PASSED=0
FAILED=0

# Function to check if a topic exists
check_topic_exists() {
    local topic_name=$1
    local requirement=$2
    
    echo -n "Checking topic '$topic_name'... "
    
    if ros2 topic list | grep -q "^${topic_name}$"; then
        echo -e "${GREEN}✓ PASS${NC} (Requirement: $requirement)"
        ((PASSED++))
        return 0
    else
        echo -e "${RED}✗ FAIL${NC} - Topic not found (Requirement: $requirement)"
        ((FAILED++))
        return 1
    fi
}

# Function to check if a deprecated topic does NOT exist
check_topic_not_exists() {
    local topic_name=$1
    local requirement=$2
    
    echo -n "Checking deprecated topic '$topic_name' is removed... "
    
    if ros2 topic list | grep -q "^${topic_name}$"; then
        echo -e "${RED}✗ FAIL${NC} - Deprecated topic still exists (Requirement: $requirement)"
        ((FAILED++))
        return 1
    else
        echo -e "${GREEN}✓ PASS${NC} (Requirement: $requirement)"
        ((PASSED++))
        return 0
    fi
}

# Function to echo a topic and verify it's publishing data
check_topic_data() {
    local topic_name=$1
    local requirement=$2
    local timeout=5
    
    echo -n "Checking topic '$topic_name' is publishing data... "
    
    # Use timeout to wait for at least one message
    if timeout $timeout ros2 topic echo "$topic_name" --once > /dev/null 2>&1; then
        echo -e "${GREEN}✓ PASS${NC} - Data received (Requirement: $requirement)"
        ((PASSED++))
        return 0
    else
        echo -e "${RED}✗ FAIL${NC} - No data received within ${timeout}s (Requirement: $requirement)"
        ((FAILED++))
        return 1
    fi
}

echo "Step 1: Checking if micro-ROS agent is running..."
echo "----------------------------------------------"
if ! pgrep -f "micro_ros_agent" > /dev/null; then
    echo -e "${YELLOW}WARNING:${NC} micro-ROS agent does not appear to be running"
    echo "Please start the micro-ROS agent first:"
    echo "  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0"
    echo ""
    read -p "Press Enter to continue anyway, or Ctrl+C to exit..."
fi
echo ""

echo "Step 2: Listing all available topics..."
echo "----------------------------------------------"
ros2 topic list
echo ""

echo "Step 3: Checking standard topic names exist..."
echo "----------------------------------------------"
# Check odometry topic (Requirement 1.4)
check_topic_exists "/odom" "1.4"

# Check cmd_vel topic (Requirement 1.6)
check_topic_exists "/cmd_vel" "1.6"

# Check IMU topic (Requirement 1.3)
check_topic_exists "/imu/data_raw" "1.3"

# Check sensor topics (Requirement 1.5)
check_topic_exists "/sensors/range_tof" "1.5"
check_topic_exists "/sensors/illuminance" "1.5"
check_topic_exists "/sensors/range_ultrasonic" "1.5"

# Check joint states topic (Requirement 1.2)
check_topic_exists "/joint_states" "1.2"
echo ""

echo "Step 4: Checking deprecated topic names are removed..."
echo "----------------------------------------------"
# Check that old /ddd/ prefixed topics don't exist (Requirement 1.7)
check_topic_not_exists "/ddd/odom" "1.7"
check_topic_not_exists "/ddd/imu" "1.7"
check_topic_not_exists "/ddd/range_tof" "1.7"
check_topic_not_exists "/ddd/range" "1.7"
check_topic_not_exists "/ddd/illuminance" "1.7"
check_topic_not_exists "/ddd/cmd_vel" "1.7"
echo ""

echo "Step 5: Verifying topics are publishing data..."
echo "----------------------------------------------"
echo "This step will wait up to 5 seconds for each topic to publish data."
echo ""

# Check that topics are actually publishing data
check_topic_data "/odom" "1.4"
check_topic_data "/imu/data_raw" "1.3"
check_topic_data "/joint_states" "1.2"
check_topic_data "/sensors/range_tof" "1.5"
check_topic_data "/sensors/illuminance" "1.5"
check_topic_data "/sensors/range_ultrasonic" "1.5"
echo ""

echo "Step 6: Checking topic message types..."
echo "----------------------------------------------"
echo "Odometry topic type:"
ros2 topic info /odom | grep "Type:"

echo "IMU topic type:"
ros2 topic info /imu/data_raw | grep "Type:"

echo "Joint states topic type:"
ros2 topic info /joint_states | grep "Type:"

echo "Range ToF topic type:"
ros2 topic info /sensors/range_tof | grep "Type:"

echo "Illuminance topic type:"
ros2 topic info /sensors/illuminance | grep "Type:"

echo "Range ultrasonic topic type:"
ros2 topic info /sensors/range_ultrasonic | grep "Type:"
echo ""

echo "=========================================="
echo "  Test Summary"
echo "=========================================="
echo -e "Tests Passed: ${GREEN}${PASSED}${NC}"
echo -e "Tests Failed: ${RED}${FAILED}${NC}"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All tests passed!${NC}"
    echo "The firmware topic standardization is complete and working correctly."
    exit 0
else
    echo -e "${RED}✗ Some tests failed.${NC}"
    echo "Please review the failures above and ensure:"
    echo "  1. The firmware has been flashed to the Pico"
    echo "  2. The micro-ROS agent is running"
    echo "  3. The Pico is connected and communicating"
    exit 1
fi
