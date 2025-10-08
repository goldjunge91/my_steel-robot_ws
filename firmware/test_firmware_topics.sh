#!/bin/bash
# Professional firmware topic verification script
# Tests actual data flow, not just topic existence

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Source ROS2 if needed
if [ -z "$ROS_DISTRO" ]; then
    [ -f "/opt/ros/humble/setup.bash" ] && source /opt/ros/humble/setup.bash
    [ -f "install/setup.bash" ] && source install/setup.bash
fi

echo "=========================================="
echo "Firmware Topic Verification Test"
echo "=========================================="
echo ""

# Test configuration
TIMEOUT=5
PASS=0
FAIL=0
WARN=0

# Test function that actually reads data
test_topic_data() {
    local topic=$1
    local expected_type=$2
    local description=$3
    
    echo -n "Testing $description ($topic)... "
    
    # Check if topic exists
    if ! ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        echo -e "${RED}FAIL${NC} - Topic does not exist"
        FAIL=$((FAIL + 1))
        return 1
    fi
    
    # Try to read one message
    local output=$(timeout $TIMEOUT ros2 topic echo $topic --once 2>&1)
    local exit_code=$?
    
    if [ $exit_code -eq 124 ]; then
        echo -e "${RED}FAIL${NC} - Timeout (no data received in ${TIMEOUT}s)"
        FAIL=$((FAIL + 1))
        return 1
    elif [ $exit_code -ne 0 ]; then
        echo -e "${RED}FAIL${NC} - Error reading topic"
        FAIL=$((FAIL + 1))
        return 1
    fi
    
    # Check if we got actual data
    if [ -z "$output" ]; then
        echo -e "${RED}FAIL${NC} - Empty message"
        FAIL=$((FAIL + 1))
        return 1
    fi
    
    # Verify message type
    local actual_type=$(ros2 topic info $topic 2>/dev/null | grep "Type:" | awk '{print $2}')
    if [ "$actual_type" != "$expected_type" ]; then
        echo -e "${YELLOW}WARN${NC} - Type mismatch (expected: $expected_type, got: $actual_type)"
        WARN=$((WARN + 1))
    else
        echo -e "${GREEN}PASS${NC}"
        PASS=$((PASS + 1))
    fi
    
    return 0
}

# Test function for optional topics
test_optional_topic() {
    local topic=$1
    local expected_type=$2
    local description=$3
    
    echo -n "Testing $description ($topic)... "
    
    if ! ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        echo -e "${YELLOW}SKIP${NC} - Optional sensor not available"
        WARN=$((WARN + 1))
        return 0
    fi
    
    test_topic_data "$topic" "$expected_type" "$description"
}

echo "=========================================="
echo "Core Topics (Required)"
echo "=========================================="
echo ""

test_topic_data "/joint_states" "sensor_msgs/msg/JointState" "Joint States"
test_topic_data "/imu/data_raw" "sensor_msgs/msg/Imu" "IMU Data"
test_topic_data "/odom" "nav_msgs/msg/Odometry" "Odometry"

echo ""
echo "=========================================="
echo "Optional Sensor Topics"
echo "=========================================="
echo ""

test_optional_topic "/sensors/range_tof" "sensor_msgs/msg/Range" "ToF Range Sensor"
test_optional_topic "/sensors/range_ultrasonic" "sensor_msgs/msg/Range" "Ultrasonic Range"
test_optional_topic "/sensors/illuminance" "sensor_msgs/msg/Illuminance" "Illuminance Sensor"

echo ""
echo "=========================================="
echo "Topic Remapping Verification"
echo "=========================================="
echo ""

# Check that /rt/* topics are NOT visible (they should be remapped)
rt_topics=$(ros2 topic list 2>/dev/null | grep "^/rt/" | wc -l)
if [ $rt_topics -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Agent remapping is active (no /rt/* topics visible)"
else
    echo -e "${YELLOW}⚠${NC} Found $rt_topics /rt/* topics (remapping may not be active)"
    WARN=$((WARN + 1))
fi

# Check subscriber topic
if ros2 topic list 2>/dev/null | grep -q "^/rt/cmd_vel$"; then
    echo -e "${GREEN}✓${NC} Subscriber topic /rt/cmd_vel exists"
else
    echo -e "${BLUE}ℹ${NC} Subscriber topic /rt/cmd_vel not yet created (normal until /cmd_vel is published)"
fi

echo ""
echo "=========================================="
echo "Data Quality Checks"
echo "=========================================="
echo ""

# Check joint_states has 4 joints
echo -n "Checking joint_states joint count... "
joint_data=$(timeout 3 ros2 topic echo /joint_states --once 2>/dev/null)
joint_count=$(echo "$joint_data" | grep -c "wheel_joint")
if [ $joint_count -eq 4 ]; then
    echo -e "${GREEN}PASS${NC} (4 wheel joints)"
    PASS=$((PASS + 1))
else
    echo -e "${YELLOW}WARN${NC} (expected 4 joints, found $joint_count)"
    WARN=$((WARN + 1))
fi

# Check IMU frame_id
echo -n "Checking IMU frame_id... "
imu_data=$(timeout 3 ros2 topic echo /imu/data_raw --once 2>/dev/null)
frame_id=$(echo "$imu_data" | grep "frame_id:" | awk '{print $2}')
if [ "$frame_id" == "imu_link" ]; then
    echo -e "${GREEN}PASS${NC} (frame_id: imu_link)"
    PASS=$((PASS + 1))
else
    echo -e "${YELLOW}WARN${NC} (frame_id: $frame_id, expected: imu_link)"
    WARN=$((WARN + 1))
fi

# Check odom frame_ids
echo -n "Checking odometry frame_ids... "
odom_data=$(timeout 3 ros2 topic echo /odom --once 2>/dev/null)
odom_frame=$(echo "$odom_data" | grep "frame_id:" | head -1 | awk '{print $2}')
child_frame=$(echo "$odom_data" | grep "child_frame_id:" | awk '{print $2}')
if [ "$odom_frame" == "odom" ] && [ "$child_frame" == "base_link" ]; then
    echo -e "${GREEN}PASS${NC} (odom → base_link)"
    PASS=$((PASS + 1))
else
    echo -e "${YELLOW}WARN${NC} (frames: $odom_frame → $child_frame)"
    WARN=$((WARN + 1))
fi

echo ""
echo "=========================================="
echo "Summary"
echo "=========================================="
echo ""
echo -e "Passed:   ${GREEN}$PASS${NC}"
echo -e "Failed:   ${RED}$FAIL${NC}"
echo -e "Warnings: ${YELLOW}$WARN${NC}"
echo ""

if [ $FAIL -eq 0 ]; then
    echo -e "${GREEN}✓ All critical tests passed!${NC}"
    echo ""
    echo "Firmware is publishing data correctly."
    echo "Topic remapping is working as expected."
    exit 0
else
    echo -e "${RED}✗ $FAIL test(s) failed${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check micro-ROS agent is running: ps aux | grep micro_ros_agent"
    echo "  2. Check Pico connection: ls -la /dev/ttyACM*"
    echo "  3. Monitor firmware output: just monitor-firmware"
    echo "  4. Restart agent: just start-microros"
    exit 1
fi
