#!/bin/bash
# Comprehensive Firmware Test Suite
# Tests firmware functionality automatically

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PASSED=0
FAILED=0
WARNINGS=0

# Test result tracking
declare -a FAILED_TESTS
declare -a WARNING_TESTS

# Helper functions
pass() {
    echo -e "${GREEN}✓ PASS${NC}: $1"
    PASSED=$((PASSED + 1))
}

fail() {
    echo -e "${RED}✗ FAIL${NC}: $1"
    FAILED=$((FAILED + 1))
    FAILED_TESTS+=("$1")
}

warn() {
    echo -e "${YELLOW}⚠ WARN${NC}: $1"
    WARNINGS=$((WARNINGS + 1))
    WARNING_TESTS+=("$1")
}

info() {
    echo -e "${BLUE}ℹ INFO${NC}: $1"
}

section() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
}

# Capture firmware output for analysis
capture_firmware_output() {
    local duration=$1
    local output_file="/tmp/firmware_output.txt"
    
    info "Capturing firmware output for ${duration}s..."
    
    # Find device
    DEVICE=$(ls /dev/ttyACM* 2>/dev/null | head -1)
    if [ -z "$DEVICE" ]; then
        fail "No Pico device found"
        return 1
    fi
    
    # Capture output using timeout and cat
    timeout ${duration} cat ${DEVICE} > ${output_file} 2>&1 &
    local PID=$!
    sleep ${duration}
    kill $PID 2>/dev/null || true
    
    echo ${output_file}
}

section "Firmware Comprehensive Test Suite"

# TEST 1: USB Connection
section "TEST 1: USB Device Detection"
if ls /dev/ttyACM* &>/dev/null; then
    DEVICE=$(ls /dev/ttyACM* | head -1)
    pass "Pico found at ${DEVICE}"
else
    fail "No Pico device found on USB"
    echo ""
    echo "Cannot continue without device. Exiting."
    exit 1
fi

# TEST 2: micro-ROS Agent
section "TEST 2: micro-ROS Agent Status"
if pgrep -f "micro_ros_agent" > /dev/null; then
    AGENT_PID=$(pgrep -f "micro_ros_agent")
    pass "micro-ROS agent running (PID: ${AGENT_PID})"
else
    fail "micro-ROS agent NOT running"
    warn "Start with: ros2 run micro_ros_agent micro_ros_agent serial --dev ${DEVICE}"
fi

# TEST 3: Firmware Output Analysis
section "TEST 3: Firmware Boot Sequence"
info "Capturing firmware serial output..."

# Capture 10 seconds of output
OUTPUT_FILE=$(capture_firmware_output 10)

if [ ! -f "$OUTPUT_FILE" ] || [ ! -s "$OUTPUT_FILE" ]; then
    fail "Could not capture firmware output"
else
    pass "Firmware output captured"
    
    # Test 3.1: Boot message
    if grep -q "Booting.*firmware" "$OUTPUT_FILE"; then
        pass "Firmware boot message found"
    else
        fail "No firmware boot message"
    fi
    
    # Test 3.2: FreeRTOS start
    if grep -q "Starting FreeRTOS" "$OUTPUT_FILE"; then
        pass "FreeRTOS started"
    else
        fail "FreeRTOS not started"
    fi
    
    # Test 3.3: Agent startup sequence
    if grep -q "Starting BlinkAgent" "$OUTPUT_FILE"; then
        pass "BlinkAgent started"
    else
        warn "BlinkAgent not found in output"
    fi
    
    if grep -q "Starting MotorsAgent" "$OUTPUT_FILE"; then
        pass "MotorsAgent started"
    else
        fail "MotorsAgent NOT started - CRITICAL!"
        warn "This means motors will not work"
    fi
    
    if grep -q "Starting HCSR04Agent" "$OUTPUT_FILE"; then
        pass "HCSR04Agent started"
    else
        warn "HCSR04Agent not found"
    fi
    
    if grep -q "Starting.*ImuAgent" "$OUTPUT_FILE"; then
        pass "ImuAgent started"
    else
        fail "ImuAgent NOT started"
    fi
    
    if grep -q "Starting DDD Agent" "$OUTPUT_FILE"; then
        pass "DDD Agent started"
    else
        fail "DDD Agent NOT started"
    fi
    
    if grep -q "Starting uRosBridge" "$OUTPUT_FILE"; then
        pass "uRosBridge started"
    else
        fail "uRosBridge NOT started"
    fi
    
    # Test 3.4: micro-ROS connection
    if grep -q "AGENT_CONNECTED" "$OUTPUT_FILE"; then
        pass "micro-ROS agent connected"
    else
        fail "micro-ROS agent NOT connected"
    fi
    
    # Test 3.5: Entity creation
    if grep -q "Creating ROS2 entities" "$OUTPUT_FILE"; then
        pass "ROS2 entities creation started"
    else
        fail "No entity creation found"
    fi
    
    if grep -q "Entities Created" "$OUTPUT_FILE"; then
        pass "ROS2 entities created successfully"
    else
        fail "Entity creation did not complete"
    fi
    
    # Test 3.6: Check for errors
    ERROR_COUNT=$(grep -c "ERROR\|FAILED\|Failed" "$OUTPUT_FILE" || echo "0")
    if [ "$ERROR_COUNT" -gt 5 ]; then
        fail "High error count: ${ERROR_COUNT} errors found"
    elif [ "$ERROR_COUNT" -gt 0 ]; then
        warn "${ERROR_COUNT} errors found in output"
    else
        pass "No errors in firmware output"
    fi
fi

# TEST 4: ROS2 Topics
section "TEST 4: ROS2 Topic Availability"

# Wait a bit for topics to appear
sleep 2

# Test 4.1: joint_states
if ros2 topic list 2>/dev/null | grep -q "^/joint_states$"; then
    pass "/joint_states topic exists"
    
    # Test 4.1.1: Can we read it?
    if timeout 2 ros2 topic echo /joint_states --once &>/dev/null; then
        pass "/joint_states is publishing data"
    else
        fail "/joint_states exists but no data"
    fi
else
    fail "/joint_states topic NOT found - MotorsAgent problem!"
fi

# Test 4.2: cmd_vel
if ros2 topic list 2>/dev/null | grep -q "^/cmd_vel$"; then
    pass "/cmd_vel topic exists"
else
    fail "/cmd_vel topic NOT found"
fi

# Test 4.3: odom
if ros2 topic list 2>/dev/null | grep -q "^/odom$"; then
    pass "/odom topic exists"
else
    fail "/odom topic NOT found"
fi

# Test 4.4: imu/data_raw
if ros2 topic list 2>/dev/null | grep -q "^/imu/data_raw$"; then
    pass "/imu/data_raw topic exists"
else
    fail "/imu/data_raw topic NOT found"
fi

# TEST 5: Joint States Content
section "TEST 5: Joint States Message Content"

if ros2 topic list 2>/dev/null | grep -q "^/joint_states$"; then
    JOINT_MSG=$(timeout 3 ros2 topic echo /joint_states --once 2>/dev/null)
    
    if [ -n "$JOINT_MSG" ]; then
        pass "Joint states message received"
        
        # Test 5.1: Check for front_right_wheel_joint
        if echo "$JOINT_MSG" | grep -q "front_right_wheel_joint"; then
            pass "front_right_wheel_joint found"
        else
            fail "front_right_wheel_joint NOT found"
        fi
        
        # Test 5.2: Check for position data
        if echo "$JOINT_MSG" | grep -q "position:"; then
            pass "Position data present"
        else
            fail "No position data"
        fi
        
        # Test 5.3: Check for velocity data
        if echo "$JOINT_MSG" | grep -q "velocity:"; then
            pass "Velocity data present"
        else
            fail "No velocity data"
        fi
        
        # Test 5.4: Count joints
        JOINT_COUNT=$(echo "$JOINT_MSG" | grep -c "wheel_joint" || echo "0")
        if [ "$JOINT_COUNT" -ge 1 ]; then
            pass "Found ${JOINT_COUNT} wheel joint(s)"
            if [ "$JOINT_COUNT" -lt 4 ]; then
                warn "Only ${JOINT_COUNT}/4 motors configured"
            fi
        else
            fail "No wheel joints found"
        fi
    else
        fail "Could not read joint states message"
    fi
else
    warn "Skipping joint states content test (topic not available)"
fi

# TEST 6: Publishing Rate
section "TEST 6: Topic Publishing Rates"

if ros2 topic list 2>/dev/null | grep -q "^/joint_states$"; then
    info "Measuring /joint_states rate (5 seconds)..."
    RATE=$(timeout 5 ros2 topic hz /joint_states 2>&1 | grep "average rate" | tail -1 | awk '{print $3}')
    
    if [ -n "$RATE" ]; then
        if (( $(echo "$RATE >= 8.0" | bc -l) )); then
            pass "Publishing rate: ${RATE} Hz (good)"
        elif (( $(echo "$RATE >= 4.0" | bc -l) )); then
            warn "Publishing rate: ${RATE} Hz (acceptable but low)"
        else
            fail "Publishing rate: ${RATE} Hz (too low!)"
        fi
    else
        fail "Could not measure publishing rate"
    fi
else
    warn "Skipping rate test (topic not available)"
fi

# TEST 7: Motor Response Test
section "TEST 7: Motor Command Response"

if ros2 topic list 2>/dev/null | grep -q "^/cmd_vel$" && \
   ros2 topic list 2>/dev/null | grep -q "^/joint_states$"; then
    
    info "Testing motor response to cmd_vel..."
    
    # Get initial position
    INITIAL_POS=$(timeout 2 ros2 topic echo /joint_states --once --field "position[1]" 2>/dev/null)
    
    if [ -n "$INITIAL_POS" ]; then
        info "Initial position: ${INITIAL_POS}"
        
        # Send command
        ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
            "{linear: {x: 0.05, y: 0.0, z: 0.0}}" \
            --rate 10 &
        CMD_PID=$!
        
        sleep 3
        
        # Get final position
        FINAL_POS=$(timeout 2 ros2 topic echo /joint_states --once --field "position[1]" 2>/dev/null)
        
        # Stop command
        kill $CMD_PID 2>/dev/null || true
        ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
            "{linear: {x: 0.0, y: 0.0, z: 0.0}}" --once &>/dev/null
        
        if [ -n "$FINAL_POS" ]; then
            info "Final position: ${FINAL_POS}"
            
            if [ "$INITIAL_POS" != "$FINAL_POS" ]; then
                pass "Motor responded to cmd_vel (position changed)"
            else
                fail "Motor did NOT respond (position unchanged)"
                warn "Check motor connections and power"
            fi
        else
            fail "Could not read final position"
        fi
    else
        fail "Could not read initial position"
    fi
else
    warn "Skipping motor response test (topics not available)"
fi

# TEST 8: Controller Manager
section "TEST 8: ROS2 Control System"

if ros2 node list 2>/dev/null | grep -q "controller_manager"; then
    pass "Controller Manager is running"
    
    # Test 8.1: Hardware interfaces
    if ros2 control list_hardware_interfaces 2>/dev/null | grep -q "front_right_wheel_joint"; then
        pass "Hardware interfaces available"
    else
        fail "Hardware interfaces NOT available"
    fi
    
    # Test 8.2: Controllers
    CONTROLLER_COUNT=$(ros2 control list_controllers 2>/dev/null | grep -c "active" || echo "0")
    if [ "$CONTROLLER_COUNT" -gt 0 ]; then
        pass "${CONTROLLER_COUNT} controller(s) active"
    else
        warn "No active controllers"
    fi
else
    warn "Controller Manager not running (optional for firmware test)"
fi

# FINAL SUMMARY
section "Test Summary"

echo "Results:"
echo -e "  ${GREEN}Passed:${NC}   ${PASSED}"
echo -e "  ${RED}Failed:${NC}   ${FAILED}"
echo -e "  ${YELLOW}Warnings:${NC} ${WARNINGS}"
echo ""

if [ ${FAILED} -eq 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  ALL TESTS PASSED! ✓${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo "Firmware is working correctly!"
    echo ""
    echo "Next steps:"
    echo "  - Test motor movement: bash test_front_right_motor.sh"
    echo "  - Connect remaining motors"
    echo "  - Test full system"
    EXIT_CODE=0
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}  TESTS FAILED! ✗${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo "Failed tests:"
    for test in "${FAILED_TESTS[@]}"; do
        echo -e "  ${RED}✗${NC} $test"
    done
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check firmware output: screen ${DEVICE} 115200"
    echo "  2. Rebuild and reflash: bash rebuild_and_flash.sh"
    echo "  3. Check micro-ROS agent is running"
    echo "  4. Check motor connections"
    EXIT_CODE=1
fi

if [ ${WARNINGS} -gt 0 ]; then
    echo ""
    echo "Warnings:"
    for test in "${WARNING_TESTS[@]}"; do
        echo -e "  ${YELLOW}⚠${NC} $test"
    done
fi

echo ""
echo "Detailed firmware output saved to: ${OUTPUT_FILE}"
echo ""

exit ${EXIT_CODE}
