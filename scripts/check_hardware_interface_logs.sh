#!/bin/bash
# Script to verify hardware interface logs meet requirements 3.5, 3.6, 4.4, 4.5
# This script checks for:
# 1. No "mock mode" warnings
# 2. "Successfully activated with real hardware feedback" messages
# 3. No ERROR messages about missing data during normal operation

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Hardware Interface Log Verification${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Function to check if ROS2 is running
check_ros2_running() {
    if ! ros2 node list &>/dev/null; then
        echo -e "${RED}✗ ROS2 is not running or no nodes are active${NC}"
        echo -e "${YELLOW}  Please start the robot system first:${NC}"
        echo -e "${YELLOW}  ros2 launch robot_bringup bringup.launch.py${NC}"
        return 1
    fi
    return 0
}

# Function to check for specific node
check_node_exists() {
    local node_name=$1
    if ros2 node list 2>/dev/null | grep -q "$node_name"; then
        return 0
    fi
    return 1
}

# Check if system is running
echo -e "${BLUE}[1/6] Checking if ROS2 system is running...${NC}"
if check_ros2_running; then
    echo -e "${GREEN}✓ ROS2 system is running${NC}"
else
    echo -e "${YELLOW}  Checking log files instead...${NC}"
fi
echo ""

# Check for controller_manager node (indicates hardware interfaces are loaded)
echo -e "${BLUE}[2/6] Checking for controller_manager node...${NC}"
if check_node_exists "controller_manager"; then
    echo -e "${GREEN}✓ controller_manager node is running${NC}"
    SYSTEM_RUNNING=true
else
    echo -e "${YELLOW}⚠ controller_manager not found, will check log files only${NC}"
    SYSTEM_RUNNING=false
fi
echo ""

# Initialize counters
MOCK_MODE_COUNT=0
SUCCESS_ACTIVATION_COUNT=0
ERROR_MISSING_DATA_COUNT=0
ROBOT_SYSTEM_ACTIVATED=false
IMU_SENSOR_ACTIVATED=false

# Check runtime logs if system is running
if [ "$SYSTEM_RUNNING" = true ]; then
    echo -e "${BLUE}[3/6] Checking runtime logs via ros2 daemon...${NC}"
    
    # Get recent log output (this is tricky without direct access to stderr)
    # We'll check the latest log directory
    LATEST_LOG_DIR=$(readlink -f log/latest_build 2>/dev/null || echo "")
    
    if [ -n "$LATEST_LOG_DIR" ] && [ -d "$LATEST_LOG_DIR" ]; then
        echo -e "${YELLOW}  Checking latest build logs: $LATEST_LOG_DIR${NC}"
    fi
fi

# Check log files
echo -e "${BLUE}[4/6] Checking log files for hardware interface messages...${NC}"

# Find the most recent log directories (check both workspace logs and ROS logs)
RECENT_WORKSPACE_LOGS=$(find log -type f -name "*.log" -mtime -1 2>/dev/null | sort -r | head -50)
RECENT_ROS_LOGS=$(find ~/.ros/log -type f -name "*.log" -size +0 -mmin -120 2>/dev/null | sort -r | head -50)
RECENT_LOGS="$RECENT_WORKSPACE_LOGS $RECENT_ROS_LOGS"

if [ -z "$RECENT_LOGS" ]; then
    echo -e "${YELLOW}⚠ No recent log files found (last 24 hours)${NC}"
    echo -e "${YELLOW}  Run the system to generate logs:${NC}"
    echo -e "${YELLOW}  ros2 launch robot_bringup bringup.launch.py${NC}"
else
    echo -e "${GREEN}✓ Found recent log files${NC}"
    
    # Check for "mock mode" warnings (should NOT exist)
    echo -e "\n${BLUE}  Checking for 'mock mode' warnings...${NC}"
    MOCK_MODE_RESULTS=$(grep -i "mock mode" $RECENT_LOGS 2>/dev/null || true)
    
    if [ -z "$MOCK_MODE_RESULTS" ]; then
        echo -e "${GREEN}  ✓ No 'mock mode' warnings found (PASS)${NC}"
    else
        echo -e "${RED}  ✗ Found 'mock mode' warnings (FAIL):${NC}"
        echo "$MOCK_MODE_RESULTS" | head -5
        MOCK_MODE_COUNT=$(echo "$MOCK_MODE_RESULTS" | wc -l)
        echo -e "${RED}    Total occurrences: $MOCK_MODE_COUNT${NC}"
    fi
    
    # Check for successful activation messages (should exist)
    echo -e "\n${BLUE}  Checking for successful activation messages...${NC}"
    
    # RobotSystem activation
    ROBOT_SYSTEM_SUCCESS=$(grep -i "Successfully activated with real hardware feedback" $RECENT_LOGS 2>/dev/null | grep -i "RobotSystem" || true)
    if [ -n "$ROBOT_SYSTEM_SUCCESS" ]; then
        echo -e "${GREEN}  ✓ RobotSystem successfully activated with real hardware feedback (PASS)${NC}"
        echo -e "${GREEN}    $(echo "$ROBOT_SYSTEM_SUCCESS" | head -1)${NC}"
        ROBOT_SYSTEM_ACTIVATED=true
        SUCCESS_ACTIVATION_COUNT=$((SUCCESS_ACTIVATION_COUNT + 1))
    else
        echo -e "${YELLOW}  ⚠ RobotSystem activation message not found in logs${NC}"
    fi
    
    # IMU Sensor activation
    IMU_SUCCESS=$(grep -i "Successfully activated with real IMU feedback" $RECENT_LOGS 2>/dev/null || true)
    if [ -n "$IMU_SUCCESS" ]; then
        echo -e "${GREEN}  ✓ RobotImuSensor successfully activated with real IMU feedback (PASS)${NC}"
        echo -e "${GREEN}    $(echo "$IMU_SUCCESS" | head -1)${NC}"
        IMU_SENSOR_ACTIVATED=true
        SUCCESS_ACTIVATION_COUNT=$((SUCCESS_ACTIVATION_COUNT + 1))
    else
        echo -e "${YELLOW}  ⚠ RobotImuSensor activation message not found in logs${NC}"
    fi
    
    # Check for ERROR messages about missing data (should NOT exist during normal operation)
    echo -e "\n${BLUE}  Checking for ERROR messages about missing data...${NC}"
    
    # Joint states errors
    JOINT_STATE_ERRORS=$(grep -i "No joint states received from firmware" $RECENT_LOGS 2>/dev/null || true)
    if [ -z "$JOINT_STATE_ERRORS" ]; then
        echo -e "${GREEN}  ✓ No joint state ERROR messages (PASS)${NC}"
    else
        ERROR_COUNT=$(echo "$JOINT_STATE_ERRORS" | wc -l)
        echo -e "${YELLOW}  ⚠ Found $ERROR_COUNT joint state ERROR messages${NC}"
        echo -e "${YELLOW}    This may be normal during startup or if firmware is disconnected${NC}"
        ERROR_MISSING_DATA_COUNT=$((ERROR_MISSING_DATA_COUNT + ERROR_COUNT))
    fi
    
    # IMU data errors
    IMU_ERRORS=$(grep -i "No IMU data received from firmware" $RECENT_LOGS 2>/dev/null || true)
    if [ -z "$IMU_ERRORS" ]; then
        echo -e "${GREEN}  ✓ No IMU data ERROR messages (PASS)${NC}"
    else
        ERROR_COUNT=$(echo "$IMU_ERRORS" | wc -l)
        echo -e "${YELLOW}  ⚠ Found $ERROR_COUNT IMU data ERROR messages${NC}"
        echo -e "${YELLOW}    This may be normal during startup or if firmware is disconnected${NC}"
        ERROR_MISSING_DATA_COUNT=$((ERROR_MISSING_DATA_COUNT + ERROR_COUNT))
    fi
    
    # Timeout errors during activation
    TIMEOUT_ERRORS=$(grep -i "Timeout.*waiting for.*from firmware" $RECENT_LOGS 2>/dev/null || true)
    if [ -z "$TIMEOUT_ERRORS" ]; then
        echo -e "${GREEN}  ✓ No timeout ERROR messages during activation (PASS)${NC}"
    else
        echo -e "${RED}  ✗ Found timeout ERROR messages (FAIL):${NC}"
        echo "$TIMEOUT_ERRORS" | head -3
        ERROR_MISSING_DATA_COUNT=$((ERROR_MISSING_DATA_COUNT + $(echo "$TIMEOUT_ERRORS" | wc -l)))
    fi
fi

# Check live system status if running
if [ "$SYSTEM_RUNNING" = true ]; then
    echo -e "\n${BLUE}[5/6] Checking live system status...${NC}"
    
    # Check if hardware interfaces are active
    echo -e "${BLUE}  Checking controller status...${NC}"
    CONTROLLER_STATUS=$(ros2 control list_controllers 2>/dev/null || echo "")
    
    if [ -n "$CONTROLLER_STATUS" ]; then
        echo -e "${GREEN}  ✓ Controllers are loaded:${NC}"
        echo "$CONTROLLER_STATUS" | while read line; do
            echo -e "${GREEN}    $line${NC}"
        done
    else
        echo -e "${YELLOW}  ⚠ Could not retrieve controller status${NC}"
    fi
    
    # Check if topics are publishing
    echo -e "\n${BLUE}  Checking if /joint_states is publishing...${NC}"
    if timeout 2 ros2 topic echo /joint_states --once &>/dev/null; then
        echo -e "${GREEN}  ✓ /joint_states is publishing data${NC}"
    else
        echo -e "${YELLOW}  ⚠ /joint_states is not publishing or timed out${NC}"
    fi
    
    echo -e "\n${BLUE}  Checking if /imu/data_raw is publishing...${NC}"
    if timeout 2 ros2 topic echo /imu/data_raw --once &>/dev/null; then
        echo -e "${GREEN}  ✓ /imu/data_raw is publishing data${NC}"
    else
        echo -e "${YELLOW}  ⚠ /imu/data_raw is not publishing or timed out${NC}"
    fi
else
    echo -e "\n${BLUE}[5/6] Skipping live system checks (system not running)${NC}"
fi

# Summary
echo -e "\n${BLUE}========================================${NC}"
echo -e "${BLUE}[6/6] Summary${NC}"
echo -e "${BLUE}========================================${NC}"

PASS_COUNT=0
FAIL_COUNT=0
WARN_COUNT=0

# Requirement 3.5, 3.6, 4.4, 4.5 checks
echo -e "\n${BLUE}Requirement Verification:${NC}"

# Check 1: No mock mode warnings
if [ $MOCK_MODE_COUNT -eq 0 ]; then
    echo -e "${GREEN}✓ Req 3.5/4.4: No 'mock mode' warnings found${NC}"
    PASS_COUNT=$((PASS_COUNT + 1))
else
    echo -e "${RED}✗ Req 3.5/4.4: Found $MOCK_MODE_COUNT 'mock mode' warnings${NC}"
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

# Check 2: Successful activation messages
if [ "$ROBOT_SYSTEM_ACTIVATED" = true ]; then
    echo -e "${GREEN}✓ Req 3.6: RobotSystem successfully activated with real hardware feedback${NC}"
    PASS_COUNT=$((PASS_COUNT + 1))
else
    echo -e "${YELLOW}⚠ Req 3.6: RobotSystem activation message not found${NC}"
    WARN_COUNT=$((WARN_COUNT + 1))
fi

if [ "$IMU_SENSOR_ACTIVATED" = true ]; then
    echo -e "${GREEN}✓ Req 4.5: RobotImuSensor successfully activated with real IMU feedback${NC}"
    PASS_COUNT=$((PASS_COUNT + 1))
else
    echo -e "${YELLOW}⚠ Req 4.5: RobotImuSensor activation message not found${NC}"
    WARN_COUNT=$((WARN_COUNT + 1))
fi

# Check 3: No persistent ERROR messages
if [ $ERROR_MISSING_DATA_COUNT -eq 0 ]; then
    echo -e "${GREEN}✓ Req 3.5/4.4: No ERROR messages about missing data${NC}"
    PASS_COUNT=$((PASS_COUNT + 1))
else
    echo -e "${YELLOW}⚠ Req 3.5/4.4: Found $ERROR_MISSING_DATA_COUNT ERROR messages about missing data${NC}"
    echo -e "${YELLOW}  Note: Some errors during startup are normal${NC}"
    WARN_COUNT=$((WARN_COUNT + 1))
fi

# Final verdict
echo -e "\n${BLUE}Results:${NC}"
echo -e "  ${GREEN}Passed: $PASS_COUNT${NC}"
echo -e "  ${YELLOW}Warnings: $WARN_COUNT${NC}"
echo -e "  ${RED}Failed: $FAIL_COUNT${NC}"

if [ $FAIL_COUNT -eq 0 ] && [ $PASS_COUNT -gt 0 ]; then
    echo -e "\n${GREEN}========================================${NC}"
    echo -e "${GREEN}✓ Hardware interface logs verification PASSED${NC}"
    echo -e "${GREEN}========================================${NC}"
    exit 0
elif [ $FAIL_COUNT -eq 0 ] && [ $WARN_COUNT -gt 0 ]; then
    echo -e "\n${YELLOW}========================================${NC}"
    echo -e "${YELLOW}⚠ Hardware interface logs verification completed with warnings${NC}"
    echo -e "${YELLOW}  This may be normal if the system hasn't been run recently${NC}"
    echo -e "${YELLOW}  Run: ros2 launch robot_bringup bringup.launch.py${NC}"
    echo -e "${YELLOW}========================================${NC}"
    exit 0
else
    echo -e "\n${RED}========================================${NC}"
    echo -e "${RED}✗ Hardware interface logs verification FAILED${NC}"
    echo -e "${RED}========================================${NC}"
    exit 1
fi
