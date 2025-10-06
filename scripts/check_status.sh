#!/bin/bash
# Robot Status Check Script

echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║                    ROBOT STATUS CHECK                            ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

check_service() {
    local name=$1
    local check_cmd=$2
    
    if eval "$check_cmd" &>/dev/null; then
        echo -e "  ${GREEN}✓${NC} $name"
        return 0
    else
        echo -e "  ${RED}✗${NC} $name"
        return 1
    fi
}

check_topic() {
    local name=$1
    local topic=$2
    
    if ros2 topic info "$topic" &>/dev/null; then
        hz=$(timeout 2 ros2 topic hz "$topic" 2>&1 | grep "average rate" | awk '{print $3}')
        if [ -n "$hz" ]; then
            echo -e "  ${GREEN}✓${NC} $name (${hz} Hz)"
        else
            echo -e "  ${YELLOW}⚠${NC} $name (topic exists but no data)"
        fi
        return 0
    else
        echo -e "  ${RED}✗${NC} $name"
        return 1
    fi
}

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "ROS2 Environment"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-NOT SET}"
echo "  RMW: ${RMW_IMPLEMENTATION:-NOT SET}"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Hardware Connections"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
check_service "Raspberry Pi Pico (USB)" "ls /dev/ttyACM* 2>/dev/null"
check_service "Camera Device" "ls /dev/v4l/by-id/* 2>/dev/null"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "ROS2 Nodes"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
check_service "Micro-ROS Agent" "ros2 node list | grep -q micro_ros_agent"
check_service "Robot State Publisher" "ros2 node list | grep -q robot_state_publisher"
check_service "Controller Manager" "ros2 node list | grep -q controller_manager"
check_service "Camera Node" "ros2 node list | grep -q camera"
check_service "Foxglove Bridge" "ros2 node list | grep -q foxglove_bridge"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Sensor Topics"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
check_topic "Camera Image" "/camera/image_raw"
check_topic "IMU Data" "/imu/data_raw"
check_topic "ToF Sensor" "/range"
check_topic "Joint States" "/joint_states"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Controller Topics"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
check_topic "Cmd Vel" "/diff_drive_controller/cmd_vel_unstamped"
check_topic "Odom" "/diff_drive_controller/odom"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Network Services"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
check_service "Foxglove WebSocket (Port 8765)" "netstat -tuln | grep -q :8765"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Complete Node List:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
ros2 node list 2>/dev/null || echo "  No nodes found"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Complete Topic List:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
ros2 topic list 2>/dev/null || echo "  No topics found"
echo ""

echo "╚══════════════════════════════════════════════════════════════════╝"
