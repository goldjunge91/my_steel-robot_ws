#!/bin/bash
# Test script to verify Pico firmware topics are correctly published
# This script checks that all expected topics from the firmware are available

set -e

echo "=========================================="
echo "Pico Firmware Topic Verification Test"
echo "=========================================="
echo ""
echo "This script verifies that the Pico firmware is publishing"
echo "all expected topics with the /rt/ prefix added by micro-ROS agent."
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Expected firmware topics (with /rt/ prefix added by micro-ROS agent)
EXPECTED_TOPICS=(
    "/rt/joint_states"
    "/rt/imu/data_raw"
    "/rt/odom"
    "/rt/sensors/range_tof"
    "/rt/sensors/range_ultrasonic"
    "/rt/sensors/illuminance"
    "/rt/pico_rnd"
)

# Expected remapped topics (after agent remapping)
REMAPPED_TOPICS=(
    "/joint_states"
    "/imu/data_raw"
    "/odom"
    "/sensors/range_tof"
    "/sensors/range_ultrasonic"
    "/sensors/illuminance"
)

# Subscriber topics (firmware subscribes to these)
SUBSCRIBER_TOPICS=(
    "/rt/cmd_vel"
)

echo "Waiting 3 seconds for topics to stabilize..."
sleep 3

echo ""
echo "=========================================="
echo "1. Checking Raw Firmware Topics (/rt/*)"
echo "=========================================="
echo ""

all_topics=$(ros2 topic list 2>/dev/null)
failed=0

for topic in "${EXPECTED_TOPICS[@]}"; do
    if echo "$all_topics" | grep -q "^${topic}$"; then
        echo -e "${GREEN}✓${NC} Found: $topic"
    else
        echo -e "${RED}✗${NC} Missing: $topic"
        failed=1
    fi
done

echo ""
echo "=========================================="
echo "2. Checking Remapped Topics (Standard ROS2)"
echo "=========================================="
echo ""

for topic in "${REMAPPED_TOPICS[@]}"; do
    if echo "$all_topics" | grep -q "^${topic}$"; then
        echo -e "${GREEN}✓${NC} Found: $topic"
    else
        echo -e "${YELLOW}⚠${NC} Missing: $topic (check agent remapping)"
        failed=1
    fi
done

echo ""
echo "=========================================="
echo "3. Checking Subscriber Topics"
echo "=========================================="
echo ""

for topic in "${SUBSCRIBER_TOPICS[@]}"; do
    if echo "$all_topics" | grep -q "^${topic}$"; then
        echo -e "${GREEN}✓${NC} Found: $topic"
    else
        echo -e "${YELLOW}⚠${NC} Missing: $topic (will be created when published to)"
    fi
done

echo ""
echo "=========================================="
echo "4. Topic Data Verification"
echo "=========================================="
echo ""

echo "Checking /joint_states for data..."
timeout 2 ros2 topic echo /joint_states --once > /dev/null 2>&1 && \
    echo -e "${GREEN}✓${NC} /joint_states is publishing data" || \
    echo -e "${RED}✗${NC} /joint_states has no data"

echo "Checking /imu/data_raw for data..."
timeout 2 ros2 topic echo /imu/data_raw --once > /dev/null 2>&1 && \
    echo -e "${GREEN}✓${NC} /imu/data_raw is publishing data" || \
    echo -e "${RED}✗${NC} /imu/data_raw has no data"

echo "Checking /odom for data..."
timeout 2 ros2 topic echo /odom --once > /dev/null 2>&1 && \
    echo -e "${GREEN}✓${NC} /odom is publishing data" || \
    echo -e "${RED}✗${NC} /odom has no data"

echo ""
echo "=========================================="
echo "5. Firmware Topic Summary"
echo "=========================================="
echo ""
echo "Expected firmware topics (as published by Pico):"
echo "  - joint_states          → /rt/joint_states → /joint_states"
echo "  - imu/data_raw          → /rt/imu/data_raw → /imu/data_raw"
echo "  - odom                  → /rt/odom → /odom"
echo "  - sensors/range_tof     → /rt/sensors/range_tof → /sensors/range_tof"
echo "  - sensors/range_ultrasonic → /rt/sensors/range_ultrasonic → /sensors/range_ultrasonic"
echo "  - sensors/illuminance   → /rt/sensors/illuminance → /sensors/illuminance"
echo "  - pico_rnd              → /rt/pico_rnd (debug counter)"
echo ""
echo "Firmware subscribes to:"
echo "  - cmd_vel               → /rt/cmd_vel (from /cmd_vel)"
echo ""

if [ $failed -eq 0 ]; then
    echo -e "${GREEN}=========================================="
    echo "✓ All firmware topics verified successfully!"
    echo -e "==========================================${NC}"
    exit 0
else
    echo -e "${RED}=========================================="
    echo "✗ Some topics are missing or not publishing"
    echo "==========================================${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Ensure micro-ROS agent is running"
    echo "  2. Ensure Pico firmware is flashed and connected"
    echo "  3. Check agent remapping configuration"
    echo "  4. Verify firmware is running (check serial output)"
    exit 1
fi
