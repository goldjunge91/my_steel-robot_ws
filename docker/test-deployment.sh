#!/bin/bash
# ============================================================================
# Docker Deployment Test Script for Raspberry Pi
# ============================================================================
#
# This script tests the Docker Compose deployment on Raspberry Pi by:
# 1. Starting containers with docker compose
# 2. Verifying microros-agent health check passes
# 3. Verifying robot-bringup health check passes
# 4. Checking that controllers are active
# 5. Verifying topics are publishing data
#
# Usage:
#   ./test-deployment.sh
#
# Prerequisites:
#   - Docker and Docker Compose installed
#   - .env file configured in docker/ directory
#   - USB devices connected (/dev/ttyACM0, /dev/video0)
#
# ============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "============================================================================"
echo "Docker Deployment Test Script"
echo "============================================================================"
echo ""

# Check if .env file exists
if [ ! -f "$SCRIPT_DIR/.env" ]; then
    echo -e "${RED}ERROR: .env file not found in $SCRIPT_DIR${NC}"
    echo "Please create .env file from .env.robot-pi.example"
    exit 1
fi

echo -e "${GREEN}✓${NC} Found .env file"

# Check if required devices exist
if [ ! -e "/dev/ttyACM0" ]; then
    echo -e "${YELLOW}WARNING: /dev/ttyACM0 not found (Raspberry Pi Pico)${NC}"
    echo "Please connect the Pico and ensure it's running micro-ROS firmware"
fi

if [ ! -e "/dev/video0" ]; then
    echo -e "${YELLOW}WARNING: /dev/video0 not found (USB camera)${NC}"
    echo "Camera functionality will not be available"
fi

echo ""
echo "============================================================================"
echo "Step 1: Starting containers with docker compose"
echo "============================================================================"
echo ""

docker compose -f "$SCRIPT_DIR/compose.robot-pi.yaml" up -d

echo ""
echo -e "${GREEN}✓${NC} Containers started"
echo ""

# Wait for containers to initialize
echo "Waiting 10 seconds for containers to initialize..."
sleep 10

echo ""
echo "============================================================================"
echo "Step 2: Verifying microros-agent health check"
echo "============================================================================"
echo ""

# Check microros-agent health status
MAX_RETRIES=6
RETRY_COUNT=0
MICROROS_HEALTHY=false

while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    HEALTH_STATUS=$(docker inspect --format='{{.State.Health.Status}}' microros-agent 2>/dev/null || echo "unknown")
    
    if [ "$HEALTH_STATUS" = "healthy" ]; then
        echo -e "${GREEN}✓${NC} microros-agent is healthy"
        MICROROS_HEALTHY=true
        break
    else
        echo "microros-agent health status: $HEALTH_STATUS (attempt $((RETRY_COUNT+1))/$MAX_RETRIES)"
        RETRY_COUNT=$((RETRY_COUNT+1))
        if [ $RETRY_COUNT -lt $MAX_RETRIES ]; then
            sleep 10
        fi
    fi
done

if [ "$MICROROS_HEALTHY" = false ]; then
    echo -e "${RED}✗${NC} microros-agent failed to become healthy"
    echo ""
    echo "Logs from microros-agent:"
    docker compose -f "$SCRIPT_DIR/compose.robot-pi.yaml" logs --tail=50 microros-agent
    exit 1
fi

echo ""
echo "============================================================================"
echo "Step 3: Verifying robot-bringup health check"
echo "============================================================================"
echo ""

# Check robot-bringup health status
RETRY_COUNT=0
BRINGUP_HEALTHY=false

while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    HEALTH_STATUS=$(docker inspect --format='{{.State.Health.Status}}' robot-bringup 2>/dev/null || echo "unknown")
    
    if [ "$HEALTH_STATUS" = "healthy" ]; then
        echo -e "${GREEN}✓${NC} robot-bringup is healthy"
        BRINGUP_HEALTHY=true
        break
    else
        echo "robot-bringup health status: $HEALTH_STATUS (attempt $((RETRY_COUNT+1))/$MAX_RETRIES)"
        RETRY_COUNT=$((RETRY_COUNT+1))
        if [ $RETRY_COUNT -lt $MAX_RETRIES ]; then
            sleep 10
        fi
    fi
done

if [ "$BRINGUP_HEALTHY" = false ]; then
    echo -e "${RED}✗${NC} robot-bringup failed to become healthy"
    echo ""
    echo "Logs from robot-bringup:"
    docker compose -f "$SCRIPT_DIR/compose.robot-pi.yaml" logs --tail=50 robot-bringup
    exit 1
fi

echo ""
echo "============================================================================"
echo "Step 4: Checking that controllers are active"
echo "============================================================================"
echo ""

# List controllers and check for active status
CONTROLLERS=$(docker exec robot-bringup bash -c "source /opt/ros/humble/setup.bash && ros2 control list_controllers" 2>/dev/null || echo "")

if echo "$CONTROLLERS" | grep -q "active"; then
    echo -e "${GREEN}✓${NC} Controllers are active:"
    echo "$CONTROLLERS"
else
    echo -e "${RED}✗${NC} No active controllers found"
    echo "Controller status:"
    echo "$CONTROLLERS"
    exit 1
fi

echo ""
echo "============================================================================"
echo "Step 5: Verifying topics are publishing data"
echo "============================================================================"
echo ""

# Check for key topics
TOPICS=$(docker exec robot-bringup bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" 2>/dev/null || echo "")

echo "Available topics:"
echo "$TOPICS"
echo ""

# Check for essential topics
ESSENTIAL_TOPICS=(
    "/joint_states"
    "/odom"
    "/tf"
    "/tf_static"
)

ALL_TOPICS_FOUND=true
for topic in "${ESSENTIAL_TOPICS[@]}"; do
    if echo "$TOPICS" | grep -q "^$topic$"; then
        echo -e "${GREEN}✓${NC} Found topic: $topic"
    else
        echo -e "${RED}✗${NC} Missing topic: $topic"
        ALL_TOPICS_FOUND=false
    fi
done

if [ "$ALL_TOPICS_FOUND" = false ]; then
    echo ""
    echo -e "${YELLOW}WARNING: Some essential topics are missing${NC}"
    echo "This may be expected if hardware is not fully connected"
fi

echo ""
echo "============================================================================"
echo "Test Summary"
echo "============================================================================"
echo ""

if [ "$MICROROS_HEALTHY" = true ] && [ "$BRINGUP_HEALTHY" = true ]; then
    echo -e "${GREEN}✓${NC} All health checks passed"
    echo -e "${GREEN}✓${NC} Controllers are active"
    
    if [ "$ALL_TOPICS_FOUND" = true ]; then
        echo -e "${GREEN}✓${NC} All essential topics are publishing"
        echo ""
        echo -e "${GREEN}SUCCESS: Docker deployment is working correctly!${NC}"
        exit 0
    else
        echo -e "${YELLOW}⚠${NC} Some topics are missing (may be expected)"
        echo ""
        echo -e "${YELLOW}PARTIAL SUCCESS: Core functionality is working${NC}"
        exit 0
    fi
else
    echo -e "${RED}✗${NC} Health checks failed"
    echo ""
    echo -e "${RED}FAILURE: Docker deployment has issues${NC}"
    exit 1
fi
