#!/bin/bash

echo "=== Verifying Message Initialization Fixes ==="
echo ""

echo "1. Checking current odometry message content..."
echo "Looking for garbage covariance values..."
echo ""

# Check for garbage values in odometry
ODOM_OUTPUT=$(timeout 3 ros2 topic echo /ddd/odom --once 2>/dev/null)
if echo "$ODOM_OUTPUT" | grep -q "2.498335"; then
    echo "❌ STILL BROKEN: Found garbage covariance values"
    echo "   Example: $(echo "$ODOM_OUTPUT" | grep "2.498335" | head -1)"
    echo "   → Need to flash updated firmware"
else
    echo "✅ FIXED: No garbage covariance values found"
fi

# Check quaternion orientation
if echo "$ODOM_OUTPUT" | grep -A4 "orientation:" | grep -q "w: 1.0"; then
    echo "✅ FIXED: Quaternion orientation is identity (w=1.0)"
else
    echo "❌ ISSUE: Quaternion orientation not set to identity"
    echo "   Current: $(echo "$ODOM_OUTPUT" | grep -A4 "orientation:" | grep "w:")"
fi

echo ""
echo "2. Checking joint states message..."

# Check joint states
JOINT_OUTPUT=$(timeout 3 ros2 topic echo /joint_states --once 2>/dev/null)
if [ -n "$JOINT_OUTPUT" ]; then
    echo "✅ Joint states message received successfully"
    POSITION_COUNT=$(echo "$JOINT_OUTPUT" | grep -A10 "position:" | grep -c "[-0-9]")
    echo "   Position array elements: $POSITION_COUNT"
else
    echo "❌ No joint states message received"
fi

echo ""
echo "3. Checking connection stability..."

# Check if agent is running
if pgrep -f "micro_ros_agent" >/dev/null; then
    echo "✅ micro_ros_agent is running"
else
    echo "❌ micro_ros_agent is not running"
    echo "   Start with: ./scripts/unblock_and_run_agent.sh --kill --start-agent"
fi

echo ""
echo "4. Current firmware status:"
if [ -f "firmware/build/src/my_firmware.uf2" ]; then
    FIRMWARE_TIME=$(stat -c %Y firmware/build/src/my_firmware.uf2)
    CURRENT_TIME=$(date +%s)
    AGE=$((CURRENT_TIME - FIRMWARE_TIME))
    
    if [ $AGE -lt 300 ]; then  # Less than 5 minutes old
        echo "✅ Firmware recently built (${AGE}s ago)"
    else
        echo "⚠️  Firmware is older (${AGE}s ago) - consider rebuilding"
    fi
else
    echo "❌ No firmware file found - run 'make build' in firmware directory"
fi

echo ""
echo "=== Next Steps ==="
if echo "$ODOM_OUTPUT" | grep -q "2.498335"; then
    echo "1. Rebuild firmware: cd firmware && make build"
    echo "2. Flash to Pico: cp build/src/my_firmware.uf2 /media/marco/RPI-RP2/"
    echo "3. Run this script again to verify"
else
    echo "✅ Messages look good! Monitor for connection stability:"
    echo "   sudo screen /dev/ttyACM0 115200"
    echo "   Look for stable 'ALIVE' messages without 'Entities Destroyed'"
fi