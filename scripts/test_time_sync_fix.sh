#!/bin/bash

echo "=== Testing Time Sync Fix ==="
echo "This script will help you test the micro-ROS time sync fix"
echo ""

# Check if agent is running
if pgrep -f "micro_ros_agent" >/dev/null; then
    echo "✓ micro_ros_agent is running"
else
    echo "✗ micro_ros_agent is not running"
    echo "Please start the agent first:"
    echo "  ./scripts/unblock_and_run_agent.sh --kill --start-agent"
    exit 1
fi

echo ""
echo "=== Instructions ==="
echo "1. Flash the new firmware to your Pico:"
echo "   - Put Pico in bootloader mode (hold BOOTSEL while connecting USB)"
echo "   - Copy firmware/build/src/my_firmware.uf2 to the RPI-RP2 drive"
echo ""
echo "2. Monitor the serial output:"
echo "   - Connect to /dev/ttyACM0 at 115200 baud"
echo "   - Look for these improved messages:"
echo "     • 'TIME SYNC successful' (good)"
echo "     • 'TIME SYNC error (agent not ready) – continuing without sync' (acceptable)"
echo "     • 'TIME SYNC timeout (agent busy) – continuing without sync' (acceptable)"
echo ""
echo "3. What should change:"
echo "   - No more 'ERROR Time synk failed (ret=1)' followed by 'Entities Destroyed'"
echo "   - No more continuous reconnection loops"
echo "   - System should continue working even if time sync fails"
echo ""
echo "4. Expected behavior:"
echo "   - Entities should stay created even if time sync fails"
echo "   - Publishers should work normally"
echo "   - No more 'Queue Pub failed 300' errors"
echo ""

# Check if firmware file exists
if [ -f "firmware/build/src/my_firmware.uf2" ]; then
    echo "✓ Firmware file ready: firmware/build/src/my_firmware.uf2"
else
    echo "✗ Firmware file not found. Run 'make build' in firmware directory first."
fi

echo ""
echo "=== Monitoring Commands ==="
echo "To monitor the Pico output:"
echo "  sudo screen /dev/ttyACM0 115200"
echo "  # or"
echo "  sudo minicom -D /dev/ttyACM0 -b 115200"
echo ""
echo "To check ROS topics:"
echo "  ros2 topic list"
echo "  ros2 topic echo /odom"
echo "  ros2 topic echo /joint_states"