#!/bin/bash

echo "=== IMU Publisher Diagnostic Test ==="
echo "This script will test the IMU publisher specifically"
echo

# Check if Pico is connected
echo "1. Checking for Pico connection..."
if ls /dev/ttyACM* 2>/dev/null; then
    PICO_PORT=$(ls /dev/ttyACM* | head -1)
    echo "   Found Pico at: $PICO_PORT"
else
    echo "   ERROR: No Pico found. Please connect Pico and flash firmware."
    exit 1
fi

echo
echo "2. Starting micro-ROS Agent..."
echo "   Command: ros2 run micro_ros_agent micro_ros_agent serial --dev $PICO_PORT"
echo "   Press Ctrl+C to stop the agent when done testing"
echo

# Start micro-ROS agent in background
ros2 run micro_ros_agent micro_ros_agent serial --dev $PICO_PORT &
AGENT_PID=$!

# Wait for agent to start
sleep 3

echo
echo "3. Testing ROS2 topics..."
echo "   Checking available topics:"
ros2 topic list

echo
echo "4. Checking for IMU topic specifically:"
if ros2 topic list | grep -q "/ddd/imu"; then
    echo "   ✅ IMU topic found: /ddd/imu"
    
    echo
    echo "5. Testing IMU topic info:"
    ros2 topic info /ddd/imu
    
    echo
    echo "6. Listening to IMU messages (5 seconds):"
    timeout 5s ros2 topic echo /ddd/imu --once
    
else
    echo "   ❌ IMU topic NOT found"
    echo "   Available topics:"
    ros2 topic list | sed 's/^/      /'
fi

echo
echo "7. Checking serial output from Pico:"
echo "   Monitoring $PICO_PORT for debug messages..."
timeout 3s cat $PICO_PORT 2>/dev/null || echo "   No serial output detected"

# Clean up
echo
echo "8. Cleaning up..."
kill $AGENT_PID 2>/dev/null
wait $AGENT_PID 2>/dev/null

echo
echo "=== Test Complete ==="