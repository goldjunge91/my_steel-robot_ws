#!/bin/bash

echo "=== micro-ROS Diagnose Script ==="
echo ""

# Kill any existing agents
echo "1. Cleaning up existing processes..."
sudo pkill -f "micro_ros_agent" || true
sudo fuser -k /dev/ttyACM0 2>/dev/null || true
sleep 2

# Check if Pico is connected
echo ""
echo "2. Hardware Connection Check..."
if [ -e /dev/ttyACM0 ]; then
    echo "✅ Pico found at /dev/ttyACM0"
    ls -la /dev/ttyACM0
else
    echo "❌ Pico not found at /dev/ttyACM0"
    echo "Available serial devices:"
    ls -la /dev/tty* | grep -E "(ACM|USB)"
    exit 1
fi

# Check permissions
echo ""
echo "3. Permission Check..."
if [ -r /dev/ttyACM0 ] && [ -w /dev/ttyACM0 ]; then
    echo "✅ Read/Write permissions OK"
else
    echo "❌ Permission issue - adding user to dialout group"
    sudo usermod -a -G dialout $USER
    echo "⚠️  You may need to logout/login for group changes to take effect"
fi

# Check what's on the serial port (firmware output)
echo ""
echo "4. Checking Pico Serial Output (5 seconds)..."
timeout 5 cat /dev/ttyACM0 2>/dev/null || echo "No serial output detected"

# Start agent with verbose logging
echo ""
echo "5. Starting Agent with verbose logging..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start agent in background with full logging
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v6 &
AGENT_PID=$!
echo "Agent started with PID: $AGENT_PID"

# Wait for agent to initialize
echo ""
echo "6. Waiting for Agent initialization (10 seconds)..."
sleep 10

# Check if agent is still running
if ! ps -p $AGENT_PID > /dev/null; then
    echo "❌ Agent crashed during startup"
    exit 1
fi

# Check ROS2 daemon
echo ""
echo "7. ROS2 Environment Check..."
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 daemon status
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"

# List all topics
echo ""
echo "8. All Available Topics:"
timeout 5 ros2 topic list -t || echo "Failed to list topics"

# Look specifically for micro-ROS topics
echo ""
echo "9. Searching for micro-ROS topics..."
timeout 5 ros2 topic list | grep -E "(ddd|pico|micro)" || echo "No micro-ROS topics found"

# Check node list
echo ""
echo "10. Available Nodes:"
timeout 5 ros2 node list || echo "Failed to list nodes"

# Try to echo any available topic for 5 seconds
echo ""
echo "11. Testing topic communication (5 seconds)..."
TOPICS=$(timeout 3 ros2 topic list 2>/dev/null)
if [ ! -z "$TOPICS" ]; then
    for topic in $TOPICS; do
        if [[ $topic == *"ddd"* ]] || [[ $topic == *"odom"* ]]; then
            echo "Testing topic: $topic"
            timeout 5 ros2 topic echo $topic --once 2>/dev/null || echo "No data on $topic"
        fi
    done
else
    echo "No topics available to test"
fi

# Final status
echo ""
echo "12. Final Status..."
if ps -p $AGENT_PID > /dev/null; then
    echo "✅ Agent still running (PID: $AGENT_PID)"
    echo ""
    echo "=== Next Steps ==="
    echo "1. Check firmware build: cd firmware && make"
    echo "2. Flash firmware: Follow firmware/README.md"
    echo "3. Check serial output: cat /dev/ttyACM0"
    echo "4. Restart Pico: Unplug/replug USB"
    echo ""
    echo "To stop agent: kill $AGENT_PID"
else
    echo "❌ Agent stopped running"
fi

echo ""
echo "=== Diagnosis Complete ==="