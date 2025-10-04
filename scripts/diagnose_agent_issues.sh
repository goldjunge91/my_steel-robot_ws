#!/bin/bash

echo "=== Diagnosing micro-ROS Agent Issues ==="
echo ""

# Check agent process
echo "1. Agent Process Status:"
if pgrep -f "micro_ros_agent" >/dev/null; then
    AGENT_PID=$(pgrep -f "micro_ros_agent")
    echo "✅ Agent running (PID: $AGENT_PID)"
    
    # Check CPU usage
    CPU_USAGE=$(ps -p $AGENT_PID -o %cpu --no-headers)
    echo "   CPU usage: ${CPU_USAGE}%"
    
    # Check memory usage
    MEM_USAGE=$(ps -p $AGENT_PID -o %mem --no-headers)
    echo "   Memory usage: ${MEM_USAGE}%"
else
    echo "❌ Agent not running"
    exit 1
fi

echo ""
echo "2. Serial Port Status:"
DEVICE="/dev/ttyACM0"
if [ -e "$DEVICE" ]; then
    echo "✅ Device exists: $DEVICE"
    
    # Check if device is busy
    if lsof "$DEVICE" >/dev/null 2>&1; then
        echo "   Device in use by:"
        lsof "$DEVICE" | tail -n +2
    else
        echo "   Device not in use"
    fi
else
    echo "❌ Device not found: $DEVICE"
fi

echo ""
echo "3. ROS2 Topics Status:"
echo "Available topics:"
ros2 topic list | while read topic; do
    if [[ "$topic" == "/ddd/"* ]] || [[ "$topic" == "/joint_states" ]] || [[ "$topic" == "/pico_count" ]]; then
        TYPE=$(ros2 topic info "$topic" 2>/dev/null | grep "Type:" | cut -d' ' -f2)
        PUB_COUNT=$(ros2 topic info "$topic" 2>/dev/null | grep "Publisher count:" | cut -d' ' -f3)
        echo "   $topic ($TYPE) - Publishers: $PUB_COUNT"
    fi
done

echo ""
echo "4. Message Rate Analysis:"
echo "Checking message rates (5 second sample)..."

for topic in "/ddd/odom" "/joint_states" "/pico_count"; do
    if ros2 topic list | grep -q "^$topic$"; then
        echo -n "   $topic: "
        RATE=$(timeout 5 ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | cut -d' ' -f3)
        if [ -n "$RATE" ]; then
            echo "${RATE} Hz"
        else
            echo "No messages received"
        fi
    fi
done

echo ""
echo "5. Agent Log Analysis:"
echo "Recent agent activity (if available):"
# This would show recent agent logs if we had access to them
# For now, just show what we can observe

echo ""
echo "6. Recommendations:"
echo ""

# Check message rates
ODOM_RATE=$(timeout 3 ros2 topic hz "/ddd/odom" 2>/dev/null | grep "average rate" | cut -d' ' -f3 | cut -d'.' -f1)
if [ -n "$ODOM_RATE" ] && [ "$ODOM_RATE" -gt 20 ]; then
    echo "⚠️  High odometry rate (${ODOM_RATE} Hz) - consider reducing frequency"
fi

# Check for error patterns
if ros2 topic echo /ddd/odom --once >/dev/null 2>&1; then
    echo "✅ Odometry messages are being received"
else
    echo "❌ Odometry messages not available - check publisher"
fi

echo ""
echo "=== Suggested Fixes ==="
echo "1. Reduce publishing frequency in firmware (increase vTaskDelay)"
echo "2. Restart agent with higher verbosity: add -v6 flag"
echo "3. Check for USB connection stability"
echo "4. Monitor agent logs for specific error messages"
echo ""
echo "To restart agent with debug info:"
echo "./scripts/unblock_and_run_agent.sh --kill"
echo "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v6"