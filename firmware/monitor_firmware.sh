<<<<<<< HEAD
#!/bin/bash

# Firmware monitoring script for my_steel robot
# Monitors UART debug output and micro-ROS connection status

set -e

UART_PORT="${1:-/dev/ttyUSBO}"
UART_BAUD="${2:-115200}"
USB_PORT="${3:-/dev/ttyACM0}"

echo "=== my_steel Firmware Monitor ==="
echo "UART Debug: $UART_PORT @ $UART_BAUD"
echo "USB micro-ROS: $USB_PORT"
echo "Press Ctrl+C to exit"
echo

# Function to monitor UART debug output
monitor_uart() {
    echo "--- UART Debug Output ---"
    if [ -e "$UART_PORT" ]; then
        stty -F "$UART_PORT" "$UART_BAUD" raw -echo
        timeout 30 cat "$UART_PORT" || echo "UART timeout or no data"
    else
        echo "UART port $UART_PORT not found"
    fi
}

# Function to check micro-ROS connection
check_microros() {
    echo "--- micro-ROS Connection Status ---"
    if [ -e "$USB_PORT" ]; then
        echo "USB port $USB_PORT detected"
        # Check if micro-ROS agent can connect
        timeout 10 ros2 run micro_ros_agent micro_ros_agent serial --dev "$USB_PORT" -v 2>/dev/null || \
        echo "micro-ROS agent connection test failed or timed out"
    else
        echo "USB port $USB_PORT not found"
    fi
}

# Function to show ROS topics from Pico
check_topics() {
    echo "--- Active ROS Topics from Pico ---"
    timeout 5 ros2 topic list | grep -E "(pico_count|odom|range)" || echo "No Pico topics found"
}

# Main monitoring loop
while true; do
    clear
    echo "=== Firmware Status Check $(date) ==="
    echo
    
    monitor_uart
    echo
    check_microros
    echo
    check_topics
    echo
    
    echo "Waiting 10 seconds... (Ctrl+C to exit)"
    sleep 10
=======
#!/bin/bash

# Firmware monitoring script for my_steel robot
# Monitors UART debug output and micro-ROS connection status

set -e

UART_PORT="${1:-/dev/ttyUSB0}"
UART_BAUD="${2:-115200}"
USB_PORT="${3:-/dev/ttyACM0}"

echo "=== my_steel Firmware Monitor ==="
echo "UART Debug: $UART_PORT @ $UART_BAUD"
echo "USB micro-ROS: $USB_PORT"
echo "Press Ctrl+C to exit"
echo

# Function to monitor UART debug output
monitor_uart() {
    echo "--- UART Debug Output ---"
    if [ -e "$UART_PORT" ]; then
        stty -F "$UART_PORT" "$UART_BAUD" raw -echo
        timeout 30 cat "$UART_PORT" || echo "UART timeout or no data"
    else
        echo "UART port $UART_PORT not found"
    fi
}

# Function to check micro-ROS connection
check_microros() {
    echo "--- micro-ROS Connection Status ---"
    if [ -e "$USB_PORT" ]; then
        echo "USB port $USB_PORT detected"
        # Check if micro-ROS agent can connect
        timeout 10 ros2 run micro_ros_agent micro_ros_agent serial --dev "$USB_PORT" -v 2>/dev/null || \
        echo "micro-ROS agent connection test failed or timed out"
    else
        echo "USB port $USB_PORT not found"
    fi
}

# Function to show ROS topics from Pico
check_topics() {
    echo "--- Active ROS Topics from Pico ---"
    timeout 5 ros2 topic list | grep -E "(pico_count|odom|range)" || echo "No Pico topics found"
}

# Main monitoring loop
while true; do
    clear
    echo "=== Firmware Status Check $(date) ==="
    echo
    
    monitor_uart
    echo
    check_microros
    echo
    check_topics
    echo
    
    echo "Waiting 10 seconds... (Ctrl+C to exit)"
    sleep 10
>>>>>>> d4e5564 (Add documentation for robot packages and directories)
done