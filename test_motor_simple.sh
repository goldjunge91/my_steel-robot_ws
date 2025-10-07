#!/bin/bash
# Einfacher Motor-Test

set -e

echo "=== Rebuilding robot_hardware_interfaces ==="
cd ~/workspace/ros2_dev_ws/my_steel-robot_ws
colcon build --merge-install --packages-select robot_hardware_interfaces
source install/setup.bash

echo ""
echo "=== Testing Motors ==="
echo ""

# Zeige aktuelle Joint States
echo "Current joint states:"
timeout 2 ros2 topic echo /joint_states --once || echo "No joint states yet"

echo ""
echo "Sending forward velocity command (0.1 m/s)..."
echo "Watch the joint states change!"
echo ""

# Sende Bewegungsbefehl und zeige Joint States
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {z: 0.0}}" \
  --rate 10 &

CMD_PID=$!

# Zeige Joint States für 5 Sekunden
echo "Watching joint states for 5 seconds..."
timeout 5 ros2 topic echo /joint_states || true

# Stoppe Bewegung
kill $CMD_PID 2>/dev/null || true

echo ""
echo "Sending STOP command..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" \
  --once

echo ""
echo "Test complete!"
