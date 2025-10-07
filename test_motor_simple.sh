#!/bin/bash
# Einfacher Motor-Test ohne IMU

set -e

echo "=== Rebuilding robot_hardware_interfaces ==="
cd ~/workspace/ros2_dev_ws/my_steel-robot_ws
colcon build --merge-install --packages-select robot_hardware_interfaces
source install/setup.bash

echo ""
echo "=== Starting Controller Manager (without IMU) ==="
echo "Press Ctrl+C to stop"
echo ""

# Starte nur mit drive_controller, ohne imu_broadcaster
ros2 launch robot_controller controller.launch.py \
  use_sim_time:=false \
  --ros-args \
  -p controller_manager.imu_broadcaster.type:='' \
  || true

echo ""
echo "Test beendet"
