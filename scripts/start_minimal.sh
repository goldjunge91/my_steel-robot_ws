#!/bin/bash
# Minimal robot start without micro-ROS dependencies

echo "=== Starting Minimal Robot System ==="

# Set environment
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
unset CYCLONEDDS_URI

# Source ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Environment configured:"
echo "RMW: $RMW_IMPLEMENTATION"
echo "Domain: $ROS_DOMAIN_ID"

echo "Starting minimal components..."

# Start only robot state publisher and basic components
ros2 launch robot_controller controller.launch.py \
    robot_model:=robot_xl \
    mecanum:=True \
    use_sim:=True \
    configuration:=basic