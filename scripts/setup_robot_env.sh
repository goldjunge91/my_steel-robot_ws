#!/bin/env bash

# Environment setup script for robot hardware deployment
# Source this file before running ROS2 nodes

# ROS2 Humble setup
source /opt/ros/humble/setup.bash

# Workspace setup (if built)
if [ -f "$HOME/workspace/ros2_dev_ws/my_steel-robot_ws/install/setup.bash" ]; then
    source "$HOME/workspace/ros2_dev_ws/my_steel-robot_ws/install/setup.bash"
fi

# ROS Domain ID (must match on all machines!)
export ROS_DOMAIN_ID=42

# DDS configuration
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/workspace/ros2_dev_ws/my_steel-robot_ws/config/cyclone_dds.xml

# Disable shared memory (for network communication)
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

# Robot model
export ROBOT_MODEL_NAME=robot_xl

echo "ROS2 Environment configured:"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  RMW: $RMW_IMPLEMENTATION"
echo "  DDS Config: $CYCLONEDDS_URI"
echo "  Robot Model: $ROBOT_MODEL_NAME"
