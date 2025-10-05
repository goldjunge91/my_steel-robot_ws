#!/bin/bash

# Remote PC environment setup
# Copy this file to your remote PC and source it before running ROS2 nodes

# ROS2 Humble setup
source /opt/ros/humble/setup.bash

# ROS Domain ID (MUST MATCH ROBOT!)
export ROS_DOMAIN_ID=42

# DDS configuration
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Copy cyclone_dds.xml from robot to same location or adjust path:
# export CYCLONEDDS_URI=file:///path/to/cyclone_dds.xml

# Robot IP (optional, for verification)
export ROBOT_IP=192.168.1.XXX  # Replace with your robot's IP

echo "Remote PC ROS2 Environment configured:"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  RMW: $RMW_IMPLEMENTATION"
echo "  Robot IP: $ROBOT_IP"
echo ""
echo "Available commands:"
echo "  ros2 topic list          - List all topics"
echo "  ros2 topic echo /topic   - Monitor topic"
echo "  ros2 node list           - List all nodes"
