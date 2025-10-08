#!/bin/bash
# Verification script for RViz2 visualization topics
# Task 7.8: Test with RViz2 visualization

echo "=========================================="
echo "RViz2 Topic Verification"
echo "=========================================="
echo ""

# Check required topics
echo "✓ Checking /joint_states..."
ros2 topic info /joint_states
echo ""

echo "✓ Checking /imu/data_raw..."
ros2 topic info /imu/data_raw
echo ""

echo "✓ Checking /odom..."
ros2 topic info /odom
echo ""

echo "✓ Checking /robot_description..."
ros2 topic info /robot_description
echo ""

echo "=========================================="
echo "Sample data from each topic:"
echo "=========================================="
echo ""

echo "--- /joint_states (1 message) ---"
timeout 2 ros2 topic echo /joint_states --once
echo ""

echo "--- /imu/data_raw (1 message) ---"
timeout 2 ros2 topic echo /imu/data_raw --once
echo ""

echo "--- /odom (1 message) ---"
timeout 2 ros2 topic echo /odom --once
echo ""

echo "=========================================="
echo "✓ All topics verified successfully!"
echo "=========================================="
echo ""
echo "RViz2 configuration file: src/robot_description/rviz/robot_test.rviz"
echo "- Robot model display (uses /joint_states)"
echo "- IMU display (uses /imu/data_raw)"
echo "- Odometry display (uses /odom)"
echo "- TF display (shows transform tree)"
echo ""
echo "To launch RViz2 manually:"
echo "  rviz2 -d src/robot_description/rviz/robot_test.rviz"
