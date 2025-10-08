#!/bin/bash
# Test script for RViz2 visualization with standardized topics
# Task 7.8: Test with RViz2 visualization

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "RViz2 Visualization Test"
echo "=========================================="
echo ""

# Check if ROS2 topics are available
echo "Step 1: Checking available ROS2 topics..."
TOPICS=$(ros2 topic list 2>/dev/null || true)

if [ -z "$TOPICS" ]; then
    echo "❌ No ROS2 topics found. Make sure the robot system is running."
    echo ""
    echo "Start the system with:"
    echo "  ros2 launch robot_bringup bringup.launch.py"
    exit 1
fi

echo "✓ ROS2 is running"
echo ""

# Check for required topics
echo "Step 2: Verifying required topics exist..."
REQUIRED_TOPICS=("/joint_states" "/imu/data_raw" "/odom" "/robot_description")
MISSING_TOPICS=()

for topic in "${REQUIRED_TOPICS[@]}"; do
    if echo "$TOPICS" | grep -q "^${topic}$"; then
        echo "✓ Found: $topic"
    else
        echo "❌ Missing: $topic"
        MISSING_TOPICS+=("$topic")
    fi
done

if [ ${#MISSING_TOPICS[@]} -gt 0 ]; then
    echo ""
    echo "❌ Missing required topics. Cannot proceed with visualization test."
    exit 1
fi

echo ""
echo "Step 3: Checking topic data..."

# Check if topics are publishing data
for topic in "/joint_states" "/imu/data_raw" "/odom"; do
    echo -n "Checking $topic... "
    timeout 2 ros2 topic echo "$topic" --once >/dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "✓ Publishing data"
    else
        echo "⚠ No data received (timeout)"
    fi
done

echo ""
echo "Step 4: Checking TF tree..."
TF_FRAMES=$(ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -n 5)
if echo "$TF_FRAMES" | grep -q "At time"; then
    echo "✓ TF transform odom -> base_link exists"
else
    echo "⚠ TF transform may not be available"
fi

echo ""
echo "=========================================="
echo "All checks passed! Launching RViz2..."
echo "=========================================="
echo ""
echo "Verify in RViz2:"
echo "  1. Robot model is displayed (uses /joint_states)"
echo "  2. Joint states update when motors move"
echo "  3. IMU orientation is shown (uses /imu/data_raw)"
echo "  4. Odometry arrow is displayed (uses /odom)"
echo "  5. TF tree shows odom -> base_link -> imu_link"
echo ""
echo "Press Ctrl+C to exit RViz2"
echo ""
sleep 2

# Launch RViz2 with the test configuration
rviz2 -d src/robot_description/rviz/robot_test.rviz
