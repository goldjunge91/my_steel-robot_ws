#!/bin/bash
# Remote PC Environment Setup
# Quelle: source ~/remote_pc_setup.sh

# WICHTIG: Passe diese IP an!
export ROBOT_IP="192.168.1.100"  # ← DEINE RASPBERRY PI IP

# ROS2 Environment
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0

# ROS2 Base
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS2 Humble sourced"
else
    echo "✗ ROS2 Humble nicht gefunden!"
    return 1
fi

# Aliases
alias robot-nodes='ros2 node list'
alias robot-topics='ros2 topic list'
alias robot-check='ros2 topic hz /camera/image_raw & ros2 topic hz /imu/data_raw & ros2 topic hz /joint_states'
alias robot-rviz='ros2 launch robot_description rviz.launch.py'
alias robot-teleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard'
alias robot-joy='ros2 run teleop_twist_joy teleop_node'
alias robot-drive='ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"'
alias robot-stop='ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"'

echo ""
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║              REMOTE PC ENVIRONMENT CONFIGURED                    ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo ""
echo "  Robot IP:      $ROBOT_IP"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  RMW:           $RMW_IMPLEMENTATION"
echo ""
echo "Quick Commands:"
echo "  robot-nodes     → List all nodes"
echo "  robot-topics    → List all topics"
echo "  robot-check     → Check sensor frequencies"
echo "  robot-rviz      → Start RViz2"
echo "  robot-teleop    → Keyboard control"
echo "  robot-joy       → Joystick control"
echo "  robot-drive     → Test drive forward"
echo "  robot-stop      → Emergency stop"
echo ""
echo "Foxglove: ws://$ROBOT_IP:8765"
echo ""
