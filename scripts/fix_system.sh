#!/bin/bash
# Quick fix script for ROS2 system issues

echo "=== Fixing ROS2 System Issues ==="

# 1. Install missing micro-ROS agent
echo "Installing micro-ROS agent..."
sudo apt update
sudo apt install ros-humble-micro-ros-agent -y

# 2. Set correct RMW implementation
echo "Setting FastRTPS as RMW implementation..."
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc

# 3. Disable problematic CycloneDDS config
echo "Disabling CycloneDDS config..."
if [ -f ~/.bashrc ]; then
    sed -i '/CYCLONEDDS_URI/d' ~/.bashrc
fi

# 4. Source environment
source ~/.bashrc

echo "=== Fixes applied! Please restart your terminal and try again ==="
echo ""
echo "Next steps:"
echo "1. Open new terminal"
echo "2. cd ~/my_steel-robot_ws"
echo "3. source /opt/ros/humble/setup.bash"
echo "4. source install/setup.bash"
echo "5. ros2 launch robot_bringup bringup.launch.py robot_model:=robot_xl mecanum:=True microros:=False"