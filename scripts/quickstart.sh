#!/bin/bash
# Kompletter Robot Setup auf Raspberry Pi

cat << 'EOF'
╔══════════════════════════════════════════════════════════════════╗
║                    ROBOT HARDWARE QUICKSTART                     ║
╚══════════════════════════════════════════════════════════════════╝

📍 RASPBERRY PI (Robot)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1️⃣  Install Dependencies:
   sudo apt install ros-humble-foxglove-bridge ros-humble-v4l2-camera

2️⃣  Build Workspace:
   cd /home/pi/workspace/ros2_dev_ws/my_steel-robot_ws
   ./build.sh

3️⃣  Start Robot:
   ./scripts/start_robot.sh

   Started services:
   ✓ Micro-ROS Agent (Pico → ROS2)
   ✓ Camera (v4l2)
   ✓ IMU Broadcaster
   ✓ Mecanum Drive Controller
   ✓ Foxglove Bridge (Port 8765)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

💻 REMOTE PC (Laptop/Desktop)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1️⃣  Setup Environment:
   source ~/remote_pc_setup_env.sh
   # Edit file to set ROBOT_IP!

2️⃣  Test Connection:
   ros2 node list
   ros2 topic list

3️⃣  Start Controller:
   ros2 launch ~/remote_pc_joystick.launch.py
   # LB + Left Stick = Drive

4️⃣  Start RViz2:
   ros2 launch robot_description rviz.launch.py

5️⃣  Foxglove Studio:
   Browser: ws://<ROBOT_IP>:8765

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🔍 MONITORING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Camera:      ros2 topic hz /camera/image_raw
IMU:         ros2 topic echo /imu/data_raw
ToF:         ros2 topic echo /range
Motors:      ros2 topic echo /joint_states
Commands:    ros2 topic echo /diff_drive_controller/cmd_vel_unstamped

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

⚠️  TROUBLESHOOTING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

No nodes visible:
  → Check ROS_DOMAIN_ID=42 on both machines
  → ros2 daemon stop && ros2 daemon start

Pico not connected:
  → ls -l /dev/ttyACM*
  → cd firmware && make flash

Camera failed:
  → ls -l /dev/v4l/by-id/
  → Adjust device in hardware_bringup.launch.py

Foxglove not connecting:
  → Check firewall: sudo ufw allow 8765
  → Verify bridge: ros2 node info /foxglove_bridge

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📚 Full Documentation: HARDWARE_SETUP_PLAN.md

╚══════════════════════════════════════════════════════════════════╝
EOF
