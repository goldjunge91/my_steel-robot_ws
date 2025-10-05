#!/bin/bash
# Install micro-ROS agent as systemd service

echo "=== Installing micro-ROS Agent Service ==="

# Create systemd service file
sudo tee /etc/systemd/system/microros-agent.service > /dev/null <<EOF
[Unit]
Description=micro-ROS Agent for my_steel Robot
After=network.target
Wants=network.target

[Service]
Type=simple
User=pi
Group=pi
WorkingDirectory=/home/pi/workspace/ros2_dev_ws/my_steel-robot_ws
Environment="ROS_DISTRO=humble"
Environment="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
Environment="ROS_DOMAIN_ID=0"
ExecStartPre=/bin/bash -c 'source /opt/ros/humble/setup.bash'
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v6'
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd and enable service
sudo systemctl daemon-reload
sudo systemctl enable microros-agent.service

echo "=== Service installed! ==="
echo ""
echo "Commands:"
echo "Start:   sudo systemctl start microros-agent"
echo "Stop:    sudo systemctl stop microros-agent"
echo "Status:  sudo systemctl status microros-agent"
echo "Logs:    journalctl -u microros-agent -f"
echo ""
echo "Service will start automatically on boot!"