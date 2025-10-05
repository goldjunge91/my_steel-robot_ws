#!/bin/bash
# Install main robot system as systemd service

echo "=== Installing Robot System Service ==="

# Create systemd service file
sudo tee /etc/systemd/system/my-steel-robot.service > /dev/null <<EOF
[Unit]
Description=my_steel Robot Main System
After=network.target microros-agent.service
Wants=network.target
Requires=microros-agent.service

[Service]
Type=simple
User=pi
Group=pi
WorkingDirectory=/home/pi/workspace/ros2_dev_ws/my_steel-robot_ws
Environment="ROS_DISTRO=humble"
Environment="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
Environment="ROS_DOMAIN_ID=0"
ExecStartPre=/bin/bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash'
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_bringup bringup.launch.py robot_model:=robot_xl mecanum:=True microros:=False'
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd and enable service
sudo systemctl daemon-reload
sudo systemctl enable my-steel-robot.service

echo "=== Robot Service installed! ==="
echo ""
echo "Commands:"
echo "Start:   sudo systemctl start my-steel-robot"
echo "Stop:    sudo systemctl stop my-steel-robot"
echo "Status:  sudo systemctl status my-steel-robot"
echo "Logs:    journalctl -u my-steel-robot -f"
echo ""
echo "Service will start automatically on boot!"