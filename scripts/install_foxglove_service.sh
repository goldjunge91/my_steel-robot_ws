#!/bin/bash
# Install Foxglove Bridge as systemd service

echo "=== Installing Foxglove Bridge Service ==="

# Create systemd service file
sudo tee /etc/systemd/system/foxglove-bridge.service > /dev/null <<EOF
[Unit]
Description=Foxglove Bridge for my_steel Robot
After=network.target my-steel-robot.service
Wants=network.target
Requires=my-steel-robot.service

[Service]
Type=simple
User=pi
Group=pi
WorkingDirectory=/home/pi/workspace/ros2_dev_ws/my_steel-robot_ws
Environment="ROS_DISTRO=humble"
Environment="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
Environment="ROS_DOMAIN_ID=0"
ExecStartPre=/bin/bash -c 'source /opt/ros/humble/setup.bash'
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && ros2 run foxglove_bridge foxglove_bridge --port 8765'
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd and enable service
sudo systemctl daemon-reload
sudo systemctl enable foxglove-bridge.service

echo "=== Foxglove Service installed! ==="
echo ""
echo "Commands:"
echo "Start:   sudo systemctl start foxglove-bridge"
echo "Stop:    sudo systemctl stop foxglove-bridge"
echo "Status:  sudo systemctl status foxglove-bridge"
echo "Logs:    journalctl -u foxglove-bridge -f"
echo ""
echo "Service will start automatically on boot!"