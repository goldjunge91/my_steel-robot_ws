#!/bin/bash
# Complete autostart setup for my_steel robot

echo "=== Setting up Complete Autostart System ==="

# Make scripts executable
chmod +x scripts/install_microros_service.sh
chmod +x scripts/install_robot_service.sh
chmod +x scripts/install_foxglove_service.sh

# Install all services
echo "Installing micro-ROS Agent service..."
bash scripts/install_microros_service.sh

echo ""
echo "Installing Robot System service..."
bash scripts/install_robot_service.sh

echo ""
echo "Installing Foxglove Bridge service..."
bash scripts/install_foxglove_service.sh

echo ""
echo "=== All Services Installed! ==="
echo ""
echo "Service startup order:"
echo "1. microros-agent      (micro-ROS communication)"
echo "2. my-steel-robot      (main robot system)"
echo "3. foxglove-bridge     (web interface)"
echo ""
echo "Management commands:"
echo "Start all:    sudo systemctl start microros-agent my-steel-robot foxglove-bridge"
echo "Stop all:     sudo systemctl stop foxglove-bridge my-steel-robot microros-agent"
echo "Status all:   sudo systemctl status microros-agent my-steel-robot foxglove-bridge"
echo ""
echo "View logs:"
echo "journalctl -u microros-agent -f"
echo "journalctl -u my-steel-robot -f"
echo "journalctl -u foxglove-bridge -f"
echo ""
echo "All services will start automatically on boot!"