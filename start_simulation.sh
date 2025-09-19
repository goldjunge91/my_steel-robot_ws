#!/bin/bash

# Simulation Starter Script für Robot
# Dieses Script startet die Gazebo-Simulation und öffnet weitere nützliche Terminals

set -e

echo "🤖 Starting Robot Simulation Environment..."

# Source the workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Workspace sourced"
else
    echo "⚠️  Warning: install/setup.bash not found. Make sure to build first with 'colcon build'"
fi

# Start main simulation in background
echo "🚀 Starting Gazebo simulation..."
gnome-terminal --tab --title="Main Simulation" -- bash -c "
    source /opt/ros/humble/setup.bash
    if [ -f install/setup.bash ]; then source install/setup.bash; fi
    echo 'Starting main simulation...'
    ros2 launch robot launch_sim.launch.py
    exec bash
" &

# Wait a moment for simulation to start
sleep 3

# Terminal 2: Robot control (teleop)
echo "�� Opening teleop control terminal..."
gnome-terminal --tab --title="Robot Control" -- bash -c "
    source /opt/ros/humble/setup.bash
    if [ -f install/setup.bash ]; then source install/setup.bash; fi
    echo '🎮 Robot Control Terminal'
    echo 'Available commands:'
    echo '  ros2 run teleop_twist_keyboard teleop_twist_keyboard'
    echo '  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...'
    echo '  ros2 topic echo /mecanum_drive_controller/odometry'
    echo ''
    exec bash
" &

# Terminal 3: Monitoring and debugging
echo "🔍 Opening monitoring terminal..."
gnome-terminal --tab --title="Monitor & Debug" -- bash -c "
    source /opt/ros/humble/setup.bash
    if [ -f install/setup.bash ]; then source install/setup.bash; fi
    echo '🔍 Monitoring & Debug Terminal'
    echo 'Useful commands:'
    echo '  ros2 node list'
    echo '  ros2 topic list'
    echo '  ros2 service list'
    echo '  ros2 control list_controllers'
    echo '  rviz2'
    echo '  rqt'
    echo ''
    exec bash
" &

echo "✅ All terminals opened!"
echo ""
echo "📋 Summary:"
echo "   Terminal 1: Main Gazebo Simulation"
echo "   Terminal 2: Robot Control (teleop commands)"
echo "   Terminal 3: Monitoring & Debugging tools"
echo ""
echo "💡 Tip: Wait for Gazebo to fully load before sending commands"
