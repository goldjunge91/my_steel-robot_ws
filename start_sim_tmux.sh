#!/bin/bash

# Robot Simulation mit tmux (funktioniert besser in Containern)
# Dieses Script startet die Simulation in mehreren tmux-Fenstern

set -e

SESSION_NAME="robot_sim"

echo "🤖 Starting Robot Simulation with tmux..."

# Kill existing session if it exists
tmux kill-session -t $SESSION_NAME 2>/dev/null || true

# Create new tmux session
tmux new-session -d -s $SESSION_NAME -n "main"

# Source workspace in first window and start simulation
tmux send-keys -t $SESSION_NAME:main "source /opt/ros/humble/setup.bash" Enter
tmux send-keys -t $SESSION_NAME:main "if [ -f install/setup.bash ]; then source install/setup.bash; fi" Enter
tmux send-keys -t $SESSION_NAME:main "echo '🚀 Starting Gazebo simulation...'" Enter
tmux send-keys -t $SESSION_NAME:main "ros2 launch robot launch_sim.launch.py world:=src/robot/worlds/obstacles.world use_sim_time:=true" Enter

# Create second window for robot control
tmux new-window -t $SESSION_NAME -n "control"
tmux send-keys -t $SESSION_NAME:control "source /opt/ros/humble/setup.bash" Enter
tmux send-keys -t $SESSION_NAME:control "if [ -f install/setup.bash ]; then source install/setup.bash; fi" Enter
tmux send-keys -t $SESSION_NAME:control "echo '🎮 Robot Control Terminal'" Enter
tmux send-keys -t $SESSION_NAME:control "echo 'Available commands:'" Enter
tmux send-keys -t $SESSION_NAME:control "echo '  ros2 run teleop_twist_keyboard teleop_twist_keyboard'" Enter
tmux send-keys -t $SESSION_NAME:control "echo '  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.5}, angular: {z: 0.0}}\"'" Enter
tmux send-keys -t $SESSION_NAME:control "echo '  ros2 topic echo /mecanum_drive_controller/odometry'" Enter
tmux send-keys -t $SESSION_NAME:control "echo ''" Enter

# Create third window for monitoring
tmux new-window -t $SESSION_NAME -n "monitor"
tmux send-keys -t $SESSION_NAME:monitor "source /opt/ros/humble/setup.bash" Enter
tmux send-keys -t $SESSION_NAME:monitor "if [ -f install/setup.bash ]; then source install/setup.bash; fi" Enter
tmux send-keys -t $SESSION_NAME:monitor "echo '🔍 Monitoring & Debug Terminal'" Enter
tmux send-keys -t $SESSION_NAME:monitor "echo 'Useful commands:'" Enter
tmux send-keys -t $SESSION_NAME:monitor "echo '  ros2 node list'" Enter
tmux send-keys -t $SESSION_NAME:monitor "echo '  ros2 topic list'" Enter
tmux send-keys -t $SESSION_NAME:monitor "echo '  ros2 service list'" Enter
tmux send-keys -t $SESSION_NAME:monitor "echo '  ros2 control list_controllers'" Enter
tmux send-keys -t $SESSION_NAME:monitor "echo '  rviz2'" Enter
tmux send-keys -t $SESSION_NAME:monitor "echo '  rqt'" Enter
tmux send-keys -t $SESSION_NAME:monitor "echo ''" Enter

# Switch back to main window
tmux select-window -t $SESSION_NAME:main

echo "✅ Tmux session '$SESSION_NAME' created!"
echo ""
echo "📋 Usage:"
echo "  tmux attach -t $SESSION_NAME    # Attach to session"
echo "  Ctrl+b, n                       # Next window"
echo "  Ctrl+b, p                       # Previous window"
echo "  Ctrl+b, 0/1/2                   # Switch to window 0/1/2"
echo "  Ctrl+b, d                       # Detach from session"
echo ""
echo "💡 Tip: Wait for Gazebo to fully load before sending commands"
echo "🛑 To stop: tmux kill-session -t $SESSION_NAME"
