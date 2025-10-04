#!/bin/bash

echo "🚀 Starte Gazebo micro-ROS Test"
echo ""

# Environment setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Überprüfe ob micro-ROS Agent läuft
if ! pgrep -f "micro_ros_agent" >/dev/null; then
    echo "❌ micro-ROS Agent läuft nicht!"
    echo "Starte zuerst: ./proper_test_procedure.sh"
    exit 1
fi

echo "✅ micro-ROS Agent läuft"

# Überprüfe Hardware-Topics
echo "📡 Überprüfe Hardware-Topics..."
if ros2 topic list | grep -q "/ddd/odom"; then
    echo "✅ Hardware Odometry verfügbar"
else
    echo "❌ Hardware Odometry nicht verfügbar"
    echo "Überprüfe Pico-Verbindung"
    exit 1
fi

echo ""
echo "🎮 Starte Gazebo Simulation..."

# Starte Gazebo im Hintergrund
ros2 launch robot_gazebo simulation.launch.py rviz:=true robot_model:=robot_xl & 
GAZEBO_PID=$!

echo "Gazebo PID: $GAZEBO_PID"
echo "Warte auf Gazebo-Start..."
sleep 15

echo ""
echo "📊 Starte Odometry-Vergleich..."

# Starte Python-Vergleichstool
python3 gazebo_comparison_setup.py &
COMPARISON_PID=$!

echo "Vergleichstool PID: $COMPARISON_PID"

echo ""
echo "🎯 Test-Setup bereit!"
echo ""
echo "=== Verfügbare Tests ==="
echo ""
echo "1. Manuelle Steuerung (Hardware):"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/ddd/cmd_vel"
echo ""
echo "2. Manuelle Steuerung (Simulation):"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "3. Testbewegungen:"
echo "   # Vorwärts (Hardware)"
echo "   ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'"
echo ""
echo "   # Vorwärts (Simulation)"
echo "   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'"
echo ""
echo "   # Drehen (beide)"
echo "   ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'"
echo "   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'"
echo ""
echo "4. Topics überwachen:"
echo "   ros2 topic echo /ddd/odom"
echo "   ros2 topic echo /odom"
echo ""
echo "=== In RViz ==="
echo "Füge folgende Topics hinzu:"
echo "- /ddd/odom (Hardware, Farbe: Rot)"
echo "- /odom (Simulation, Farbe: Blau)"
echo "- /joint_states"
echo "- /scan (falls verfügbar)"
echo ""
echo "Drücke Enter um alle Prozesse zu beenden..."
read

echo ""
echo "🛑 Beende alle Prozesse..."
kill $GAZEBO_PID $COMPARISON_PID 2>/dev/null
pkill -f "ros2 launch robot_gazebo" 2>/dev/null
pkill -f "gazebo" 2>/dev/null
pkill -f "rviz" 2>/dev/null

echo "✅ Test beendet"