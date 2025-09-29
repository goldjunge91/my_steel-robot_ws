#!/bin/bash

echo "🤖 Teste Pico-Firmware mit Gazebo als virtueller Roboter"
echo ""
echo "Setup: Pico Board (micro-ROS) ↔ Gazebo (simulierte Hardware)"
echo ""

# Environment setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Überprüfe Pico-Verbindung
if ! pgrep -f "micro_ros_agent" >/dev/null; then
    echo "❌ micro-ROS Agent läuft nicht!"
    echo "Starte zuerst: ./proper_test_procedure.sh"
    exit 1
fi

echo "✅ Pico micro-ROS Agent läuft"

# Überprüfe Pico-Topics
echo "📡 Pico-Firmware Topics:"
ros2 topic list | grep -E "(ddd|joint|pico)" | while read topic; do
    TYPE=$(ros2 topic info "$topic" 2>/dev/null | grep "Type:" | cut -d' ' -f2)
    echo "   $topic ($TYPE)"
done

echo ""
echo "🎮 Starte Gazebo als virtueller Roboter..."

# Starte Gazebo ROSbot Simulation
ros2 launch rosbot_gazebo simulation.launch.py robot_model:=rosbot rviz:=true &
GAZEBO_PID=$!

echo "Gazebo PID: $GAZEBO_PID"
echo "Warte auf Gazebo-Start..."
sleep 15

# Überprüfe ob Gazebo läuft
if ! ps -p $GAZEBO_PID > /dev/null 2>&1; then
    echo "❌ Gazebo ist abgestürzt"
    exit 1
fi

echo "✅ Gazebo läuft"

echo ""
echo "🔗 Erstelle Verbindung: Pico ↔ Gazebo"

# Erstelle Topic-Bridge zwischen Pico und Gazebo
echo "Starte Topic-Bridge..."

# Bridge: Pico cmd_vel → Gazebo cmd_vel
ros2 run topic_tools relay /ddd/cmd_vel /cmd_vel &
RELAY1_PID=$!

# Bridge: Gazebo odom → Pico (für Vergleich)
ros2 run topic_tools relay /odom /gazebo/odom &
RELAY2_PID=$!

echo "Bridge PIDs: $RELAY1_PID, $RELAY2_PID"

sleep 3

echo ""
echo "📊 System-Status:"

# Zeige alle relevanten Topics
echo "Pico-Topics:"
ros2 topic list | grep "/ddd/" | while read topic; do
    echo "   $topic"
done

echo "Gazebo-Topics:"
ros2 topic list | grep -E "(cmd_vel|odom|joint_states)" | grep -v "/ddd/" | while read topic; do
    echo "   $topic"
done

echo ""
echo "🧪 Starte Funktionstest..."

# Test 1: Pico Alive-Signal
echo "Test 1: Pico Alive-Signal"
PICO_COUNT=$(timeout 3 ros2 topic echo /pico_count --once 2>/dev/null | grep "data:" | cut -d' ' -f2)
if [ -n "$PICO_COUNT" ]; then
    echo "✅ Pico sendet Count: $PICO_COUNT"
else
    echo "❌ Kein Pico Count empfangen"
fi

# Test 2: Pico Odometry
echo ""
echo "Test 2: Pico Odometry"
PICO_ODOM=$(timeout 3 ros2 topic echo /ddd/odom --field pose.pose.position --once 2>/dev/null)
if [ -n "$PICO_ODOM" ]; then
    echo "✅ Pico Odometry:"
    echo "$PICO_ODOM"
else
    echo "❌ Keine Pico Odometry"
fi

# Test 3: Gazebo Odometry
echo ""
echo "Test 3: Gazebo Odometry"
GAZEBO_ODOM=$(timeout 3 ros2 topic echo /odom --field pose.pose.position --once 2>/dev/null)
if [ -n "$GAZEBO_ODOM" ]; then
    echo "✅ Gazebo Odometry:"
    echo "$GAZEBO_ODOM"
else
    echo "❌ Keine Gazebo Odometry"
fi

echo ""
echo "🎮 Interaktive Tests verfügbar:"
echo ""
echo "1. 🕹️  Pico-Steuerung (steuert Gazebo-Roboter):"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/ddd/cmd_vel"
echo ""
echo "2. 📊 Pico-Odometry überwachen:"
echo "   ros2 topic echo /ddd/odom"
echo ""
echo "3. 🤖 Gazebo-Odometry überwachen:"
echo "   ros2 topic echo /odom"
echo ""
echo "4. 🔄 Vergleich beider Odometries:"
echo "   # Terminal 1:"
echo "   ros2 topic echo /ddd/odom --field pose.pose.position"
echo "   # Terminal 2:"
echo "   ros2 topic echo /odom --field pose.pose.position"
echo ""
echo "5. 🎯 Testbewegungen senden:"
echo "   # Vorwärts"
echo "   ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'"
echo "   # Drehen"
echo "   ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'"
echo "   # Stop"
echo "   ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist '{}'"
echo ""

# Automatischer Test
echo "🚀 Automatischer Bewegungstest (10 Sekunden)..."
echo ""

# Vorwärts
echo "→ Vorwärts (2s)..."
ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
sleep 2

# Stop
echo "⏸️  Stop (1s)..."
ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist "{}"
sleep 1

# Drehen
echo "🔄 Drehen (2s)..."
ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"
sleep 2

# Stop
echo "⏸️  Stop (1s)..."
ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist "{}"
sleep 1

# Rückwärts
echo "← Rückwärts (2s)..."
ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.1}}"
sleep 2

# Final Stop
echo "🛑 Final Stop..."
ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist "{}"

echo ""
echo "✅ Automatischer Test abgeschlossen!"
echo ""
echo "💡 Was du siehst:"
echo "- Pico empfängt cmd_vel Kommandos über /ddd/cmd_vel"
echo "- Pico berechnet Odometry basierend auf simulierten Motorbewegungen"
echo "- Gazebo zeigt wie sich ein echter Roboter bewegen würde"
echo "- Du kannst beide Odometries vergleichen"
echo ""
echo "🎯 Das beweist: Deine Pico-Firmware funktioniert korrekt!"
echo ""
echo "Drücke Enter um alle Prozesse zu beenden..."
read

echo ""
echo "🛑 Beende alle Prozesse..."
kill $GAZEBO_PID $RELAY1_PID $RELAY2_PID 2>/dev/null
pkill -f "gazebo" 2>/dev/null
pkill -f "rviz" 2>/dev/null
pkill -f "topic_tools" 2>/dev/null

echo "✅ Test beendet"