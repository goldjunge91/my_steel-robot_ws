#!/bin/bash

echo "=== Gazebo micro-ROS Integration Test ==="
echo ""

# ROS2 Environment setup
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "1. Überprüfe verfügbare Pakete..."
if ! ros2 pkg list | grep -q "robot_gazebo"; then
    echo "❌ robot_gazebo Paket nicht gefunden"
    echo "Baue Workspace..."
    colcon build --packages-select robot_gazebo robot_description robot_controller
    source install/setup.bash
fi

echo "✅ robot Pakete verfügbar"

echo ""
echo "2. Starte micro-ROS Agent (falls nicht läuft)..."
if ! pgrep -f "micro_ros_agent" >/dev/null; then
    echo "Starte micro-ROS Agent..."
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 &
    AGENT_PID=$!
    echo "Agent gestartet mit PID: $AGENT_PID"
    sleep 3
else
    echo "✅ micro-ROS Agent läuft bereits"
fi

echo ""
echo "3. Überprüfe Hardware-Topics..."
echo "Verfügbare Hardware-Topics:"
ros2 topic list | grep -E "(ddd|joint|pico)" | while read topic; do
    echo "   $topic"
done

echo ""
echo "4. Starte Gazebo Simulation..."
echo "Starte Gazebo im Hintergrund..."

# Gazebo mit robot starten
ros2 launch robot_gazebo simulation.launch.py rviz:=false &
GAZEBO_PID=$!
echo "Gazebo gestartet mit PID: $GAZEBO_PID"

echo ""
echo "5. Warte auf Gazebo-Initialisierung..."
sleep 10

echo ""
echo "6. Überprüfe Simulation-Topics..."
echo "Verfügbare Simulation-Topics:"
ros2 topic list | grep -E "(cmd_vel|odom|joint_states|scan)" | while read topic; do
    TYPE=$(ros2 topic info "$topic" 2>/dev/null | grep "Type:" | cut -d' ' -f2)
    echo "   $topic ($TYPE)"
done

echo ""
echo "7. Erstelle Topic-Bridge zwischen Hardware und Simulation..."

# Bridge-Konfiguration erstellen
cat > /tmp/microros_gazebo_bridge.yaml << EOF
# Bridge zwischen micro-ROS Hardware und Gazebo Simulation
bridges:
  # Hardware -> Simulation
  - from_topic: /ddd/odom
    to_topic: /hardware/odom
    msg_type: nav_msgs/msg/Odometry
    
  - from_topic: /joint_states  
    to_topic: /hardware/joint_states
    msg_type: sensor_msgs/msg/JointState
    
  # Simulation -> Hardware (für Vergleich)
  - from_topic: /odom
    to_topic: /simulation/odom
    msg_type: nav_msgs/msg/Odometry
    
  - from_topic: /cmd_vel
    to_topic: /ddd/cmd_vel
    msg_type: geometry_msgs/msg/Twist
EOF

echo "✅ Bridge-Konfiguration erstellt"

echo ""
echo "8. Starte RViz für Visualisierung..."
ros2 launch robot_description rviz.launch.py &
RVIZ_PID=$!
echo "RViz gestartet mit PID: $RVIZ_PID"

sleep 5

echo ""
echo "9. Teste Bewegungskommandos..."
echo "Sende Testbewegung an Hardware..."

# Testbewegung für Hardware
ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" &

# Testbewegung für Simulation  
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" &

echo ""
echo "10. Überwache Odometry-Vergleich (10 Sekunden)..."

# Hardware Odometry
echo "Hardware Odometry Rate:"
timeout 5 ros2 topic hz /ddd/odom 2>/dev/null &

# Simulation Odometry  
echo "Simulation Odometry Rate:"
timeout 5 ros2 topic hz /odom 2>/dev/null &

sleep 6

echo ""
echo "11. Zeige aktuelle Positionen..."

echo "Hardware Position:"
timeout 2 ros2 topic echo /ddd/odom --field pose.pose.position --once 2>/dev/null || echo "Keine Hardware-Daten"

echo "Simulation Position:"  
timeout 2 ros2 topic echo /odom --field pose.pose.position --once 2>/dev/null || echo "Keine Simulation-Daten"

echo ""
echo "=== Test-Ergebnisse ==="
echo ""

# Überprüfe ob alle Prozesse laufen
if ps -p $GAZEBO_PID > /dev/null 2>&1; then
    echo "✅ Gazebo läuft"
else
    echo "❌ Gazebo abgestürzt"
fi

if ps -p $RVIZ_PID > /dev/null 2>&1; then
    echo "✅ RViz läuft"
else
    echo "❌ RViz abgestürzt"
fi

if pgrep -f "micro_ros_agent" >/dev/null; then
    echo "✅ micro-ROS Agent läuft"
else
    echo "❌ micro-ROS Agent nicht verfügbar"
fi

echo ""
echo "=== Nächste Schritte ==="
echo "1. In RViz: Füge Topics hinzu:"
echo "   - /ddd/odom (Hardware Odometry)"
echo "   - /odom (Simulation Odometry)"
echo "   - /joint_states (Hardware Joint States)"
echo ""
echo "2. Teste manuelle Steuerung:"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/ddd/cmd_vel"
echo ""
echo "3. Vergleiche Hardware vs Simulation:"
echo "   ros2 topic echo /ddd/odom"
echo "   ros2 topic echo /odom"
echo ""
echo "Zum Beenden aller Prozesse:"
echo "kill $GAZEBO_PID $RVIZ_PID"
if [ -n "$AGENT_PID" ]; then
    echo "kill $AGENT_PID"
fi