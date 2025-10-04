<<<<<<< HEAD
#!/bin/bash

echo "🚀 Einfacher Gazebo micro-ROS Test"
echo ""

# Environment setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Überprüfe ob micro-ROS Agent läuft
if ! pgrep -f "micro_ros_agent" >/dev/null; then
    echo "❌ micro-ROS Agent läuft nicht!"
    echo "Starte zuerst den Agent mit: ./proper_test_procedure.sh"
    exit 1
fi

echo "✅ micro-ROS Agent läuft"

# Überprüfe Hardware-Topics
echo "📡 Überprüfe Hardware-Topics..."
if ros2 topic list | grep -q "/ddd/odom"; then
    echo "✅ Hardware Odometry verfügbar (/ddd/odom)"
    HARDWARE_RATE=$(timeout 3 ros2 topic hz /ddd/odom 2>/dev/null | grep "average rate" | tail -1 | cut -d' ' -f3)
    if [ -n "$HARDWARE_RATE" ]; then
        echo "   Rate: ${HARDWARE_RATE} Hz"
    fi
else
    echo "❌ Hardware Odometry nicht verfügbar"
    exit 1
fi

echo ""
echo "🎮 Starte Gazebo mit korrektem robot_model..."

# Starte Gazebo mit explizitem robot_model Parameter
ros2 launch robot_gazebo simulation.launch.py robot_model:=robot rviz:=false &
GAZEBO_PID=$!

echo "Gazebo PID: $GAZEBO_PID"
echo "Warte auf Gazebo-Start (20 Sekunden)..."

# Warte und überwache Gazebo-Start
for i in {1..20}; do
    if ! ps -p $GAZEBO_PID > /dev/null 2>&1; then
        echo "❌ Gazebo ist abgestürzt"
        exit 1
    fi
    echo -n "."
    sleep 1
done
echo ""

echo "📊 Überprüfe Simulation-Topics..."
sleep 5

# Zeige verfügbare Topics
echo "Alle verfügbaren Topics:"
ros2 topic list | sort

echo ""
echo "🔍 Suche Simulation-Odometry..."

# Suche nach Simulation-Odometry Topics
SIM_ODOM_TOPICS=$(ros2 topic list | grep -E "(odom|odometry)" | grep -v "/ddd/")
if [ -n "$SIM_ODOM_TOPICS" ]; then
    echo "✅ Gefundene Simulation-Odometry Topics:"
    echo "$SIM_ODOM_TOPICS" | while read topic; do
        TYPE=$(ros2 topic info "$topic" 2>/dev/null | grep "Type:" | cut -d' ' -f2)
        echo "   $topic ($TYPE)"
    done
    SIM_ODOM_TOPIC=$(echo "$SIM_ODOM_TOPICS" | head -1)
else
    echo "❌ Keine Simulation-Odometry gefunden"
    echo "Verfügbare Topics:"
    ros2 topic list | grep -v "/ddd/"
    SIM_ODOM_TOPIC=""
fi

echo ""
echo "🎯 Starte Vergleichstest..."

# Erstelle temporäres Python-Script für Vergleich
cat > /tmp/quick_comparison.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import sys
import time

class QuickComparison(Node):
    def __init__(self, sim_topic):
        super().__init__('quick_comparison')
        
        self.hw_data = None
        self.sim_data = None
        
        # Hardware Subscription
        self.hw_sub = self.create_subscription(
            Odometry, '/ddd/odom', self.hw_callback, 10)
            
        # Simulation Subscription (falls verfügbar)
        if sim_topic:
            self.sim_sub = self.create_subscription(
                Odometry, sim_topic, self.sim_callback, 10)
        else:
            self.sim_sub = None
            
        self.timer = self.create_timer(2.0, self.print_status)
        self.start_time = time.time()
        
    def hw_callback(self, msg):
        self.hw_data = msg
        
    def sim_callback(self, msg):
        self.sim_data = msg
        
    def print_status(self):
        elapsed = time.time() - self.start_time
        print(f"\n⏱️  Zeit: {elapsed:.1f}s")
        
        if self.hw_data:
            x = self.hw_data.pose.pose.position.x
            y = self.hw_data.pose.pose.position.y
            vx = self.hw_data.twist.twist.linear.x
            vz = self.hw_data.twist.twist.angular.z
            print(f"🤖 HARDWARE: pos=({x:.3f}, {y:.3f}), vel=({vx:.3f}, {vz:.3f})")
        else:
            print("🤖 HARDWARE: ❌ Keine Daten")
            
        if self.sim_sub and self.sim_data:
            x = self.sim_data.pose.pose.position.x
            y = self.sim_data.pose.pose.position.y
            vx = self.sim_data.twist.twist.linear.x
            vz = self.sim_data.twist.twist.angular.z
            print(f"🎮 SIMULATION: pos=({x:.3f}, {y:.3f}), vel=({vx:.3f}, {vz:.3f})")
        elif self.sim_sub:
            print("🎮 SIMULATION: ❌ Keine Daten")
        else:
            print("🎮 SIMULATION: ❌ Topic nicht verfügbar")
            
        if elapsed > 30:  # Nach 30 Sekunden beenden
            print("\n✅ Test abgeschlossen")
            rclpy.shutdown()

def main():
    rclpy.init()
    sim_topic = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] != "None" else None
    node = QuickComparison(sim_topic)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
EOF

# Starte Vergleich
python3 /tmp/quick_comparison.py "$SIM_ODOM_TOPIC" &
COMPARISON_PID=$!

echo "Vergleich läuft... (30 Sekunden)"
echo ""
echo "🎮 Teste Bewegungskommandos..."

sleep 3

# Sende Testbewegungen
echo "Sende Vorwärts-Kommando an Hardware..."
ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" 

if [ -n "$SIM_ODOM_TOPIC" ]; then
    echo "Sende Vorwärts-Kommando an Simulation..."
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
fi

sleep 5

echo "Sende Dreh-Kommando..."
ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"

if [ -n "$SIM_ODOM_TOPIC" ]; then
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"
fi

sleep 5

echo "Sende Stop-Kommando..."
ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist "{}"

if [ -n "$SIM_ODOM_TOPIC" ]; then
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
fi

echo ""
echo "⏳ Warte auf Test-Abschluss..."
wait $COMPARISON_PID

echo ""
echo "🛑 Beende Gazebo..."
kill $GAZEBO_PID 2>/dev/null
sleep 2
pkill -f "gazebo" 2>/dev/null

echo ""
echo "✅ Test abgeschlossen!"
echo ""
echo "=== Nächste Schritte ==="
echo "1. Für detaillierten Vergleich mit RViz:"
echo "   ros2 launch robot_gazebo simulation.launch.py robot_model:=robot"
echo ""
echo "2. Manuelle Steuerung testen:"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/ddd/cmd_vel"
echo ""
echo "3. Hardware-Odometry überwachen:"
=======
#!/bin/bash

echo "🚀 Einfacher Gazebo micro-ROS Test"
echo ""

# Environment setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Überprüfe ob micro-ROS Agent läuft
if ! pgrep -f "micro_ros_agent" >/dev/null; then
    echo "❌ micro-ROS Agent läuft nicht!"
    echo "Starte zuerst den Agent mit: ./proper_test_procedure.sh"
    exit 1
fi

echo "✅ micro-ROS Agent läuft"

# Überprüfe Hardware-Topics
echo "📡 Überprüfe Hardware-Topics..."
if ros2 topic list | grep -q "/ddd/odom"; then
    echo "✅ Hardware Odometry verfügbar (/ddd/odom)"
    HARDWARE_RATE=$(timeout 3 ros2 topic hz /ddd/odom 2>/dev/null | grep "average rate" | tail -1 | cut -d' ' -f3)
    if [ -n "$HARDWARE_RATE" ]; then
        echo "   Rate: ${HARDWARE_RATE} Hz"
    fi
else
    echo "❌ Hardware Odometry nicht verfügbar"
    exit 1
fi

echo ""
echo "🎮 Starte Gazebo mit korrektem robot_model..."

# Starte Gazebo mit explizitem robot_model Parameter
ros2 launch robot_gazebo simulation.launch.py robot_model:=robot rviz:=false &
GAZEBO_PID=$!

echo "Gazebo PID: $GAZEBO_PID"
echo "Warte auf Gazebo-Start (20 Sekunden)..."

# Warte und überwache Gazebo-Start
for i in {1..20}; do
    if ! ps -p $GAZEBO_PID > /dev/null 2>&1; then
        echo "❌ Gazebo ist abgestürzt"
        exit 1
    fi
    echo -n "."
    sleep 1
done
echo ""

echo "📊 Überprüfe Simulation-Topics..."
sleep 5

# Zeige verfügbare Topics
echo "Alle verfügbaren Topics:"
ros2 topic list | sort

echo ""
echo "🔍 Suche Simulation-Odometry..."

# Suche nach Simulation-Odometry Topics
SIM_ODOM_TOPICS=$(ros2 topic list | grep -E "(odom|odometry)" | grep -v "/ddd/")
if [ -n "$SIM_ODOM_TOPICS" ]; then
    echo "✅ Gefundene Simulation-Odometry Topics:"
    echo "$SIM_ODOM_TOPICS" | while read topic; do
        TYPE=$(ros2 topic info "$topic" 2>/dev/null | grep "Type:" | cut -d' ' -f2)
        echo "   $topic ($TYPE)"
    done
    SIM_ODOM_TOPIC=$(echo "$SIM_ODOM_TOPICS" | head -1)
else
    echo "❌ Keine Simulation-Odometry gefunden"
    echo "Verfügbare Topics:"
    ros2 topic list | grep -v "/ddd/"
    SIM_ODOM_TOPIC=""
fi

echo ""
echo "🎯 Starte Vergleichstest..."

# Erstelle temporäres Python-Script für Vergleich
cat > /tmp/quick_comparison.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import sys
import time

class QuickComparison(Node):
    def __init__(self, sim_topic):
        super().__init__('quick_comparison')
        
        self.hw_data = None
        self.sim_data = None
        
        # Hardware Subscription
        self.hw_sub = self.create_subscription(
            Odometry, '/ddd/odom', self.hw_callback, 10)
            
        # Simulation Subscription (falls verfügbar)
        if sim_topic:
            self.sim_sub = self.create_subscription(
                Odometry, sim_topic, self.sim_callback, 10)
        else:
            self.sim_sub = None
            
        self.timer = self.create_timer(2.0, self.print_status)
        self.start_time = time.time()
        
    def hw_callback(self, msg):
        self.hw_data = msg
        
    def sim_callback(self, msg):
        self.sim_data = msg
        
    def print_status(self):
        elapsed = time.time() - self.start_time
        print(f"\n⏱️  Zeit: {elapsed:.1f}s")
        
        if self.hw_data:
            x = self.hw_data.pose.pose.position.x
            y = self.hw_data.pose.pose.position.y
            vx = self.hw_data.twist.twist.linear.x
            vz = self.hw_data.twist.twist.angular.z
            print(f"🤖 HARDWARE: pos=({x:.3f}, {y:.3f}), vel=({vx:.3f}, {vz:.3f})")
        else:
            print("🤖 HARDWARE: ❌ Keine Daten")
            
        if self.sim_sub and self.sim_data:
            x = self.sim_data.pose.pose.position.x
            y = self.sim_data.pose.pose.position.y
            vx = self.sim_data.twist.twist.linear.x
            vz = self.sim_data.twist.twist.angular.z
            print(f"🎮 SIMULATION: pos=({x:.3f}, {y:.3f}), vel=({vx:.3f}, {vz:.3f})")
        elif self.sim_sub:
            print("🎮 SIMULATION: ❌ Keine Daten")
        else:
            print("🎮 SIMULATION: ❌ Topic nicht verfügbar")
            
        if elapsed > 30:  # Nach 30 Sekunden beenden
            print("\n✅ Test abgeschlossen")
            rclpy.shutdown()

def main():
    rclpy.init()
    sim_topic = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] != "None" else None
    node = QuickComparison(sim_topic)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
EOF

# Starte Vergleich
python3 /tmp/quick_comparison.py "$SIM_ODOM_TOPIC" &
COMPARISON_PID=$!

echo "Vergleich läuft... (30 Sekunden)"
echo ""
echo "🎮 Teste Bewegungskommandos..."

sleep 3

# Sende Testbewegungen
echo "Sende Vorwärts-Kommando an Hardware..."
ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" 

if [ -n "$SIM_ODOM_TOPIC" ]; then
    echo "Sende Vorwärts-Kommando an Simulation..."
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
fi

sleep 5

echo "Sende Dreh-Kommando..."
ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"

if [ -n "$SIM_ODOM_TOPIC" ]; then
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"
fi

sleep 5

echo "Sende Stop-Kommando..."
ros2 topic pub --once /ddd/cmd_vel geometry_msgs/msg/Twist "{}"

if [ -n "$SIM_ODOM_TOPIC" ]; then
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
fi

echo ""
echo "⏳ Warte auf Test-Abschluss..."
wait $COMPARISON_PID

echo ""
echo "🛑 Beende Gazebo..."
kill $GAZEBO_PID 2>/dev/null
sleep 2
pkill -f "gazebo" 2>/dev/null

echo ""
echo "✅ Test abgeschlossen!"
echo ""
echo "=== Nächste Schritte ==="
echo "1. Für detaillierten Vergleich mit RViz:"
echo "   ros2 launch robot_gazebo simulation.launch.py robot_model:=robot"
echo ""
echo "2. Manuelle Steuerung testen:"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/ddd/cmd_vel"
echo ""
echo "3. Hardware-Odometry überwachen:"
>>>>>>> 0397074 (changed naming)
echo "   ros2 topic echo /ddd/odom"