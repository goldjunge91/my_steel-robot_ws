#!/usr/bin/env python3
"""
Gazebo micro-ROS Comparison Tool
Vergleicht Hardware-Odometry mit Gazebo-Simulation
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time

class OdometryComparison(Node):
    def __init__(self):
        super().__init__('odometry_comparison')
        
        # Hardware Odometry (von micro-ROS)
        self.hw_odom_sub = self.create_subscription(
            Odometry,
            '/ddd/odom',
            self.hardware_odom_callback,
            10
        )
        
        # Simulation Odometry (von Gazebo)
        self.sim_odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.simulation_odom_callback,
            10
        )
        
        # Command publisher für beide
        self.hw_cmd_pub = self.create_publisher(Twist, '/ddd/cmd_vel', 10)
        self.sim_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer für regelmäßige Ausgabe
        self.timer = self.create_timer(2.0, self.print_comparison)
        
        # Daten speichern
        self.hw_odom = None
        self.sim_odom = None
        self.hw_last_time = None
        self.sim_last_time = None
        
        self.get_logger().info("Odometry Comparison Node gestartet")
        self.get_logger().info("Hardware Topic: /ddd/odom")
        self.get_logger().info("Simulation Topic: /odom")
        
    def hardware_odom_callback(self, msg):
        self.hw_odom = msg
        self.hw_last_time = time.time()
        
    def simulation_odom_callback(self, msg):
        self.sim_odom = msg
        self.sim_last_time = time.time()
        
    def print_comparison(self):
        current_time = time.time()
        
        print("\n" + "="*60)
        print("ODOMETRY COMPARISON")
        print("="*60)
        
        # Hardware Status
        if self.hw_odom and self.hw_last_time and (current_time - self.hw_last_time) < 5:
            hw_x = self.hw_odom.pose.pose.position.x
            hw_y = self.hw_odom.pose.pose.position.y
            hw_z = self.hw_odom.pose.pose.orientation.z
            hw_w = self.hw_odom.pose.pose.orientation.w
            hw_yaw = 2 * math.atan2(hw_z, hw_w) * 180 / math.pi
            
            print(f"🤖 HARDWARE (micro-ROS):")
            print(f"   Position: x={hw_x:.3f}, y={hw_y:.3f}")
            print(f"   Rotation: {hw_yaw:.1f}°")
            print(f"   Linear Vel: {self.hw_odom.twist.twist.linear.x:.3f} m/s")
            print(f"   Angular Vel: {self.hw_odom.twist.twist.angular.z:.3f} rad/s")
        else:
            print("🤖 HARDWARE: ❌ Keine Daten empfangen")
            
        # Simulation Status
        if self.sim_odom and self.sim_last_time and (current_time - self.sim_last_time) < 5:
            sim_x = self.sim_odom.pose.pose.position.x
            sim_y = self.sim_odom.pose.pose.position.y
            sim_z = self.sim_odom.pose.pose.orientation.z
            sim_w = self.sim_odom.pose.pose.orientation.w
            sim_yaw = 2 * math.atan2(sim_z, sim_w) * 180 / math.pi
            
            print(f"🎮 SIMULATION (Gazebo):")
            print(f"   Position: x={sim_x:.3f}, y={sim_y:.3f}")
            print(f"   Rotation: {sim_yaw:.1f}°")
            print(f"   Linear Vel: {self.sim_odom.twist.twist.linear.x:.3f} m/s")
            print(f"   Angular Vel: {self.sim_odom.twist.twist.angular.z:.3f} rad/s")
        else:
            print("🎮 SIMULATION: ❌ Keine Daten empfangen")
            
        # Vergleich wenn beide verfügbar
        if (self.hw_odom and self.sim_odom and 
            self.hw_last_time and self.sim_last_time and
            (current_time - self.hw_last_time) < 5 and 
            (current_time - self.sim_last_time) < 5):
            
            dx = abs(hw_x - sim_x)
            dy = abs(hw_y - sim_y)
            distance = math.sqrt(dx*dx + dy*dy)
            
            print(f"📊 VERGLEICH:")
            print(f"   Positionsabweichung: {distance:.3f} m")
            print(f"   ΔX: {dx:.3f} m, ΔY: {dy:.3f} m")
            
            if distance < 0.1:
                print("   ✅ Sehr gute Übereinstimmung")
            elif distance < 0.5:
                print("   ⚠️  Moderate Abweichung")
            else:
                print("   ❌ Große Abweichung")
                
    def send_test_command(self, linear_x=0.1, angular_z=0.0):
        """Sendet Testkommando an beide Systeme"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        
        self.hw_cmd_pub.publish(cmd)
        self.sim_cmd_pub.publish(cmd)
        
        self.get_logger().info(f"Testkommando gesendet: linear={linear_x}, angular={angular_z}")

def main():
    rclpy.init()
    
    node = OdometryComparison()
    
    print("\n🚀 Gazebo micro-ROS Vergleich gestartet!")
    print("Drücke Ctrl+C zum Beenden")
    print("\nVerfügbare Kommandos:")
    print("- Vorwärts: ros2 service call /test_forward std_srvs/srv/Empty")
    print("- Drehen: ros2 service call /test_rotate std_srvs/srv/Empty")
    print("- Stop: ros2 service call /test_stop std_srvs/srv/Empty")
    
    try:
        # Sende initial ein Testkommando
        time.sleep(2)
        node.send_test_command(0.0, 0.0)  # Stop command
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 Beende Vergleich...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()