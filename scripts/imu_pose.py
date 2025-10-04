#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

class ImuToPoseConverter(Node):
    def __init__(self):
        super().__init__('imu_to_pose_converter')
        
        # Abonniert das IMU-Topic von deinem Pico
        self.create_subscription(
            Imu,
            '/ddd/imu',
            self.imu_callback,
            10
        )
        
        # Veröffentlicht das neue Pose-Topic für RViz2
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/ddd/imu_pose',
            10
        )
        
        self.get_logger().info("IMU to Pose Converter started. Listening on /ddd/imu...")

    def imu_callback(self, imu_msg: Imu):
        # Erstelle eine neue PoseStamped-Nachricht
        pose_msg = PoseStamped()
        
        # Kopiere den Header (Zeitstempel und Frame-ID) von der IMU-Nachricht
        pose_msg.header = imu_msg.header
        
        # Setze die Position auf Null, da wir nur die Orientierung visualisieren wollen
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        
        # Kopiere die Orientierungsdaten (Quaternion) von der IMU-Nachricht
        pose_msg.pose.orientation = imu_msg.orientation
        
        # Veröffentliche die neue Nachricht
        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuToPoseConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
