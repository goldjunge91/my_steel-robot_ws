#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Illuminance
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class RvizConverter(Node):
    def __init__(self):
        super().__init__('rviz_converter')
        
        # Subscriber für den Abstandssensor
        self.create_subscription(Range, '/ddd/range_tof', self.range_callback, 10)
        # Subscriber für den Helligkeitssensor
        self.create_subscription(Illuminance, '/ddd/illuminance', self.illuminance_callback, 10)
        
        # Publisher für die Visualisierungs-Marker
        self.range_marker_pub = self.create_publisher(Marker, '/ddd/range_marker', 10)
        self.illuminance_marker_pub = self.create_publisher(Marker, '/ddd/illuminance_marker', 10)
        self.get_logger().info("RViz Converter Node started.")

    def range_callback(self, msg: Range):
        marker = Marker()
        marker.header = msg.header
        marker.ns = "range"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Der Pfeil startet am Sensor und zeigt nach vorne
        # Länge des Pfeils = gemessene Distanz
        marker.scale.x = float(msg.range)
        marker.scale.y = 0.01 # Dicke
        marker.scale.z = 0.01 # Dicke
        
        # Farbe basierend auf der Distanz (grün = nah, rot = fern)
        intensity = min(float(msg.range) / msg.max_range, 1.0)
        marker.color = ColorRGBA(r=intensity, g=1.0 - intensity, b=0.0, a=1.0)
        
        self.range_marker_pub.publish(marker)

    def illuminance_callback(self, msg: Illuminance):
        marker = Marker()
        # WICHTIG: Der Marker muss im richtigen Frame sein, damit TF funktioniert
        marker.header.frame_id = "tof_link" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "illuminance"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Die Größe und Farbe der Kugel repräsentiert die Helligkeit
        # Skaliere den Lux-Wert auf eine sinnvolle Größe (z.B. 0.01 bis 0.2 m)
        size = min(max(float(msg.illuminance) / 500.0, 0.02), 0.2)
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        
        # Farbe ändert sich von blau (dunkel) zu gelb (hell)
        intensity = min(float(msg.illuminance) / 1000.0, 1.0)
        marker.color = ColorRGBA(r=intensity, g=intensity, b=1.0-intensity, a=0.8)

        # Positioniere die Kugel leicht über dem Sensor
        marker.pose.position.z = 0.05
        
        self.illuminance_marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RvizConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Range
# from visualization_msgs.msg import Marker
# from std_msgs.msg import ColorRGBA
# from builtin_interfaces.msg import Duration as BuiltinDuration

# class RangeToMarker(Node):
#     def __init__(self):
#         super().__init__('range_to_marker')
#         self.pub = self.create_publisher(Marker, 'visualization_marker', 10)
#         self.sub = self.create_subscription(Range, '/ddd/range_tof', self.cb, 10)

#     def cb(self, msg: Range):
#         m = Marker()
#         m.header = msg.header
#         m.ns = 'tof'
#         m.id = 0
#         m.type = Marker.SPHERE
#         m.action = Marker.ADD
#         # Platzieren entlang +X in Sensor-Frame (du kannst das anpassen)
#         m.pose.position.x = float(msg.range)  # Distanz entlang x
#         m.pose.position.y = 0.0
#         m.pose.position.z = 0.0
#         m.pose.orientation.w = 1.0
#         # Kleine Kugel
#         m.scale.x = 0.05
#         m.scale.y = 0.05
#         m.scale.z = 0.05
#         m.color.r = 0.0
#         m.color.g = 1.0
#         m.color.b = 0.0
#         m.color.a = 1.0
#         # Use builtin_interfaces.msg.Duration for lifetime
#         m.lifetime = BuiltinDuration(sec=0, nanosec=int(0.5 * 1e9))
#         self.pub.publish(m)

# def main(args=None):
#     rclpy.init(args=args)
#     node = RangeToMarker()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()