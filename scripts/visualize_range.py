#!/usr/bin/env python3
"""
visualize_range.py

Kleines Hilfsprogramm zum Visualisieren von sensor_msgs/Range in rviz2.

Usage:
  source install/local_setup.sh   # oder /opt/ros/humble/setup.bash und workspace local_setup
  python3 scripts/visualize_range.py --topic /range --frame pico_link

Das Skript abonniert ein Range-Topic und publisht einen visualization_msgs/Marker
anhand des gemessenen Abstands. Praktisch für Ultraschall-/Entfernungssensoren am Pico.
"""
import argparse
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker


class RangeVisualizer(Node):
    def __init__(self, topic: str, frame: str, marker_scale: float, ns: str):
        super().__init__('range_visualizer')
        self.topic = topic
        self.frame = frame
        self.ns = ns
        self.marker_scale = marker_scale

        self.pub = self.create_publisher(Marker, topic + '/marker', 10)
        self.sub = self.create_subscription(Range, topic, self.cb_range, 10)
        self.get_logger().info(f'Bones: visualizing {topic} in frame "{frame}"')

    def cb_range(self, msg: Range):
        # Erstelle einfachen Marker: Kugel am gemessenen Abstand entlang der x-Achse
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.ns = self.ns
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        # Position: entlang x-Achse mit Abstand = msg.range
        dist = float(msg.range) if not math.isnan(msg.range) else 0.0
        m.pose.position.x = dist
        m.pose.position.y = 0.0
        m.pose.position.z = 0.0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0

        # Größe & Farbe
        m.scale.x = self.marker_scale
        m.scale.y = self.marker_scale
        m.scale.z = self.marker_scale
        # orange-ish
        m.color.r = 1.0
        m.color.g = 0.5
        m.color.b = 0.0
        m.color.a = 0.9

        # kurze Lebensdauer, damit bei Ausfall der Daten nichts hängen bleibt
        m.lifetime.sec = 1
        m.lifetime.nanosec = 0

        self.pub.publish(m)


def main():
    parser = argparse.ArgumentParser(description='Visualize sensor_msgs/Range as Marker in rviz2')
    parser.add_argument('--topic', '-t', default='/range', help='Input Range topic')
    parser.add_argument('--frame', '-f', default='pico_link', help='frame_id for marker')
    parser.add_argument('--scale', '-s', type=float, default=0.05, help='marker scale (m)')
    parser.add_argument('--ns', default='pico_range', help='marker namespace')
    args = parser.parse_args()

    rclpy.init()
    node = RangeVisualizer(args.topic, args.frame, args.scale, args.ns)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
