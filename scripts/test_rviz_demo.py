#!/usr/bin/env python3
"""
Lightweight demo that publishes sample topics (Range, Illuminance, Odometry, JointState, TF)
and optionally launches rviz2 using the existing wrapper `~/.local/bin/rviz2-no-snap`.

Usage:
  python3 scripts/test_rviz_demo.py [--no-rviz]

Run in a terminal with ROS2 environment sourced:
  source /opt/ros/humble/setup.bash
  python3 scripts/test_rviz_demo.py

The script is intended for quick local debugging and RViz visualization.
"""

import argparse
import math
import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range, Illuminance, JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf_transformations
from tf2_ros import TransformBroadcaster


class DemoNode(Node):
    def __init__(self):
        super().__init__('rviz_demo')
        # Publishers
        self.range_pub = self.create_publisher(Range, '/ddd/range_tof', 10)
        self.ill_pub = self.create_publisher(Illuminance, '/ddd/illuminance', 10)
        self.odom_pub = self.create_publisher(Odometry, '/ddd/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/ddd/imu', 10)
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.seq = 0

    def timer_cb(self):
        now = self.get_clock().now().to_msg()
        t = time.time() - self.start_time

        # Range
        r = Range()
        r.header.stamp = now
        r.header.frame_id = 'tof_link'
        r.radiation_type = Range.INFRARED
        r.field_of_view = 0.1
        r.min_range = 0.0
        r.max_range = 0.2
        r.range = 0.1 + 0.05 * math.sin(t * 2.0)
        self.range_pub.publish(r)

        # Illuminance
        ill = Illuminance()
        ill.header.stamp = now
        ill.header.frame_id = 'tof_link'
        ill.illuminance = 100.0 + 20.0 * math.sin(t)
        self.ill_pub.publish(ill)

        # Odometry (simple circular motion)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = 0.5 * math.cos(t * 0.5)
        odom.pose.pose.position.y = 0.5 * math.sin(t * 0.5)
        q = tf_transformations.quaternion_from_euler(0, 0, t * 0.5)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom.twist.twist.linear.x = -0.25 * math.sin(t * 0.5)
        odom.twist.twist.linear.y = 0.25 * math.cos(t * 0.5)
        self.odom_pub.publish(odom)

        # JointState with two joints
        js = JointState()
        js.header.stamp = now
        js.name = ['joint1', 'joint2']
        js.position = [0.1 * math.sin(t), 0.2 * math.cos(t)]
        js.velocity = [0.1 * math.cos(t), -0.2 * math.sin(t)]
        js.effort = [0.0, 0.0]
        self.joint_pub.publish(js)

        # TF: publish odom -> base_link and base_link -> tof_link
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'odom'
        t1.child_frame_id = 'base_link'
        t1.transform.translation.x = odom.pose.pose.position.x
        t1.transform.translation.y = odom.pose.pose.position.y
        t1.transform.translation.z = 0.0
        t1.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t1)

        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'tof_link'
        t2.transform.translation.x = 0.2
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.1
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t2)
        # IMU: publish orientation that slowly rotates in Z
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'imu_link'
        q_imu = tf_transformations.quaternion_from_euler(0.0, 0.0, t * 0.7)
        imu.orientation = Quaternion(x=q_imu[0], y=q_imu[1], z=q_imu[2], w=q_imu[3])
        # angular velocity and linear acceleration are examples
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = 0.7
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu)

        # Broadcast imu_link as child of base_link so RViz can visualize relative pose
        t3 = TransformStamped()
        t3.header.stamp = now
        t3.header.frame_id = 'base_link'
        t3.child_frame_id = 'imu_link'
        t3.transform.translation.x = 0.0
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 0.2
        t3.transform.rotation.x = q_imu[0]
        t3.transform.rotation.y = q_imu[1]
        t3.transform.rotation.z = q_imu[2]
        t3.transform.rotation.w = q_imu[3]
        self.tf_broadcaster.sendTransform(t3)


def launch_rviz():
    wrapper = os.path.expanduser('~/.local/bin/rviz2-no-snap')
    if os.path.isfile(wrapper) and os.access(wrapper, os.X_OK):
        try:
            # Launch rviz in a subprocess detached from this process
            subprocess.Popen([wrapper], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            print('Failed to launch rviz via wrapper:', e)
    else:
        print('rviz wrapper not found or not executable; please run rviz manually')


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-rviz', action='store_true', help='Do not auto-launch RViz')
    args = parser.parse_args(argv)

    rclpy.init()
    node = DemoNode()

    if not args.no_rviz:
        # Launch rviz in background
        threading.Thread(target=launch_rviz, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
