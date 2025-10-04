#!/usr/bin/env python3
"""
Analyze IMU noise characteristics from rosbag or live topic
to estimate realistic covariance values.

Usage:
    # Live analysis (robot must be stationary!)
    ros2 run <package> analyze_imu_noise.py
    
    # Or simply subscribe to topic
    ros2 topic echo /ddd/imu > imu_data.txt
    # Then process offline
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import sys


class ImuNoiseAnalyzer(Node):
    def __init__(self):
        super().__init__('imu_noise_analyzer')
        
        self.subscription = self.create_subscription(
            Imu,
            '/ddd/imu',
            self.imu_callback,
            10)
        
        # Storage for samples (collect 1000 samples = ~100 seconds at 10Hz)
        self.accel_samples = []
        self.gyro_samples = []
        self.max_samples = 1000
        
        # Timeout detection
        self.last_msg_time = self.get_clock().now()
        self.timeout_warned = False
        
        # Timer to check for incoming messages
        self.timer = self.create_timer(5.0, self.check_timeout)
        
        self.get_logger().info('IMU Noise Analyzer started')
        self.get_logger().info('Keep robot STATIONARY for accurate measurements!')
        self.get_logger().info(f'Collecting {self.max_samples} samples...')
        self.get_logger().info('Waiting for IMU messages on /ddd/imu...')
    
    def check_timeout(self):
        """Check if we're receiving IMU messages"""
        if len(self.accel_samples) == 0 and not self.timeout_warned:
            self.timeout_warned = True
            self.get_logger().error('⚠️  No IMU messages received yet!')
            self.get_logger().error('Troubleshooting steps:')
            self.get_logger().error('  1. Check if micro-ROS agent is running: ps aux | grep micro_ros_agent')
            self.get_logger().error('  2. Check firmware UART output: picocom -b 115200 /dev/ttyUSB0')
            self.get_logger().error('  3. Verify IMU sensor initialized: Look for "[ImuAgent] ✅ Sensor initialized"')
            self.get_logger().error('  4. Check ROS connection: ros2 topic list (should show /ddd/imu)')
            self.get_logger().error('  5. Test topic: ros2 topic echo /ddd/imu')
    
    def imu_callback(self, msg):
        if len(self.accel_samples) >= self.max_samples:
            return
        
        # Update last message time
        self.last_msg_time = self.get_clock().now()
        
        # Store samples
        self.accel_samples.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        self.gyro_samples.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Progress indicator
        if len(self.accel_samples) == 1:
            self.get_logger().info('✅ First message received! Starting collection...')
        
        if len(self.accel_samples) % 100 == 0:
            self.get_logger().info(f'Collected {len(self.accel_samples)}/{self.max_samples} samples')
        
        # Analyze when done
        if len(self.accel_samples) == self.max_samples:
            self.analyze_and_report()
            sys.exit(0)
    
    def analyze_and_report(self):
        accel = np.array(self.accel_samples)
        gyro = np.array(self.gyro_samples)
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('IMU NOISE ANALYSIS RESULTS')
        self.get_logger().info('='*60)
        
        # Accelerometer analysis
        accel_mean = np.mean(accel, axis=0)
        accel_std = np.std(accel, axis=0)
        accel_var = np.var(accel, axis=0)
        
        self.get_logger().info('\n--- ACCELEROMETER (m/s²) ---')
        self.get_logger().info(f'Mean:     X={accel_mean[0]:.4f}, Y={accel_mean[1]:.4f}, Z={accel_mean[2]:.4f}')
        self.get_logger().info(f'Std Dev:  X={accel_std[0]:.4f}, Y={accel_std[1]:.4f}, Z={accel_std[2]:.4f}')
        self.get_logger().info(f'Variance: X={accel_var[0]:.6f}, Y={accel_var[1]:.6f}, Z={accel_var[2]:.6f}')
        
        # Check if robot was stationary (Z should be ~9.81, X/Y ~0)
        gravity_norm = np.linalg.norm(accel_mean)
        self.get_logger().info(f'Gravity norm: {gravity_norm:.4f} m/s² (expected ~9.81)')
        
        if abs(gravity_norm - 9.81) > 0.5:
            self.get_logger().warn('Robot may not have been stationary during measurement!')
        
        # Gyroscope analysis
        gyro_mean = np.mean(gyro, axis=0)
        gyro_std = np.std(gyro, axis=0)
        gyro_var = np.var(gyro, axis=0)
        
        self.get_logger().info('\n--- GYROSCOPE (rad/s) ---')
        self.get_logger().info(f'Mean:     X={gyro_mean[0]:.6f}, Y={gyro_mean[1]:.6f}, Z={gyro_mean[2]:.6f}')
        self.get_logger().info(f'Std Dev:  X={gyro_std[0]:.6f}, Y={gyro_std[1]:.6f}, Z={gyro_std[2]:.6f}')
        self.get_logger().info(f'Variance: X={gyro_var[0]:.8f}, Y={gyro_var[1]:.8f}, Z={gyro_var[2]:.8f}')
        
        # Check for bias drift
        if np.max(np.abs(gyro_mean)) > 0.05:
            self.get_logger().warn('Significant gyro bias detected - consider recalibration!')
        
        # Generate C++ code for covariance initialization
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('RECOMMENDED COVARIANCE VALUES FOR FIRMWARE:')
        self.get_logger().info('='*60)
        
        # Use 3-sigma (99.7% confidence) for diagonal covariances
        accel_cov = accel_var * 9  # 3-sigma squared
        gyro_cov = gyro_var * 9
        
        self.get_logger().info('\nAdd to ImuAgent constructor:')
        self.get_logger().info(f'''
  // Linear acceleration covariance (m/s²)²
  imu_msg_.linear_acceleration_covariance[0] = {accel_cov[0]:.6f};  // X
  imu_msg_.linear_acceleration_covariance[4] = {accel_cov[1]:.6f};  // Y
  imu_msg_.linear_acceleration_covariance[8] = {accel_cov[2]:.6f};  // Z
  
  // Angular velocity covariance (rad/s)²
  imu_msg_.angular_velocity_covariance[0] = {gyro_cov[0]:.6f};  // X
  imu_msg_.angular_velocity_covariance[4] = {gyro_cov[1]:.6f};  // Y
  imu_msg_.angular_velocity_covariance[8] = {gyro_cov[2]:.6f};  // Z
''')
        
        self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    analyzer = ImuNoiseAnalyzer()
    
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
