#!/usr/bin/env python3
"""
Test script to verify IMU data from /imu/data_raw topic.
Checks:
- Orientation, angular_velocity, and linear_acceleration are published
- Data is non-zero and realistic
- Publishing rate is approximately 50Hz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time
import sys
import math


class ImuDataTester(Node):
    def __init__(self):
        super().__init__('imu_data_tester')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10)
        
        self.samples = []
        self.start_time = None
        self.max_samples = 100
        self.test_duration = 5.0  # seconds
        
        self.get_logger().info('IMU Data Tester started. Collecting samples...')
    
    def imu_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
        
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed <= self.test_duration and len(self.samples) < self.max_samples:
            self.samples.append({
                'timestamp': current_time,
                'orientation': msg.orientation,
                'angular_velocity': msg.angular_velocity,
                'linear_acceleration': msg.linear_acceleration,
                'frame_id': msg.header.frame_id
            })
    
    def analyze_results(self):
        if len(self.samples) == 0:
            self.get_logger().error('❌ No IMU samples received!')
            return False
        
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info('IMU DATA TEST RESULTS')
        self.get_logger().info(f'{"="*60}')
        
        # Calculate publishing rate
        if len(self.samples) > 1:
            time_span = self.samples[-1]['timestamp'] - self.samples[0]['timestamp']
            rate = len(self.samples) / time_span if time_span > 0 else 0
            self.get_logger().info(f'📊 Samples collected: {len(self.samples)}')
            self.get_logger().info(f'⏱️  Time span: {time_span:.2f} seconds')
            self.get_logger().info(f'📈 Publishing rate: {rate:.2f} Hz')
            
            # Check if rate is reasonable (allow some tolerance)
            if rate < 5:
                self.get_logger().warn(f'⚠️  Publishing rate ({rate:.2f} Hz) is lower than expected (~50 Hz)')
            elif rate >= 40 and rate <= 60:
                self.get_logger().info(f'✅ Publishing rate is within expected range (40-60 Hz)')
            else:
                self.get_logger().info(f'ℹ️  Publishing rate: {rate:.2f} Hz (expected ~50 Hz)')
        
        # Check frame_id
        frame_id = self.samples[0]['frame_id']
        self.get_logger().info(f'🔗 Frame ID: {frame_id}')
        if frame_id == 'imu_link':
            self.get_logger().info('✅ Frame ID is correct')
        else:
            self.get_logger().warn(f'⚠️  Frame ID is "{frame_id}", expected "imu_link"')
        
        # Analyze orientation
        self.get_logger().info(f'\n{"─"*60}')
        self.get_logger().info('ORIENTATION DATA:')
        orient_sample = self.samples[len(self.samples)//2]['orientation']
        self.get_logger().info(f'  x: {orient_sample.x:.6f}')
        self.get_logger().info(f'  y: {orient_sample.y:.6f}')
        self.get_logger().info(f'  z: {orient_sample.z:.6f}')
        self.get_logger().info(f'  w: {orient_sample.w:.6f}')
        
        # Check if orientation is unit quaternion
        mag = math.sqrt(orient_sample.x**2 + orient_sample.y**2 + 
                       orient_sample.z**2 + orient_sample.w**2)
        if abs(mag - 1.0) < 0.01:
            self.get_logger().info(f'✅ Orientation is a valid unit quaternion (magnitude: {mag:.6f})')
        else:
            self.get_logger().warn(f'⚠️  Orientation magnitude: {mag:.6f} (expected ~1.0)')
        
        # Analyze angular velocity
        self.get_logger().info(f'\n{"─"*60}')
        self.get_logger().info('ANGULAR VELOCITY DATA:')
        ang_vel_x = [s['angular_velocity'].x for s in self.samples]
        ang_vel_y = [s['angular_velocity'].y for s in self.samples]
        ang_vel_z = [s['angular_velocity'].z for s in self.samples]
        
        self.get_logger().info(f'  x: min={min(ang_vel_x):.6f}, max={max(ang_vel_x):.6f}, avg={sum(ang_vel_x)/len(ang_vel_x):.6f}')
        self.get_logger().info(f'  y: min={min(ang_vel_y):.6f}, max={max(ang_vel_y):.6f}, avg={sum(ang_vel_y)/len(ang_vel_y):.6f}')
        self.get_logger().info(f'  z: min={min(ang_vel_z):.6f}, max={max(ang_vel_z):.6f}, avg={sum(ang_vel_z)/len(ang_vel_z):.6f}')
        
        # Check if angular velocity is changing (not all zeros)
        ang_vel_changing = (max(ang_vel_x) - min(ang_vel_x) > 0.001 or
                           max(ang_vel_y) - min(ang_vel_y) > 0.001 or
                           max(ang_vel_z) - min(ang_vel_z) > 0.001)
        
        if ang_vel_changing:
            self.get_logger().info('✅ Angular velocity data is changing (sensor is active)')
        else:
            self.get_logger().warn('⚠️  Angular velocity appears static')
        
        # Check if values are realistic (typical gyro range: ±250 deg/s = ±4.36 rad/s)
        max_ang_vel = max(abs(max(ang_vel_x)), abs(min(ang_vel_x)),
                         abs(max(ang_vel_y)), abs(min(ang_vel_y)),
                         abs(max(ang_vel_z)), abs(min(ang_vel_z)))
        
        if max_ang_vel < 10.0:  # Reasonable for stationary robot
            self.get_logger().info(f'✅ Angular velocity values are realistic (max: {max_ang_vel:.3f} rad/s)')
        else:
            self.get_logger().warn(f'⚠️  Angular velocity seems high (max: {max_ang_vel:.3f} rad/s)')
        
        # Analyze linear acceleration
        self.get_logger().info(f'\n{"─"*60}')
        self.get_logger().info('LINEAR ACCELERATION DATA:')
        lin_acc_x = [s['linear_acceleration'].x for s in self.samples]
        lin_acc_y = [s['linear_acceleration'].y for s in self.samples]
        lin_acc_z = [s['linear_acceleration'].z for s in self.samples]
        
        self.get_logger().info(f'  x: min={min(lin_acc_x):.3f}, max={max(lin_acc_x):.3f}, avg={sum(lin_acc_x)/len(lin_acc_x):.3f}')
        self.get_logger().info(f'  y: min={min(lin_acc_y):.3f}, max={max(lin_acc_y):.3f}, avg={sum(lin_acc_y)/len(lin_acc_y):.3f}')
        self.get_logger().info(f'  z: min={min(lin_acc_z):.3f}, max={max(lin_acc_z):.3f}, avg={sum(lin_acc_z)/len(lin_acc_z):.3f}')
        
        # Check if acceleration is non-zero
        avg_acc_magnitude = math.sqrt(
            (sum(lin_acc_x)/len(lin_acc_x))**2 +
            (sum(lin_acc_y)/len(lin_acc_y))**2 +
            (sum(lin_acc_z)/len(lin_acc_z))**2
        )
        
        self.get_logger().info(f'  Average magnitude: {avg_acc_magnitude:.3f} m/s²')
        
        # Should be close to gravity (9.81 m/s²) for stationary robot
        if 8.0 < avg_acc_magnitude < 11.0:
            self.get_logger().info(f'✅ Linear acceleration magnitude is realistic (~9.81 m/s² expected for gravity)')
        else:
            self.get_logger().warn(f'⚠️  Linear acceleration magnitude seems unusual (expected ~9.81 m/s²)')
        
        # Check if data is changing
        lin_acc_changing = (max(lin_acc_x) - min(lin_acc_x) > 0.01 or
                           max(lin_acc_y) - min(lin_acc_y) > 0.01 or
                           max(lin_acc_z) - min(lin_acc_z) > 0.01)
        
        if lin_acc_changing:
            self.get_logger().info('✅ Linear acceleration data is changing (sensor is active)')
        else:
            self.get_logger().warn('⚠️  Linear acceleration appears static')
        
        # Final summary
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info('SUMMARY:')
        
        all_checks_passed = True
        
        if len(self.samples) > 10:
            self.get_logger().info('✅ Sufficient samples collected')
        else:
            self.get_logger().error('❌ Insufficient samples collected')
            all_checks_passed = False
        
        if ang_vel_changing and lin_acc_changing:
            self.get_logger().info('✅ All sensor data fields are being published')
        else:
            self.get_logger().warn('⚠️  Some sensor data may not be updating')
            all_checks_passed = False
        
        if 8.0 < avg_acc_magnitude < 11.0:
            self.get_logger().info('✅ Data values are realistic')
        else:
            self.get_logger().warn('⚠️  Data values may be unrealistic')
        
        self.get_logger().info(f'{"="*60}\n')
        
        return all_checks_passed


def main(args=None):
    rclpy.init(args=args)
    
    tester = ImuDataTester()
    
    # Spin for test duration
    start = time.time()
    while rclpy.ok() and (time.time() - start) < tester.test_duration + 1.0:
        rclpy.spin_once(tester, timeout_sec=0.1)
    
    # Analyze results
    success = tester.analyze_results()
    
    tester.destroy_node()
    rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
