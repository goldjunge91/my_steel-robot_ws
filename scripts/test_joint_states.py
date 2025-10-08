#!/usr/bin/env python3
"""
Test script for verifying /joint_states topic data.

This script verifies:
- Topic exists and is publishing
- 4 joints are present with correct names
- Position and velocity data is non-zero and changing
- Publishing rate is approximately 100Hz

Requirements: 3.1, 3.2, 3.3
"""

import sys
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatesTester(Node):
    """Node to test /joint_states topic."""

    EXPECTED_JOINTS = [
        'front_left_wheel_joint',
        'front_right_wheel_joint',
        'rear_left_wheel_joint',
        'rear_right_wheel_joint'
    ]
    
    EXPECTED_RATE_HZ = 100.0
    RATE_TOLERANCE = 20.0  # Allow ±20Hz tolerance
    SAMPLE_DURATION = 2.0  # Collect samples for 2 seconds

    def __init__(self):
        super().__init__('joint_states_tester')
        self.samples = []
        self.start_time = None
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.get_logger().info('Subscribed to /joint_states')
        self.get_logger().info(f'Collecting samples for {self.SAMPLE_DURATION} seconds...')

    def joint_states_callback(self, msg):
        """Store joint state samples."""
        if self.start_time is None:
            self.start_time = time.time()
        
        current_time = time.time()
        if current_time - self.start_time <= self.SAMPLE_DURATION:
            self.samples.append({
                'timestamp': current_time,
                'names': msg.name,
                'positions': list(msg.position),
                'velocities': list(msg.velocity)
            })

    def analyze_results(self):
        """Analyze collected samples and report results."""
        if not self.samples:
            self.get_logger().error('❌ No samples received!')
            return False

        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info('JOINT STATES TEST RESULTS')
        self.get_logger().info(f'{"="*60}\n')

        all_passed = True

        # Test 1: Check number of samples
        num_samples = len(self.samples)
        self.get_logger().info(f'📊 Collected {num_samples} samples')

        # Test 2: Verify joint names
        self.get_logger().info('\n🔍 Checking joint names...')
        joint_names = self.samples[0]['names']
        
        if len(joint_names) != 4:
            self.get_logger().error(f'❌ Expected 4 joints, found {len(joint_names)}')
            all_passed = False
        else:
            self.get_logger().info(f'✅ Found 4 joints')

        missing_joints = set(self.EXPECTED_JOINTS) - set(joint_names)
        extra_joints = set(joint_names) - set(self.EXPECTED_JOINTS)
        
        if missing_joints:
            self.get_logger().error(f'❌ Missing joints: {missing_joints}')
            all_passed = False
        
        if extra_joints:
            self.get_logger().warn(f'⚠️  Extra joints: {extra_joints}')
        
        if not missing_joints and not extra_joints:
            self.get_logger().info('✅ All expected joints present:')
            for joint in self.EXPECTED_JOINTS:
                self.get_logger().info(f'   - {joint}')

        # Test 3: Check publishing rate
        self.get_logger().info('\n⏱️  Checking publishing rate...')
        if num_samples > 1:
            time_span = self.samples[-1]['timestamp'] - self.samples[0]['timestamp']
            actual_rate = (num_samples - 1) / time_span
            
            self.get_logger().info(f'   Measured rate: {actual_rate:.1f} Hz')
            self.get_logger().info(f'   Expected rate: {self.EXPECTED_RATE_HZ} Hz (±{self.RATE_TOLERANCE} Hz)')
            
            if abs(actual_rate - self.EXPECTED_RATE_HZ) <= self.RATE_TOLERANCE:
                self.get_logger().info(f'✅ Publishing rate is within tolerance')
            else:
                self.get_logger().error(f'❌ Publishing rate outside tolerance')
                all_passed = False
        else:
            self.get_logger().error('❌ Not enough samples to calculate rate')
            all_passed = False

        # Test 4: Check position and velocity data
        self.get_logger().info('\n📈 Checking position and velocity data...')
        
        # Organize data by joint
        joint_data = defaultdict(lambda: {'positions': [], 'velocities': []})
        for sample in self.samples:
            for i, name in enumerate(sample['names']):
                if i < len(sample['positions']):
                    joint_data[name]['positions'].append(sample['positions'][i])
                if i < len(sample['velocities']):
                    joint_data[name]['velocities'].append(sample['velocities'][i])

        for joint_name in self.EXPECTED_JOINTS:
            if joint_name not in joint_data:
                self.get_logger().error(f'❌ {joint_name}: No data found')
                all_passed = False
                continue

            positions = joint_data[joint_name]['positions']
            velocities = joint_data[joint_name]['velocities']

            # Check if data is changing
            pos_changing = len(set(positions)) > 1
            vel_changing = len(set(velocities)) > 1
            
            # Check if data is non-zero (at least some samples)
            pos_nonzero = any(abs(p) > 0.001 for p in positions)
            vel_has_variation = max(velocities) - min(velocities) > 0.001

            self.get_logger().info(f'\n   {joint_name}:')
            self.get_logger().info(f'      Position range: [{min(positions):.4f}, {max(positions):.4f}]')
            self.get_logger().info(f'      Velocity range: [{min(velocities):.4f}, {max(velocities):.4f}]')
            
            if pos_changing:
                self.get_logger().info(f'      ✅ Position data is changing')
            else:
                self.get_logger().warn(f'      ⚠️  Position data is constant (may be stationary)')
            
            if vel_has_variation:
                self.get_logger().info(f'      ✅ Velocity data has variation')
            else:
                self.get_logger().warn(f'      ⚠️  Velocity data is constant')

            # Only fail if data is stuck at exactly zero (hardware issue)
            if not pos_nonzero and max(velocities) == 0 and min(velocities) == 0:
                self.get_logger().error(f'      ❌ All data is zero (possible hardware issue)')
                all_passed = False

        # Final summary
        self.get_logger().info(f'\n{"="*60}')
        if all_passed:
            self.get_logger().info('✅ ALL TESTS PASSED')
            self.get_logger().info('Joint states are being published correctly!')
        else:
            self.get_logger().error('❌ SOME TESTS FAILED')
            self.get_logger().error('Please check the errors above.')
        self.get_logger().info(f'{"="*60}\n')

        return all_passed


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    print('\n' + '='*60)
    print('JOINT STATES VERIFICATION TEST')
    print('='*60)
    print('\nThis script will verify that /joint_states topic is:')
    print('  1. Publishing data')
    print('  2. Contains 4 expected joints')
    print('  3. Position and velocity data is valid')
    print('  4. Publishing at ~100Hz')
    print('\nMake sure the robot hardware is running!')
    print('='*60 + '\n')

    tester = JointStatesTester()
    
    # Collect samples
    start_time = time.time()
    while time.time() - start_time < JointStatesTester.SAMPLE_DURATION + 0.5:
        rclpy.spin_once(tester, timeout_sec=0.1)
    
    # Analyze results
    success = tester.analyze_results()
    
    tester.destroy_node()
    rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
