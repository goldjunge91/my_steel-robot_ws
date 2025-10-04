#!/usr/bin/env python3
"""
Topic Testing Script for my_steel Robot
Easily extensible script to test all available ROS2 topics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import sys
from typing import Dict, List, Callable, Any
import json

# Import message types
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterEvent, Log


class TopicTester(Node):
    """Extensible topic testing node"""
    
    def __init__(self):
        super().__init__('topic_tester')
        
        # QoS profile for reliable communication
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Topic configurations - easily extensible
        self.topic_configs = {
            '/ddd/imu': {
                'msg_type': Imu,
                'description': 'IMU sensor data (acceleration, gyroscope)',
                'expected_rate': 50.0,  # Hz
                'timeout': 5.0,  # seconds
                'test_fields': ['linear_acceleration', 'angular_velocity', 'header']
            },
            '/ddd_step2/cmd_vel': {
                'msg_type': Twist,
                'description': 'Motor velocity commands',
                'expected_rate': 10.0,
                'timeout': 3.0,
                'test_fields': ['linear', 'angular']
            },
            '/joint_states': {
                'msg_type': JointState,
                'description': 'Joint state information',
                'expected_rate': 20.0,
                'timeout': 3.0,
                'test_fields': ['name', 'position', 'velocity']
            },
            '/pico_count': {
                'msg_type': Int32,
                'description': 'Pico heartbeat counter',
                'expected_rate': 1.0,
                'timeout': 3.0,
                'test_fields': ['data']
            },
            '/parameter_events': {
                'msg_type': ParameterEvent,
                'description': 'Parameter change events',
                'expected_rate': 0.1,  # Low frequency
                'timeout': 10.0,
                'test_fields': ['node']
            },
            '/rosout': {
                'msg_type': Log,
                'description': 'ROS logging messages',
                'expected_rate': 1.0,
                'timeout': 5.0,
                'test_fields': ['level', 'name', 'msg']
            }
        }
        
        # Test results storage
        self.test_results = {}
        self.message_counts = {}
        self.last_message_time = {}
        self.subscribers = {}
        
        # Test statistics
        self.start_time = time.time()
        self.last_log_time = {}  # Track last log time per topic
        
    def create_subscriber_for_topic(self, topic_name: str, config: Dict):
        """Create a subscriber for a specific topic"""
        def callback(msg):
            current_time = time.time()
            
            # Update statistics
            if topic_name not in self.message_counts:
                self.message_counts[topic_name] = 0
                self.last_log_time = {}
            
            self.message_counts[topic_name] += 1
            self.last_message_time[topic_name] = current_time
            
            # Store latest message for analysis
            if topic_name not in self.test_results:
                self.test_results[topic_name] = {
                    'status': 'receiving',
                    'messages': [],
                    'first_message_time': current_time,
                    'config': config
                }
                self.last_log_time[topic_name] = 0
            
            # Keep only last few messages to avoid memory issues
            if len(self.test_results[topic_name]['messages']) >= 5:
                self.test_results[topic_name]['messages'].pop(0)
            
            self.test_results[topic_name]['messages'].append({
                'timestamp': current_time,
                'data': self.extract_message_data(msg, config['test_fields'])
            })
            
            # Limit logging output - only log every 5 seconds or first 15 messages
            should_log = False
            count = self.message_counts[topic_name]
            
            if count <= 15:  # First 15 messages
                should_log = True
            elif current_time - self.last_log_time.get(topic_name, 0) >= 5.0:  # Every 5 seconds after that
                should_log = True
                self.last_log_time[topic_name] = current_time
            
            if should_log:
                self.get_logger().info(f"📨 {topic_name}: Message #{count}")
        
        # Create subscriber
        subscriber = self.create_subscription(
            config['msg_type'],
            topic_name,
            callback,
            self.qos_profile
        )
        
        self.subscribers[topic_name] = subscriber
        self.get_logger().info(f"🔗 Subscribed to {topic_name}")
        
    def extract_message_data(self, msg: Any, test_fields: List[str]) -> Dict:
        """Extract relevant data from message for testing"""
        data = {}
        for field in test_fields:
            try:
                if hasattr(msg, field):
                    value = getattr(msg, field)
                    # Convert complex types to string representation
                    if hasattr(value, '__dict__'):
                        data[field] = str(value)
                    else:
                        data[field] = value
                else:
                    data[field] = f"Field '{field}' not found"
            except Exception as e:
                data[field] = f"Error: {str(e)}"
        return data
    
    def discover_and_test_topics(self):
        """Discover available topics and start testing"""
        self.get_logger().info("🔍 Discovering available topics...")
        
        # Get list of available topics
        topic_names_and_types = self.get_topic_names_and_types()
        available_topics = [name for name, _ in topic_names_and_types]
        
        self.get_logger().info(f"📋 Found {len(available_topics)} topics:")
        for topic in available_topics:
            self.get_logger().info(f"  - {topic}")
        
        # Subscribe to configured topics that are available
        subscribed_count = 0
        for topic_name, config in self.topic_configs.items():
            if topic_name in available_topics:
                self.create_subscriber_for_topic(topic_name, config)
                subscribed_count += 1
            else:
                self.get_logger().warn(f"⚠️  Topic {topic_name} not available")
                self.test_results[topic_name] = {
                    'status': 'not_available',
                    'config': config
                }
        
        self.get_logger().info(f"✅ Subscribed to {subscribed_count} topics")
        
    def check_topic_health(self):
        """Check health of all subscribed topics"""
        current_time = time.time()
        
        for topic_name, config in self.topic_configs.items():
            if topic_name not in self.test_results:
                continue
                
            result = self.test_results[topic_name]
            
            if result['status'] == 'not_available':
                continue
                
            # Check if we're receiving messages within timeout
            if topic_name in self.last_message_time:
                time_since_last = current_time - self.last_message_time[topic_name]
                if time_since_last > config['timeout']:
                    result['status'] = 'timeout'
                    self.get_logger().warn(f"⏰ {topic_name}: No messages for {time_since_last:.1f}s")
                else:
                    result['status'] = 'healthy'
            else:
                # Never received a message
                time_since_start = current_time - self.start_time
                if time_since_start > config['timeout']:
                    result['status'] = 'no_messages'
                    self.get_logger().warn(f"❌ {topic_name}: No messages received")
    
    def print_status_report(self):
        """Print comprehensive status report"""
        current_time = time.time()
        runtime = current_time - self.start_time
        
        print("\n" + "="*80)
        print(f"📊 TOPIC TEST REPORT - Runtime: {runtime:.1f}s")
        print("="*80)
        
        for topic_name, config in self.topic_configs.items():
            print(f"\n🔸 {topic_name}")
            print(f"   Description: {config['description']}")
            
            if topic_name not in self.test_results:
                print("   Status: ❓ Not tested")
                continue
                
            result = self.test_results[topic_name]
            status = result['status']
            
            # Status emoji
            status_emoji = {
                'healthy': '✅',
                'receiving': '📨',
                'timeout': '⏰',
                'no_messages': '❌',
                'not_available': '🚫'
            }
            
            print(f"   Status: {status_emoji.get(status, '❓')} {status.upper()}")
            
            if topic_name in self.message_counts:
                count = self.message_counts[topic_name]
                rate = count / runtime if runtime > 0 else 0
                expected_rate = config['expected_rate']
                
                print(f"   Messages: {count} (Rate: {rate:.1f} Hz, Expected: {expected_rate} Hz)")
                
                # Show latest message data
                if result.get('messages'):
                    latest = result['messages'][-1]
                    print(f"   Latest Data: {json.dumps(latest['data'], indent=15)}")
            
            print(f"   Expected Rate: {config['expected_rate']} Hz")
            print(f"   Timeout: {config['timeout']}s")
    
    def run_test(self, duration: float = 30.0):
        """Run the topic test for specified duration"""
        self.get_logger().info(f"🚀 Starting topic test for {duration}s...")
        
        # Discover and subscribe to topics
        self.discover_and_test_topics()
        
        # Test loop
        start_time = time.time()
        last_report_time = start_time
        
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=1.0)
            
            current_time = time.time()
            
            # Check health every 5 seconds
            if current_time - last_report_time >= 5.0:
                self.check_topic_health()
                self.print_status_report()
                last_report_time = current_time
        
        # Final report
        self.get_logger().info("🏁 Test completed!")
        self.print_status_report()


def main():
    """Main function"""
    rclpy.init()
    
    # Parse command line arguments
    duration = 30.0  # Default test duration
    if len(sys.argv) > 1:
        try:
            duration = float(sys.argv[1])
        except ValueError:
            print("Usage: python3 test_topics.py [duration_in_seconds]")
            sys.exit(1)
    
    try:
        tester = TopicTester()
        tester.run_test(duration)
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()