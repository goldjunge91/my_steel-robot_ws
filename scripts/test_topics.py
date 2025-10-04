# #!/usr/bin/env python3
# """
# Topic Testing Script for my_steel Robot
# Easily extensible script to test all available ROS2 topics
# """

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# import time
# import sys
# from typing import Dict, List, Callable, Any
# import json

# # Import message types
# #!/usr/bin/env python3
# """
# Enhanced Topic Testing Script for my_steel Robot

# This script discovers topics on the ROS2 network, subscribes to a configurable
# set of important topics, collects simple statistics (message count, sample
# payload snippets, message rate) and prints a summary report at the end.

# Usage: python3 scripts/test_topics.py [duration_seconds]

# The script is conservative: subscribers use a QoS profile suitable for
# interacting with micro-ROS bridges (RELIABLE by default) but this can be
# adjusted in the configuration below.
# """
# import sys
# import time
# import threading
# from typing import Dict, List, Any

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# # Import message types
# from std_msgs.msg import Int32
# from sensor_msgs.msg import Imu, JointState
# from geometry_msgs.msg import Twist
# from rcl_interfaces.msg import ParameterEvent, Log


# class TopicTester(Node):
#     """Extensible topic testing node that collects basic stats for topics."""

#     def __init__(self):
#         super().__init__('topic_tester')

#         # Default QoS profile (reliable, keep last, depth=10)
#         self.qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.RELIABLE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10,
#         )

#         # Topic configurations: add or update entries here to watch more topics
#         # Each entry describes the expected message type, a short description,
#         # an expected publish rate (Hz) used for health checks, and which fields
#         # to sample for the report.
#         self.topic_configs: Dict[str, Dict] = {
#             '/ddd/imu': {
#                 'msg_type': Imu,
#                 'description': 'IMU sensor data (acceleration, gyroscope)',
#                 'expected_rate': 50.0,
#                 'timeout': 5.0,
#                 'test_fields': ['linear_acceleration', 'angular_velocity', 'header'],
#             },
#             '/ddd_step2/cmd_vel': {
#                 'msg_type': Twist,
#                 'description': 'Motor velocity commands',
#                 'expected_rate': 10.0,
#                 'timeout': 3.0,
#                 'test_fields': ['linear', 'angular'],
#             },
#             '/joint_states': {
#                 'msg_type': JointState,
#                 'description': 'Joint state information',
#                 'expected_rate': 20.0,
#                 'timeout': 3.0,
#                 'test_fields': ['name', 'position', 'velocity'],
#             },
#             '/pico_count': {
#                 'msg_type': Int32,
#                 'description': 'Pico heartbeat counter',
#                 'expected_rate': 1.0,
#                 'timeout': 3.0,
#                 'test_fields': ['data'],
#             },
#             '/parameter_events': {
#                 'msg_type': ParameterEvent,
#                 'description': 'Parameter change events',
#                 'expected_rate': 0.1,
#                 'timeout': 10.0,
#                 'test_fields': ['node'],
#             },
#             '/rosout': {
#                 'msg_type': Log,
#                 'description': 'ROS logging messages',
#                 'expected_rate': 1.0,
#                 'timeout': 5.0,
#                 'test_fields': ['level', 'name', 'msg'],
#             },
#         }

#         # Runtime tracking structures
#         self.subscribers: Dict[str, Any] = {}
#         self.message_counts: Dict[str, int] = {}
#         self.last_message_time: Dict[str, float] = {}
#         self.first_message_time: Dict[str, float] = {}
#         self.sample_messages: Dict[str, Any] = {}
#         self.lock = threading.Lock()

#     def create_subscriber_for_topic(self, topic_name: str, config: Dict):
#         """Create a subscription for topic_name using the configured msg_type.

#         The callback captures the topic name and pushes message metadata into
#         the local statistics structures.
#         """

#         msg_type = config.get('msg_type')
#         if msg_type is None:
#             self.get_logger().warning(f'No msg_type for {topic_name}, skipping')
#             return

#         def _cb(msg, topic=topic_name):
#             now = time.time()
#             with self.lock:
#                 self.message_counts[topic] = self.message_counts.get(topic, 0) + 1
#                 self.last_message_time[topic] = now
#                 if topic not in self.first_message_time:
#                     self.first_message_time[topic] = now
#                 # store up to 3 sample messages (as repr) to include in report
#                 if topic not in self.sample_messages:
#                     self.sample_messages[topic] = []
#                 if len(self.sample_messages[topic]) < 3:
#                     try:
#                         self.sample_messages[topic].append(str(msg))
#                     except Exception:
#                         self.sample_messages[topic].append('<unserializable message>')

#         qos = config.get('qos', self.qos_profile)
#         sub = self.create_subscription(msg_type, topic_name, _cb, qos)
#         self.subscribers[topic_name] = sub
#         self.get_logger().info(f'🔗 Subscribed to {topic_name}')

#     def discover_and_test_topics(self):
#         """Discover topics and subscribe to those in the configuration.

#         If configured topics are not available, a warning is logged but the
#         script continues; this fits micro-ROS environments where topics can
#         appear later during startup.
#         """

#         # Discover currently available topics
#         topic_list = self.get_topic_names_and_types()
#         available = {name for (name, _) in topic_list}
#         self.get_logger().info(f'🔍 Found {len(available)} topics:')
#         for t in sorted(available):
#             self.get_logger().info(f'  - {t}')

#         # Subscribe to configured topics — even if not present yet we still
#         # attempt to create a subscription; ROS2 allows that.
#         for topic_name, cfg in self.topic_configs.items():
#             if topic_name in available:
#                 self.create_subscriber_for_topic(topic_name, cfg)
#             else:
#                 # still create the subscription — ROS2 will connect when the
#                 # publisher appears; log a warning for visibility
#                 self.get_logger().warning(f'⚠️  Topic {topic_name} not available — subscribing anyway')
#                 self.create_subscriber_for_topic(topic_name, cfg)

#     def compute_rate(self, topic: str) -> float:
#         with self.lock:
#             count = self.message_counts.get(topic, 0)
#             start = self.first_message_time.get(topic)
#             last = self.last_message_time.get(topic)
#         if count == 0 or start is None:
#             return 0.0
#         # Use observed window (last - first) to compute approximate rate
#         elapsed = (last - start) if last and start else 0.0
#         if elapsed <= 0.0:
#             # very small interval — treat as instantaneous
#             return float(count)
#         return float(count) / elapsed

#     def print_status_report(self):
#         """Print a summary report for all configured topics."""
#         now = time.time()
#         print('\n' + '=' * 80)
#         print('📊 TOPIC TEST REPORT')
#         print('=' * 80)

#         for topic_name, cfg in self.topic_configs.items():
#             desc = cfg.get('description', '')
#             expected = cfg.get('expected_rate', None)
#             timeout = cfg.get('timeout', 5.0)
#             with self.lock:
#                 count = self.message_counts.get(topic_name, 0)
#                 last = self.last_message_time.get(topic_name, 0)
#                 samples = self.sample_messages.get(topic_name, [])
#             rate = self.compute_rate(topic_name)
#             status = '❓ Not tested'
#             if count == 0:
#                 status = '🚫 NOT_AVAILABLE' if last == 0 else '⏰ TIMEOUT'
#             else:
#                 # If we have messages, mark checked — but evaluate expected rate
#                 status = '✅ OK'
#                 if expected is not None and rate < expected * 0.5:
#                     status = '⚠️ LOW_RATE'

#             print(f"\n🔸 {topic_name}\n   Description: {desc}\n   Status: {status}")
#             print(f"   Messages: {count} (Observed Rate: {rate:.2f} Hz, Expected: {expected} Hz)")
#             if samples:
#                 print('   Sample messages:')
#                 for s in samples:
#                     # Print only first 200 chars per sample
#                     print('    ', s[:200])

#         print('\nTest complete.\n')

#     def run_test(self, duration: float = 30.0):
#         """Run the test for duration seconds and periodically print progress."""
#         # Discover and subscribe
#         self.discover_and_test_topics()

#         start_time = time.time()
#         next_report = start_time + 5.0
#         self.get_logger().info(f'🚀 Starting topic test for {duration}s...')

#         while True:
#             now = time.time()
#             elapsed = now - start_time
#             if now >= next_report:
#                 # Print an interim small status
#                 self.get_logger().info(f'⏱ Runtime: {elapsed:.1f}s')
#                 next_report = now + 5.0
#             if elapsed >= duration:
#                 break
#             rclpy.spin_once(self, timeout_sec=0.1)

#         # Final report
#         self.print_status_report()


# def main():
#     rclpy.init()
#     duration = 30.0
#     if len(sys.argv) > 1:
#         try:
#             duration = float(sys.argv[1])
#         except Exception:
#             print('Invalid duration argument, using default 30s')

#     tester = TopicTester()
#     try:
#         tester.run_test(duration)
#     except KeyboardInterrupt:
#         print('Interrupted by user')
#     finally:
#         tester.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
#     def print_status_report(self):
#         """Print comprehensive status report"""
#         current_time = time.time()
#         runtime = current_time - self.start_time
        
#         print("\n" + "="*80)
#         print(f"📊 TOPIC TEST REPORT - Runtime: {runtime:.1f}s")
#         print("="*80)
        
#         for topic_name, config in self.topic_configs.items():
#             print(f"\n🔸 {topic_name}")
#             print(f"   Description: {config['description']}")
            
#             if topic_name not in self.test_results:
#                 print("   Status: ❓ Not tested")
#                 continue
                
#             result = self.test_results[topic_name]
#             status = result['status']
            
#             # Status emoji
#             status_emoji = {
#                 'healthy': '✅',
#                 'receiving': '📨',
#                 'timeout': '⏰',
#                 'no_messages': '❌',
#                 'not_available': '🚫'
#             }
            
#             print(f"   Status: {status_emoji.get(status, '❓')} {status.upper()}")
            
#             if topic_name in self.message_counts:
#                 count = self.message_counts[topic_name]
#                 rate = count / runtime if runtime > 0 else 0
#                 expected_rate = config['expected_rate']
                
#                 print(f"   Messages: {count} (Rate: {rate:.1f} Hz, Expected: {expected_rate} Hz)")
                
#                 # Show latest message data
#                 if result.get('messages'):
#                     latest = result['messages'][-1]
#                     print(f"   Latest Data: {json.dumps(latest['data'], indent=15)}")
            
#             print(f"   Expected Rate: {config['expected_rate']} Hz")
#             print(f"   Timeout: {config['timeout']}s")
    
#     def run_test(self, duration: float = 30.0):
#         """Run the topic test for specified duration"""
#         self.get_logger().info(f"🚀 Starting topic test for {duration}s...")
        
#         # Discover and subscribe to topics
#         self.discover_and_test_topics()
        
#         # Test loop
#         start_time = time.time()
#         last_report_time = start_time
        
#         while (time.time() - start_time) < duration:
#             rclpy.spin_once(self, timeout_sec=1.0)
            
#             current_time = time.time()
            
#             # Check health every 5 seconds
#             if current_time - last_report_time >= 5.0:
#                 self.check_topic_health()
#                 self.print_status_report()
#                 last_report_time = current_time
        
#         # Final report
#         self.get_logger().info("🏁 Test completed!")
#         self.print_status_report()


# def main():
#     """Main function"""
#     rclpy.init()
    
#     # Parse command line arguments
#     duration = 30.0  # Default test duration
#     if len(sys.argv) > 1:
#         try:
#             duration = float(sys.argv[1])
#         except ValueError:
#             print("Usage: python3 test_topics.py [duration_in_seconds]")
#             sys.exit(1)
    
#     try:
#         tester = TopicTester()
#         tester.run_test(duration)
#     except KeyboardInterrupt:
#         print("\n🛑 Test interrupted by user")
#     finally:
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
Enhanced Topic Testing Script for my_steel Robot

This script discovers topics on the ROS2 network, subscribes to a configurable
set of important topics, collects simple statistics (message count, sample
payload snippets, message rate) and prints a summary report at the end.

Usage: python3 scripts/test_topics.py [duration_seconds]

The script is conservative: subscribers use a QoS profile suitable for
interacting with micro-ROS bridges (RELIABLE by default) but this can be
adjusted in the configuration below.
"""
import sys
import time
import threading
from typing import Dict, List, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Import message types
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu, JointState, Range, Illuminance
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterEvent, Log


class TopicTester(Node):
    """Extensible topic testing node that collects basic stats for topics."""

    def __init__(self):
        super().__init__('topic_tester')

        # Default QoS profile (reliable, keep last, depth=10)
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Topic configurations: add or update entries here to watch more topics
        self.topic_configs: Dict[str, Dict] = {
            '/ddd/imu': {
                'msg_type': Imu,
                'description': 'IMU sensor data (acceleration, gyroscope)',
                'expected_rate': 5.0, # Angepasst an die Firmware (delay(200))
                'timeout': 5.0,
                'test_fields': ['linear_acceleration', 'angular_velocity', 'header'],
            },
            '/ddd/range_tof': {
                'msg_type': Range,
                'description': 'Time-of-Flight distance sensor data',
                'expected_rate': 5.0, # Angepasst an die Firmware (delay(200))
                'timeout': 3.0,
                'test_fields': ['range', 'header'],
            },
            '/ddd/illuminance': {
                'msg_type': Illuminance,
                'description': 'Ambient light sensor data',
                'expected_rate': 5.0, # Angepasst an die Firmware (delay(200))
                'timeout': 3.0,
                'test_fields': ['illuminance', 'header'],
            },
            '/ddd/cmd_vel': {
                'msg_type': Twist,
                'description': 'Motor velocity commands',
                'expected_rate': 10.0,
                'timeout': 3.0,
                'test_fields': ['linear', 'angular'],
            },
            '/joint_states': {
                'msg_type': JointState,
                'description': 'Joint state information',
                'expected_rate': 5.0, # Angepasst an die Firmware (delay(200))
                'timeout': 3.0,
                'test_fields': ['name', 'position', 'velocity'],
            },
            '/pico_count': {
                'msg_type': Int32,
                'description': 'Pico heartbeat counter',
                'expected_rate': 1.0,
                'timeout': 3.0,
                'test_fields': ['data'],
            },
            '/parameter_events': {
                'msg_type': ParameterEvent,
                'description': 'Parameter change events',
                'expected_rate': 0.1,
                'timeout': 10.0,
                'test_fields': ['node'],
            },
            '/rosout': {
                'msg_type': Log,
                'description': 'ROS logging messages',
                'expected_rate': 1.0,
                'timeout': 5.0,
                'test_fields': ['level', 'name', 'msg'],
            },
        }

        # Runtime tracking structures
        self.subscribers: Dict[str, Any] = {}
        self.message_counts: Dict[str, int] = {}
        self.last_message_time: Dict[str, float] = {}
        self.first_message_time: Dict[str, float] = {}
        self.sample_messages: Dict[str, Any] = {}
        self.lock = threading.Lock()

    def create_subscriber_for_topic(self, topic_name: str, config: Dict):
        """Create a subscription for topic_name using the configured msg_type."""
        msg_type = config.get('msg_type')
        if msg_type is None:
            self.get_logger().warning(f'No msg_type for {topic_name}, skipping')
            return

        def _cb(msg, topic=topic_name):
            now = time.time()
            with self.lock:
                self.message_counts[topic] = self.message_counts.get(topic, 0) + 1
                self.last_message_time[topic] = now
                if topic not in self.first_message_time:
                    self.first_message_time[topic] = now
                if topic not in self.sample_messages:
                    self.sample_messages[topic] = []
                if len(self.sample_messages[topic]) < 3:
                    try:
                        self.sample_messages[topic].append(str(msg))
                    except Exception:
                        self.sample_messages[topic].append('<unserializable message>')

        qos = config.get('qos', self.qos_profile)
        sub = self.create_subscription(msg_type, topic_name, _cb, qos)
        self.subscribers[topic_name] = sub
        self.get_logger().info(f'🔗 Subscribed to {topic_name}')

    def discover_and_test_topics(self):
        """Discover topics and subscribe to those in the configuration."""
        topic_list = self.get_topic_names_and_types()
        available = {name for (name, _) in topic_list}
        self.get_logger().info(f'🔍 Found {len(available)} topics:')
        for t in sorted(available):
            self.get_logger().info(f'  - {t}')

        for topic_name, cfg in self.topic_configs.items():
            if topic_name not in available:
                self.get_logger().warning(f'⚠️  Topic {topic_name} not available — subscribing anyway')
            self.create_subscriber_for_topic(topic_name, cfg)

    def compute_rate(self, topic: str) -> float:
        with self.lock:
            count = self.message_counts.get(topic, 0)
            start = self.first_message_time.get(topic)
            last = self.last_message_time.get(topic)
        if count <= 1 or start is None or last is None:
            return 0.0
        elapsed = last - start
        if elapsed <= 0.0:
            return float(count)
        return float(count) / elapsed

    def print_status_report(self):
        """Print a summary report for all configured topics."""
        now = time.time()
        print('\n' + '=' * 80)
        print('📊 TOPIC TEST REPORT')
        print('=' * 80)

        for topic_name, cfg in self.topic_configs.items():
            desc = cfg.get('description', '')
            expected = cfg.get('expected_rate', None)
            timeout = cfg.get('timeout', 5.0)
            with self.lock:
                count = self.message_counts.get(topic_name, 0)
                last = self.last_message_time.get(topic_name, 0)
                samples = self.sample_messages.get(topic_name, [])
            rate = self.compute_rate(topic_name)
            
            status = '❓ Not tested'
            if count == 0:
                status = '🚫 NOT_AVAILABLE'
            else:
                if (now - last) > timeout:
                    status = f'⏰ TIMEOUT (no msg in {now-last:.1f}s)'
                else:
                    status = '✅ OK'
                    if expected is not None and rate < expected * 0.7:
                        status = '⚠️ LOW_RATE'

            print(f"\n🔸 {topic_name}\n   Description: {desc}\n   Status: {status}")
            print(f"   Messages: {count} (Observed Rate: {rate:.2f} Hz, Expected: {expected} Hz)")
            if samples:
                print('   Sample messages:')
                for s in samples:
                    print('    ', s[:250] + ('...' if len(s) > 250 else ''))

        print('\nTest complete.\n')

    def run_test(self, duration: float = 30.0):
        """Run the test for duration seconds and periodically print progress."""
        self.discover_and_test_topics()

        start_time = time.time()
        next_report = start_time + 5.0
        self.get_logger().info(f'🚀 Starting topic test for {duration}s...')

        while rclpy.ok():
            now = time.time()
            elapsed = now - start_time
            if now >= next_report:
                self.get_logger().info(f'⏱ Runtime: {elapsed:.1f}s')
                next_report = now + 5.0
            if elapsed >= duration:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        self.print_status_report()


def main():
    rclpy.init()
    duration = 30.0
    if len(sys.argv) > 1:
        try:
            duration = float(sys.argv[1])
        except Exception:
            print('Invalid duration argument, using default 30s')

    tester = TopicTester()
    try:
        tester.run_test(duration)
    except KeyboardInterrupt:
        print('Interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
