#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.serialization import serialize_message
import argparse
from datetime import datetime
import json
from typing import Dict, Type, Any
import sys
from colorama import init, Fore, Style

# Import all possible message types
from std_msgs.msg import String, Char, UInt8, UInt16, UInt32, UInt64, Int8, Int16, Int32, Int64, Float32, Float64, Bool
from sensor_msgs.msg import NavSatFix, BatteryState, Image, CompressedImage, PointCloud2, LaserScan, JointState, Joy
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Log
from visualization_msgs.msg import MarkerArray

# Initialize colorama for cross-platform colored output
init()

class UniversalSubscriber(Node):
    TOPIC_TYPES: Dict[str, Type] = {
        'stringpub': String,
        'charpub': Char,
        'uint8pub': UInt8,
        'uint16pub': UInt16,
        'uint32pub': UInt32,
        'uint64pub': UInt64,
        'int8pub': Int8,
        'int16pub': Int16,
        'int32pub': Int32,
        'int64pub': Int64,
        'float32pub': Float32,
        'float64pub': Float64,
        'boolpub': Bool,
        'navsatfixpub': NavSatFix,
        'batterystatepub': BatteryState,
        'imagepub': Image,
        'compressedimagepub': CompressedImage,
        'pointcloud2pub': PointCloud2,
        'laserscanpub': LaserScan,
        'pointstampedpub': PointStamped,
        'posestampedpub': PoseStamped,
        'posewithcovariancestampedpub': PoseWithCovarianceStamped,
        'twistpub': Twist,
        'occupancygridpub': OccupancyGrid,
        'odometrypub': Odometry,
        'pathpub': Path,
        'logpub': Log,
        'jointstatepub': JointState,
        'joypub': Joy,
        'markerarraypub': MarkerArray
    }

    def __init__(self, selected_topics: list, show_binary: bool = False):
        super().__init__('universal_subscriber')
        
        self.show_binary = show_binary
        self.message_counts = {topic: 0 for topic in selected_topics}
        self.start_time = datetime.now()

        # QoS profile for the subscriptions
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscriptions only for selected topics
        for topic in selected_topics:
            if topic in self.TOPIC_TYPES:
                msg_type = self.TOPIC_TYPES[topic]
                self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, t=topic, mt=msg_type: self.callback(msg, t, mt),
                    qos
                )
                self.get_logger().info(f'Subscribed to: {topic} ({msg_type.__name__})')
            else:
                self.get_logger().error(f'Unknown topic type: {topic}')

    def message_to_dict(self, msg):
        """Convert ROS message to dictionary, handling nested messages"""
        if hasattr(msg, 'get_fields_and_field_types'):
            result = {}
            for field_name, _ in msg.get_fields_and_field_types().items():
                value = getattr(msg, field_name)
                if hasattr(value, 'get_fields_and_field_types'):
                    result[field_name] = self.message_to_dict(value)
                elif isinstance(value, (list, tuple)):
                    result[field_name] = [
                        self.message_to_dict(item) if hasattr(item, 'get_fields_and_field_types')
                        else item
                        for item in value
                    ]
                else:
                    result[field_name] = value
            return result
        return str(msg)

    def format_message(self, msg_dict, indent=0):
        """Format message dictionary in a readable way"""
        output = []
        spaces = " " * indent
        
        for key, value in msg_dict.items():
            if isinstance(value, dict):
                output.append(f"{spaces}{Fore.YELLOW}{key}:{Style.RESET_ALL}")
                output.append(self.format_message(value, indent + 2))
            elif isinstance(value, list):
                if value:  # Only show if list is not empty
                    output.append(f"{spaces}{Fore.YELLOW}{key}: {Style.RESET_ALL}[")
                    for item in value[:5]:  # Limit array output
                        item_str = self.format_message(item, indent + 2) if isinstance(item, dict) else f"{spaces}  {item}"
                        output.append(item_str)
                    if len(value) > 5:
                        output.append(f"{spaces}  ... ({len(value)} items total)")
                    output.append(f"{spaces}]")
            else:
                output.append(f"{spaces}{Fore.YELLOW}{key}:{Style.RESET_ALL} {value}")
        
        return "\n".join(output)

    def callback(self, msg: Any, topic: str, msg_type: Type) -> None:
        self.message_counts[topic] += 1
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        
        # Convert message to dictionary
        msg_dict = self.message_to_dict(msg)
        
        # Print formatted output
        print(f"\n{Fore.CYAN}[{timestamp}] {Fore.GREEN}{topic}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}Type: {msg_type.__name__}{Style.RESET_ALL}")
        print(self.format_message(msg_dict))
        
        if self.show_binary:
            binary = serialize_message(msg)
            print(f"{Fore.MAGENTA}Binary: {binary}{Style.RESET_ALL}")
        
        # Print statistics
        elapsed = (datetime.now() - self.start_time).total_seconds()
        rate = self.message_counts[topic] / elapsed if elapsed > 0 else 0
        print(f"{Fore.BLUE}Messages: {self.message_counts[topic]} | Rate: {rate:.2f} Hz{Style.RESET_ALL}")

def list_available_topics():
    """Print all available topics in a formatted way"""
    print(f"\n{Fore.GREEN}Available Topics:{Style.RESET_ALL}")
    for topic, msg_type in UniversalSubscriber.TOPIC_TYPES.items():
        print(f"  {Fore.YELLOW}{topic}{Style.RESET_ALL} ({msg_type.__name__})")

def main():
    parser = argparse.ArgumentParser(description='ROS2 Universal Subscriber with topic selection')
    parser.add_argument('topics', nargs='*', help='Topics to subscribe to')
    parser.add_argument('--list', '-l', action='store_true', help='List all available topics')
    parser.add_argument('--binary', '-b', action='store_true', help='Show binary message content')
    parser.add_argument('--all', '-a', action='store_true', help='Subscribe to all available topics')
    args = parser.parse_args()

    if args.list:
        list_available_topics()
        return

    if not args.topics and not args.all:
        parser.print_help()
        return

    rclpy.init()

    try:
        selected_topics = list(UniversalSubscriber.TOPIC_TYPES.keys()) if args.all else args.topics
        subscriber = UniversalSubscriber(selected_topics, args.binary)
        
        print(f"\n{Fore.GREEN}Starting Universal Subscriber...{Style.RESET_ALL}")
        print(f"Listening to topics: {', '.join(selected_topics)}")
        print(f"{Fore.YELLOW}Press Ctrl+C to exit{Style.RESET_ALL}\n")
        
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        print(f"\n{Fore.RED}Shutting down...{Style.RESET_ALL}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
