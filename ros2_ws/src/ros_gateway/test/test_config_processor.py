#!/usr/bin/env python3
# test_config_processor.py - Test script for the config processor

import os
import sys
import pprint

# Add the ros_gateway package to the Python path
# Adjust path to account for script being in test directory
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../ros_gateway'))

# Import the config loader
from ros_gateway.config_processor.config_loader import ConfigLoader, ConfigValidationError

def main():
    """Test the config processor with our sample configuration"""
    # Adjust path to account for script being in test directory
    # Go up one more level from the file location 
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../'))
    sample_config_path = os.path.join(project_root, 'config/gateway_config_sample.yaml')
    
    print(f"Testing config processor with: {sample_config_path}")
    
    if not os.path.exists(sample_config_path):
        print(f"ERROR: Sample config file not found at {sample_config_path}")
        return 1
    
    try:
        # Load the configuration
        config_loader = ConfigLoader(sample_config_path)
        config = config_loader.load_config()
        
        # Print metadata
        print("\n=== Configuration Metadata ===")
        print(f"Version: {config.get('version')}")
        print(f"Config ID: {config.get('config_id')}")
        print(f"Last Updated: {config.get('lastUpdated')}")
        print(f"Robot ID: {config.get('robot_id')}")
        
        # Print topic mapping counts
        ros2_topics = config_loader.get_topic_mappings_by_source_type('ROS2_CDM')
        ot_topics = config_loader.get_topic_mappings_by_source_type('OPEN_TELEOP')
        print(f"\nFound {len(ros2_topics)} ROS2 topics and {len(ot_topics)} Open-Teleop topics")
        
        inbound_topics = config_loader.get_topic_mappings_by_direction('INBOUND')
        outbound_topics = config_loader.get_topic_mappings_by_direction('OUTBOUND')
        print(f"Found {len(inbound_topics)} inbound topics and {len(outbound_topics)} outbound topics")
        
        # Print details of each topic mapping
        print("\n=== ROS2 Topics ===")
        for i, topic in enumerate(ros2_topics):
            print(f"{i+1}. {topic['ros_topic']} -> {topic['ott']} ({topic['direction']})")
        
        print("\n=== Open-Teleop Topics ===")
        for i, topic in enumerate(ot_topics):
            print(f"{i+1}. {topic['ott']} ({topic['direction']})")
        
        # Print ZMQ configuration
        zmq_config = config_loader.get_zmq_config()
        print("\n=== ZeroMQ Configuration ===")
        print(f"Controller Address: {zmq_config['controller_address']}")
        print(f"Publish Address: {zmq_config['publish_address']}")
        print(f"Message Buffer Size: {zmq_config['message_buffer_size']}")
        print(f"Reconnect Interval: {zmq_config['reconnect_interval_ms']}ms")
        
        # Sample look up of a specific topic
        battery_topic = config_loader.get_topic_config("/battery_state")
        if battery_topic:
            print("\n=== Battery Topic Details ===")
            pprint.pprint(battery_topic)
        
        print("\nConfiguration validation successful!")
        return 0
        
    except ConfigValidationError as e:
        print(f"Configuration validation error: {e}")
        return 1
    except Exception as e:
        print(f"Error testing config processor: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main()) 