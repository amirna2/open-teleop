#!/usr/bin/env python3
# test_config_with_mocks.py - Test the config processor with mock subscribers and publishers

import os
import sys
import time
from typing import Dict, Any, List

# Add the ros_gateway package to the Python path
# Adjust path to account for script being in test directory
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../ros_gateway'))

# Import the config loader
from ros_gateway.config_processor.config_loader import ConfigLoader, ConfigValidationError

# Mock classes for testing
class MockZmqClient:
    """Mock ZeroMQ client for testing"""
    
    def __init__(self, controller_address, publish_address, buffer_size, reconnect_interval_ms, logger=None):
        self.controller_address = controller_address
        self.publish_address = publish_address
        self.buffer_size = buffer_size
        self.reconnect_interval_ms = reconnect_interval_ms
        self.logger = logger or MockLogger()
        self.connected = False
        
        print(f"[ZMQ] Created client with controller_address={controller_address}")
        
    def connect(self): 
        """Mock connection to ZeroMQ server"""
        print(f"[ZMQ] Connecting to {self.controller_address}...")
        time.sleep(0.1)  # Simulate connection time
        self.connected = True
        print(f"[ZMQ] Connected to {self.controller_address}")
        
    def send_message(self, topic, message):
        """Mock sending a message"""
        print(f"[ZMQ] Sending message to topic '{topic}': {message[:20]}...")
        
    def shutdown(self):
        """Mock shutdown"""
        print(f"[ZMQ] Shutting down client")
        self.connected = False


class MockTopicManager:
    """Mock topic manager for testing"""
    
    def __init__(self, node, topic_mappings, defaults, message_converter, zmq_client, logger=None):
        self.node = node
        self.topic_mappings = topic_mappings
        self.defaults = defaults
        self.message_converter = message_converter
        self.zmq_client = zmq_client
        self.logger = logger or MockLogger()
        self.subscribers = {}
        
        print(f"[TopicManager] Created with {len(topic_mappings)} topic mappings")
        
        # Create subscribers for all outbound topics
        for mapping in topic_mappings:
            if mapping.get('direction') == 'OUTBOUND' and mapping.get('source_type') == 'ROS2_CDM':
                self._create_subscriber(mapping)
                
    def _create_subscriber(self, mapping):
        """Mock creation of a subscriber"""
        ros_topic = mapping['ros_topic']
        ott = mapping['ott']
        message_type = mapping['message_type']
        priority = mapping.get('priority', self.defaults.get('priority', 'STANDARD'))
        
        print(f"[TopicManager] Creating subscriber for {ros_topic} -> {ott} (priority: {priority})")
        self.subscribers[ros_topic] = {
            'ott': ott,
            'message_type': message_type,
            'priority': priority
        }
        
    def shutdown(self):
        """Mock shutdown"""
        print(f"[TopicManager] Shutting down {len(self.subscribers)} subscribers")
        self.subscribers.clear()


class MockTopicPublisher:
    """Mock topic publisher for testing"""
    
    def __init__(self, node, topic_mappings, defaults, message_converter, zmq_client, logger=None):
        self.node = node
        self.topic_mappings = topic_mappings
        self.defaults = defaults
        self.message_converter = message_converter
        self.zmq_client = zmq_client
        self.logger = logger or MockLogger()
        self.publishers = {}
        
        print(f"[TopicPublisher] Created with {len(topic_mappings)} topic mappings")
        
        # Create publishers for all inbound topics
        for mapping in topic_mappings:
            if mapping.get('direction') == 'INBOUND' and mapping.get('source_type') == 'ROS2_CDM':
                self._create_publisher(mapping)
                
    def _create_publisher(self, mapping):
        """Mock creation of a publisher"""
        ros_topic = mapping['ros_topic']
        ott = mapping['ott']
        message_type = mapping['message_type']
        
        print(f"[TopicPublisher] Creating publisher for {ott} -> {ros_topic}")
        self.publishers[ott] = {
            'ros_topic': ros_topic,
            'message_type': message_type
        }
        
    def shutdown(self):
        """Mock shutdown"""
        print(f"[TopicPublisher] Shutting down {len(self.publishers)} publishers")
        self.publishers.clear()


class MockLogger:
    """Mock logger for testing"""
    
    def info(self, message):
        """Mock info logging"""
        print(f"[INFO] {message}")
        
    def warning(self, message):
        """Mock warning logging"""
        print(f"[WARNING] {message}")
        
    def error(self, message):
        """Mock error logging"""
        print(f"[ERROR] {message}")
        
    def debug(self, message):
        """Mock debug logging"""
        pass  # Don't print debug messages to keep output clean


class MockMessageConverter:
    """Mock message converter for testing"""
    
    def __init__(self):
        print("[MessageConverter] Created")
        
    def convert_to_ott(self, ros_msg, mapping):
        """Mock conversion of ROS message to OTT message"""
        return f"OTT message for {mapping['ott']}"


class MockNode:
    """Mock ROS node for testing"""
    
    def __init__(self, name):
        self.name = name
        print(f"[Node] Created node with name '{name}'")
        
    def get_name(self):
        """Get the node name"""
        return self.name


def main():
    """Test the config processor with mock implementations"""
    # Adjust path to account for script being in test directory
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../'))
    sample_config_path = os.path.join(project_root, 'config/gateway_config_sample.yaml')
    
    print(f"Testing config processor with mocks using: {sample_config_path}")
    
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
        print(f"Robot ID: {config.get('robot_id')}")
        
        # Create mock components
        print("\n=== Creating Mock Components ===")
        node = MockNode("ros_gateway")
        logger = MockLogger()
        converter = MockMessageConverter()
        
        # Get ZeroMQ configuration
        zmq_config = config_loader.get_zmq_config()
        zmq_client = MockZmqClient(
            controller_address=zmq_config['controller_address'],
            publish_address=zmq_config['publish_address'],
            buffer_size=zmq_config['message_buffer_size'],
            reconnect_interval_ms=zmq_config['reconnect_interval_ms'],
            logger=logger
        )
        
        # Connect to ZeroMQ server
        zmq_client.connect()
        
        # Get topic mappings by type and direction
        ros2_topics = config_loader.get_topic_mappings_by_source_type('ROS2_CDM')
        inbound_topics = config_loader.get_topic_mappings_by_direction('INBOUND')
        outbound_topics = config_loader.get_topic_mappings_by_direction('OUTBOUND')
        
        # Get defaults
        defaults = config.get('defaults', {})
        
        # Create topic manager (for outbound topics)
        print("\n=== Setting up Mock Topic Manager ===")
        topic_manager = MockTopicManager(
            node=node,
            topic_mappings=outbound_topics,
            defaults=defaults,
            message_converter=converter,
            zmq_client=zmq_client,
            logger=logger
        )
        
        # Create topic publisher (for inbound topics)
        print("\n=== Setting up Mock Topic Publisher ===")
        topic_publisher = MockTopicPublisher(
            node=node,
            topic_mappings=inbound_topics,
            defaults=defaults,
            message_converter=converter,
            zmq_client=zmq_client,
            logger=logger
        )
        
        # Simulate sending a message
        print("\n=== Simulating Message Flow ===")
        
        # Simulate an outbound message (ROS to Controller)
        battery_topic = next((t for t in ros2_topics if t['ros_topic'] == '/battery_state'), None)
        if battery_topic:
            print(f"\n[TEST] Simulating message on ROS topic '{battery_topic['ros_topic']}'")
            mock_message = "{'voltage': 12.5, 'percentage': 0.75}"
            ott_message = converter.convert_to_ott(mock_message, battery_topic)
            zmq_client.send_message(battery_topic['ott'], ott_message)
        
        # Simulate an inbound message (Controller to ROS)
        cmd_vel_topic = next((t for t in ros2_topics if t['ros_topic'] == '/cmd_vel'), None)
        if cmd_vel_topic:
            print(f"\n[TEST] Simulating message from controller for '{cmd_vel_topic['ott']}'")
            print(f"[TEST] Would publish to ROS topic '{cmd_vel_topic['ros_topic']}'")
            
        # Shutdown everything
        print("\n=== Shutting Down ===")
        topic_publisher.shutdown()
        topic_manager.shutdown()
        zmq_client.shutdown()
        
        print("\nTest completed successfully!")
        return 0
        
    except ConfigValidationError as e:
        print(f"Configuration validation error: {e}")
        return 1
    except Exception as e:
        print(f"Error in test: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main()) 