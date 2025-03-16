#!/usr/bin/env python3
# gateway_node.py - Main ROS Gateway node for Open-Teleop

import rclpy
from rclpy.node import Node
import os
import sys
import signal
import yaml
from threading import Thread

# Import our gateway components
from ros_gateway.config_processor.config_loader import ConfigLoader
from ros_gateway.zeromq_client.zmq_client import ZmqClient
from ros_gateway.topic_subscriber.topic_manager import TopicManager
from ros_gateway.message_converter.converter import MessageConverter
from ros_gateway.topic_publisher.publisher import TopicPublisher

class RosGateway(Node):
    """
    ROS Gateway for Open-Teleop.
    
    This node serves as the bridge between ROS2 and the Open-Teleop Controller, managing:
    - Dynamic topic subscriptions based on configuration
    - Message conversion between ROS and OttMessage format
    - ZeroMQ communication with the controller
    - Command reception and publishing to ROS
    """
    
    def __init__(self):
        super().__init__('ros_gateway')
        # Display welcome message
        self.get_logger().info('='*80)
        self.get_logger().info('Hello, I am the ROS Gateway for Open-Teleop!')
        self.get_logger().info('This is a minimal implementation with stub components.')
        self.get_logger().info('='*80)
        self.get_logger().info('Starting ROS Gateway for Open-Teleop')
        
        # Load configuration
        config_path = self.declare_parameter('config_path', 
                                           os.path.join(os.path.dirname(__file__), '../config/gateway_config.yaml')).value
        self.get_logger().info(f"Loading configuration from: {config_path}")
        self.config_loader = ConfigLoader(config_path)
        self.config = self.config_loader.load_config()
        
        # Initialize the message converter
        self.converter = MessageConverter()
        
        # Initialize the ZeroMQ client for controller communication
        zmq_config = self.config['gateway']['zmq']
        self.zmq_client = ZmqClient(
            controller_address=zmq_config['controller_address'],
            publish_address=zmq_config['publish_address'],
            buffer_size=zmq_config['message_buffer_size'],
            reconnect_interval_ms=zmq_config['reconnect_interval_ms'],
            logger=self.get_logger()
        )
        
        # Initialize the topic manager
        self.topic_manager = TopicManager(
            node=self, 
            topic_mappings=self.config['gateway']['topic_mappings'],
            defaults=self.config['gateway'].get('defaults', {}),
            message_converter=self.converter,
            zmq_client=self.zmq_client,
            logger=self.get_logger()
        )
        
        # Initialize the topic publisher for incoming commands
        self.topic_publisher = TopicPublisher(
            node=self,
            topic_mappings=self.config['gateway']['topic_mappings'],
            defaults=self.config['gateway'].get('defaults', {}),
            message_converter=self.converter,
            zmq_client=self.zmq_client,
            logger=self.get_logger()
        )
        
        # Setup a timer for heartbeat/monitoring
        self.create_timer(1.0, self.heartbeat_callback)
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.get_logger().info('ROS Gateway is ready')
    
    def heartbeat_callback(self):
        """Periodic heartbeat to monitor system status"""
        self.get_logger().debug('Gateway heartbeat')
        # Could add metrics reporting here
    
    def signal_handler(self, sig, frame):
        """Handle shutdown signals gracefully"""
        self.get_logger().info(f'Received signal {sig}, shutting down...')
        self.shutdown()
        sys.exit(0)
    
    def shutdown(self):
        """Clean shutdown of all resources"""
        self.get_logger().info('Shutting down ROS Gateway...')
        if hasattr(self, 'topic_manager'):
            self.topic_manager.shutdown()
        if hasattr(self, 'zmq_client'):
            self.zmq_client.shutdown()
        if hasattr(self, 'topic_publisher'):
            self.topic_publisher.shutdown()
        self.get_logger().info('Shutdown complete')


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    gateway = RosGateway()
    
    # Use a MultiThreadedExecutor to handle incoming data
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(gateway)
    
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # This keeps the main thread alive
        executor_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        gateway.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 