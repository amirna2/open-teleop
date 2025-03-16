#!/usr/bin/env python3
# gateway_node.py - Main ROS Gateway node for Open-Teleop

import rclpy
from rclpy.node import Node
import os
import sys
import signal
import yaml
from threading import Thread

# Import custom logger
import open_teleop_logger as log

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
        
        # Determine project root path (3 levels up from this file)
        # This file is in ros2_ws/src/ros_gateway/ros_gateway/gateway_node.py
        # So we need to go up 4 levels to get to the project root
        self.project_root = os.path.abspath(os.path.join(
            os.path.dirname(__file__), 
            '../../../../'
        ))
        
        # Set up custom logger with temporary location
        self.logger = log.get_logger(
            name=f"{self.get_name()}",
            log_dir="/tmp/open_teleop_logs",
            console_level=log.INFO,
            file_level=log.DEBUG
        )
        
        # Display welcome message
        self.logger.info('='*80)
        self.logger.info('Hello, I am the ROS Gateway for Open-Teleop!')
        self.logger.info('This is a minimal implementation with stub components.')
        self.logger.info('='*80)
        self.logger.info('Starting ROS Gateway for Open-Teleop')
        self.logger.info(f'Project root: {self.project_root}')
        
        # Load configuration - try project config first, then fall back to package config
        project_config_path = os.path.join(self.project_root, 'config/gateway_config_sample.yaml')
        package_config_path = os.path.join(os.path.dirname(__file__), '../config/gateway_config.yaml')
        
        config_path = self.declare_parameter('config_path', project_config_path).value
        if not os.path.exists(config_path):
            self.logger.warning(f"Config file not found at {config_path}, falling back to package config")
            config_path = package_config_path
            
        self.logger.info(f"Loading configuration from: {config_path}")
        self.config_loader = ConfigLoader(config_path)
        self.config = self.config_loader.load_config()
        
        # Display config info
        self.logger.info(f"Loaded configuration version: {self.config.get('version')}")
        self.logger.info(f"Config ID: {self.config.get('config_id')}")
        self.logger.info(f"Robot ID: {self.config.get('robot_id')}")
        
        # Configure logging
        self._configure_logging()
        
        # Initialize the message converter
        self.converter = MessageConverter()
        
        # Get ZeroMQ configuration
        zmq_config = self.config_loader.get_zmq_config()
        
        # Override with environment variables if set
        if 'TELEOP_ZMQ_CONTROLLER_ADDRESS' in os.environ:
            zmq_config['controller_address'] = os.environ['TELEOP_ZMQ_CONTROLLER_ADDRESS']
            self.logger.info(f"Using controller address from environment: {zmq_config['controller_address']}")
            
        if 'TELEOP_ZMQ_PUBLISH_ADDRESS' in os.environ:
            zmq_config['publish_address'] = os.environ['TELEOP_ZMQ_PUBLISH_ADDRESS']
            self.logger.info(f"Using publish address from environment: {zmq_config['publish_address']}")
        
        # Initialize the ZeroMQ client
        self.logger.info(f"ZMQ Controller address: {zmq_config['controller_address']}")
        self.zmq_client = ZmqClient(
            controller_address=zmq_config['controller_address'],
            publish_address=zmq_config['publish_address'],
            buffer_size=zmq_config['message_buffer_size'],
            reconnect_interval_ms=zmq_config['reconnect_interval_ms'],
            logger=self.logger
        )
        
        # Get topic mappings
        topic_mappings = self.config.get('topic_mappings', [])
        defaults = self.config.get('defaults', {})
        
        # Log topic mapping info
        ros2_topics = self.config_loader.get_topic_mappings_by_source_type('ROS2_CDM')
        ot_topics = self.config_loader.get_topic_mappings_by_source_type('OPEN_TELEOP')
        self.logger.info(f"Found {len(ros2_topics)} ROS2 topics and {len(ot_topics)} Open-Teleop topics")
        
        inbound_topics = self.config_loader.get_topic_mappings_by_direction('INBOUND')
        outbound_topics = self.config_loader.get_topic_mappings_by_direction('OUTBOUND')
        self.logger.info(f"Found {len(inbound_topics)} inbound topics and {len(outbound_topics)} outbound topics")
        
        # Initialize the topic manager (for ROS2 topics only)
        self.topic_manager = TopicManager(
            node=self, 
            topic_mappings=ros2_topics,
            defaults=defaults,
            message_converter=self.converter,
            zmq_client=self.zmq_client,
            logger=self.logger
        )
        
        # Initialize the topic publisher for incoming commands
        self.topic_publisher = TopicPublisher(
            node=self,
            topic_mappings=inbound_topics,
            defaults=defaults,
            message_converter=self.converter,
            zmq_client=self.zmq_client,
            logger=self.logger
        )
        
        # Setup a timer for heartbeat/monitoring
        self.create_timer(1.0, self.heartbeat_callback)
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.logger.info('ROS Gateway is ready')
    
    def _configure_logging(self):
        """Configure the logger based on the configuration file."""
        try:
            # We already have the basic logger set up, but we might want to update it
            # based on the configuration. For simplicity, we'll just keep the current logger.
            pass
            
        except Exception as e:
            # Fall back to original logger for error reporting
            self.logger.error(f"Error configuring logger: {e}")
    
    def heartbeat_callback(self):
        """Periodic heartbeat to monitor system status"""
        self.logger.debug('Gateway heartbeat')
        # Could add metrics reporting here
    
    def signal_handler(self, sig, frame):
        """Handle shutdown signals gracefully"""
        self.logger.info(f'Received signal {sig}, shutting down...')
        self.shutdown()
        sys.exit(0)
    
    def shutdown(self):
        """Clean shutdown of all resources"""
        self.logger.info('Shutting down ROS Gateway...')
        if hasattr(self, 'topic_manager'):
            self.topic_manager.shutdown()
        if hasattr(self, 'zmq_client'):
            self.zmq_client.shutdown()
        if hasattr(self, 'topic_publisher'):
            self.topic_publisher.shutdown()
        self.logger.info('Shutdown complete')


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