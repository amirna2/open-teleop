#!/usr/bin/env python3
# gateway_node.py - Main ROS Gateway node for Open-Teleop

import rclpy
from rclpy.node import Node
import os
import sys
import signal
import yaml
import json
import time
from threading import Thread, Event

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
        
        # Initialize the message converter
        self.converter = MessageConverter()
        
        # Get ZeroMQ configuration from environment variables with fallback defaults
        controller_address = os.environ.get('TELEOP_ZMQ_CONTROLLER_ADDRESS', 'tcp://localhost:5555')
        publish_address = os.environ.get('TELEOP_ZMQ_PUBLISH_ADDRESS', 'tcp://*:5556')
        message_buffer_size = int(os.environ.get('TELEOP_ZMQ_BUFFER_SIZE', '1000'))
        reconnect_interval_ms = int(os.environ.get('TELEOP_ZMQ_RECONNECT_INTERVAL_MS', '1000'))
        
        self.logger.info(f"Using ZMQ controller address: {controller_address}")
        self.logger.info(f"Using ZMQ publish address: {publish_address}")
        
        # Initialize the ZeroMQ client early with bootstrap configuration
        self.zmq_client = ZmqClient(
            controller_address=controller_address,
            publish_address=publish_address,
            buffer_size=message_buffer_size,
            reconnect_interval_ms=reconnect_interval_ms,
            logger=self.logger
        )
        
        # Request configuration from the controller
        self.config = self.request_configuration()
        
        # Display config info
        self.logger.info(f"Loaded configuration version: {self.config.get('version')}")
        self.logger.info(f"Config ID: {self.config.get('config_id')}")
        self.logger.info(f"Robot ID: {self.config.get('robot_id')}")
        
        # Configure logging
        self._configure_logging()
        
        # Get topic mappings
        topic_mappings = self.config.get('topic_mappings', [])
        defaults = self.config.get('defaults', {})
        
        # Log topic mapping info
        ros2_topics = self.filter_topic_mappings_by_source_type(topic_mappings, defaults, 'ROS2_CDM')
        ot_topics = self.filter_topic_mappings_by_source_type(topic_mappings, defaults, 'OPEN_TELEOP')
        self.logger.info(f"Found {len(ros2_topics)} ROS2 topics and {len(ot_topics)} Open-Teleop topics")
        
        inbound_topics = self.filter_topic_mappings_by_direction(topic_mappings, defaults, 'INBOUND')
        outbound_topics = self.filter_topic_mappings_by_direction(topic_mappings, defaults, 'OUTBOUND')
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
    
    def request_configuration(self):
        """
        Request configuration from the controller.
        
        Returns:
            Dict containing the configuration received from the controller
        """
        self.logger.info("Requesting configuration from controller...")
        
        # Try to get configuration directly from the controller using our ZmqClient
        config = self.zmq_client.request_config()
        
        if config is not None:
            self.logger.info("Successfully received configuration from controller")
            
            # Subscribe to configuration updates
            self.zmq_client.subscribe_to_config_updates(self.handle_config_update)
            
            return config
        
        # Fallback to local configuration if controller request fails
        self.logger.warning("Failed to get configuration from controller, using default configuration")
        
        # Use empty default configuration as fallback
        return {
            "version": "1.0",
            "config_id": "default",
            "lastUpdated": time.time(),
            "robot_id": os.environ.get('TELEOP_ROBOT_ID', 'unknown'),
            "topic_mappings": [],
            "defaults": {
                "priority": "STANDARD",
                "direction": "OUTBOUND",
                "source_type": "ROS2_CDM"
            },
            "settings": {
                "message_buffer_size": 1000,
                "reconnect_interval_ms": 1000
            }
        }
    
    def handle_config_update(self, message):
        """
        Handle configuration updates from the controller.
        
        Args:
            message: The configuration update message
        """
        try:
            data = json.loads(message)
            
            if data.get('type') == 'CONFIG_UPDATE':
                self.logger.info("Received configuration update from controller")
                new_config = data.get('data', {})
                
                # Store the new configuration
                self.config = new_config
                
                # Display config info
                self.logger.info(f"Updated configuration version: {self.config.get('version')}")
                self.logger.info(f"Config ID: {self.config.get('config_id')}")
                
                # TODO: Implement dynamic reconfiguration based on the new config
                self.logger.warning("Dynamic reconfiguration not implemented yet")
                
            elif data.get('type') == 'CONFIG_UPDATED':
                self.logger.info("Received notification that configuration has changed")
                
                # Proactively request the new configuration
                self.config = self.zmq_client.request_config()
                
                if self.config is not None:
                    self.logger.info("Successfully received updated configuration")
                    # TODO: Implement dynamic reconfiguration
                
        except Exception as e:
            self.logger.error(f"Error processing configuration update: {e}")
    
    def filter_topic_mappings_by_source_type(self, topic_mappings, defaults, source_type):
        """Filter topic mappings by source type (replacement for config_loader method)"""
        default_source_type = defaults.get('source_type')
        result = []
        
        for mapping in topic_mappings:
            map_source_type = mapping.get('source_type', default_source_type)
            if map_source_type == source_type:
                # Apply defaults for missing values
                config = defaults.copy()
                config.update(mapping)
                result.append(config)
                
        return result
    
    def filter_topic_mappings_by_direction(self, topic_mappings, defaults, direction):
        """Filter topic mappings by direction (replacement for config_loader method)"""
        default_direction = defaults.get('direction', 'OUTBOUND')
        result = []
        
        for mapping in topic_mappings:
            map_direction = mapping.get('direction', default_direction)
            if map_direction == direction:
                # Apply defaults for missing values
                config = defaults.copy()
                config.update(mapping)
                result.append(config)
                
        return result


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