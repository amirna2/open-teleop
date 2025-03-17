#!/usr/bin/env python3
# publisher.py - Enhanced stub implementation for topic publisher

import json
from geometry_msgs.msg import Twist

class TopicPublisher:
    """
    Enhanced stub implementation of the topic publisher for the ROS Gateway.
    Responsible for publishing messages to ROS topics based on messages from the controller.
    """
    
    def __init__(self, node, topic_mappings, defaults, message_converter, zmq_client, logger=None):
        """
        Initialize the topic publisher.
        
        Args:
            node: The ROS node
            topic_mappings: List of topic mapping configurations
            defaults: Default values for topic mappings
            message_converter: The message converter to use
            zmq_client: The ZeroMQ client to use
            logger: Logger instance
        """
        self.node = node
        self.topic_mappings = topic_mappings
        self.defaults = defaults
        self.message_converter = message_converter
        self.zmq_client = zmq_client
        self.logger = logger or node.get_logger()
        self.publishers = {}
        
        self.logger.info("TopicPublisher stub initialized")
        
        # Create publishers for inbound topics
        inbound_topics = []
        for mapping in topic_mappings:
            direction = mapping.get('direction', defaults.get('direction', 'OUTBOUND'))
            if direction == 'INBOUND':
                inbound_topics.append(mapping)
                self.logger.info(f"Would publish to: {mapping['ott']} -> {mapping['ros_topic']}")
                
                # For testing purposes, create a real publisher for cmd_vel
                if mapping['ros_topic'] == '/cmd_vel':
                    self.publishers[mapping['ott']] = self.node.create_publisher(
                        Twist,
                        mapping['ros_topic'],
                        10  # QoS profile depth
                    )
                    self.logger.info(f"Created real publisher for {mapping['ros_topic']}")
                
        self.logger.info(f"Would set up {len(inbound_topics)} publishers if implemented")
        
        # Now actually start receiving messages for testing
        self.zmq_client.start_receiving(self.handle_controller_message)
    
    def handle_controller_message(self, ott_message):
        """
        Handle a message from the controller.
        
        Args:
            ott_message: The OTT message from the controller (JSON string)
        """
        try:
            # Parse the message
            message_dict = json.loads(ott_message)
            topic = message_dict.get('topic')
            
            if not topic:
                self.logger.warning("Received message with no topic")
                return
            
            self.logger.info(f"Received message for topic: {topic}")
            
            # For testing, specifically handle velocity commands
            if topic == 'teleop.control.velocity' and topic in self.publishers:
                data = message_dict.get('data', {})
                
                # Create a Twist message
                twist = Twist()
                
                # Set linear velocities
                linear = data.get('linear', {})
                twist.linear.x = linear.get('x', 0.0)
                twist.linear.y = linear.get('y', 0.0)
                twist.linear.z = linear.get('z', 0.0)
                
                # Set angular velocities
                angular = data.get('angular', {})
                twist.angular.x = angular.get('x', 0.0)
                twist.angular.y = angular.get('y', 0.0)
                twist.angular.z = angular.get('z', 0.0)
                
                # Publish the message
                self.publishers[topic].publish(twist)
                self.logger.info(f"Published Twist message to /cmd_vel: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")
        
        except json.JSONDecodeError:
            self.logger.error(f"Failed to parse message as JSON: {ott_message[:100]}...")
        except Exception as e:
            self.logger.error(f"Error handling controller message: {str(e)}")
    
    def shutdown(self):
        """Shutdown the topic publisher."""
        self.logger.info("TopicPublisher: shutting down publishers")
        for topic, publisher in self.publishers.items():
            self.logger.info(f"Shutting down publisher for {topic}")
        self.publishers.clear() 