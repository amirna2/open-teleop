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
                self.logger.info(f"Setting up publisher: {mapping['ott']} -> {mapping['ros_topic']}")
                
                # Create real publisher based on message type
                msg_type = mapping.get('message_type')
                if msg_type == 'geometry_msgs/msg/Twist':
                    self.publishers[mapping['ott']] = self.node.create_publisher(
                        Twist,
                        mapping['ros_topic'],
                        10  # QoS profile depth
                    )
                    self.logger.info(f"Created publisher for {mapping['ros_topic']} of type Twist")
                    self.logger.info(f"Publisher key in dictionary: '{mapping['ott']}'")
                
        self.logger.info(f"Successfully set up {len(self.publishers)} publishers")
        self.logger.info(f"Available publisher keys: {list(self.publishers.keys())}")
        
        # Now actually start receiving messages for testing
        self.logger.info("Starting to receive messages on 'teleop.control.' topic prefix")
        self.zmq_client.start_receiving("teleop.control.", self.handle_controller_message)
    
    def handle_controller_message(self, ott_message):
        """
        Handle a message from the controller.
        
        Args:
            ott_message: The OTT message from the controller (JSON string)
        """
        try:
            # Log the raw message with more details
            self.logger.info(f"TopicPublisher: Received raw message: {ott_message}")
            
            # Parse the message
            message_dict = json.loads(ott_message)
            
            # Log the parsed message structure
            self.logger.info(f"TopicPublisher: Parsed message keys: {list(message_dict.keys())}")
            
            topic = message_dict.get('topic')
            
            if not topic:
                self.logger.warning("TopicPublisher: Received message with no topic field")
                return
            
            self.logger.info(f"TopicPublisher: Processing message for topic: {topic}")
            self.logger.info(f"TopicPublisher: Available publishers: {list(self.publishers.keys())}")
            
            # For testing, specifically handle velocity commands
            if topic == 'teleop.control.velocity':
                if topic in self.publishers:
                    data = message_dict.get('data', {})
                    
                    # Log the data in more detail
                    self.logger.info(f"TopicPublisher: Message data: {json.dumps(data)}")
                    
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
                    
                    # Log the Twist message we're about to publish
                    self.logger.info(f"TopicPublisher: Publishing Twist message to /cmd_vel:")
                    self.logger.info(f"  - linear: x={twist.linear.x:.2f}, y={twist.linear.y:.2f}, z={twist.linear.z:.2f}")
                    self.logger.info(f"  - angular: x={twist.angular.x:.2f}, y={twist.angular.y:.2f}, z={twist.angular.z:.2f}")
                    
                    # Publish the message
                    self.publishers[topic].publish(twist)
                    self.logger.info("TopicPublisher: Message published successfully to /cmd_vel")
                else:
                    self.logger.warning(f"TopicPublisher: No publisher found for topic: {topic}")
                    self.logger.info(f"TopicPublisher: Available publishers: {list(self.publishers.keys())}")
        
        except json.JSONDecodeError as e:
            self.logger.error(f"TopicPublisher: Failed to parse message as JSON: {ott_message}")
            self.logger.error(f"TopicPublisher: JSON error: {str(e)}")
        except Exception as e:
            self.logger.error(f"TopicPublisher: Error handling controller message: {str(e)}")
            import traceback
            self.logger.error(f"TopicPublisher: {traceback.format_exc()}")
    
    def shutdown(self):
        """Shutdown the topic publisher."""
        self.logger.info("TopicPublisher: shutting down publishers")
        for topic, publisher in self.publishers.items():
            self.logger.info(f"Shutting down publisher for {topic}")
        self.publishers.clear() 