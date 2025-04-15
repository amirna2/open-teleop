#!/usr/bin/env python3
# topic_manager.py - Implementation for topic subscription manager

import importlib
import flatbuffers
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Import the generated FlatBuffer code
from ros_gateway.flatbuffers.open_teleop.message import OttMessage, ContentType

class TopicManager:
    """
    Implementation of the topic manager for the ROS Gateway.
    Responsible for managing ROS topic subscriptions.
    """
    
    def __init__(self, node, topic_mappings, defaults, message_converter, zmq_client, logger=None):
        """
        Initialize the topic manager.
        
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
        self.subscriptions = {}
        
        # Log all available topic mappings
        self.logger.info(f"TopicManager received {len(topic_mappings)} topic mappings")
        
        # Create subscribers for outbound topics
        outbound_count = 0
        for mapping in topic_mappings:
            direction = mapping.get('direction', defaults.get('direction', 'OUTBOUND'))
            if direction == 'OUTBOUND':
                # Process all outbound topics
                self.logger.info(f"Processing outbound topic: {mapping['ros_topic']} of type {mapping['message_type']}")
                success = self._create_subscriber(mapping)
                if success:
                    outbound_count += 1
                else:
                    self.logger.error(f"Failed to create subscriber for {mapping['ros_topic']}")
        
        self.logger.info(f"TopicManager initialized with {outbound_count} active subscribers")
        self.logger.info(f"Active subscriptions: {list(self.subscriptions.keys())}")
    
    def _resolve_message_type(self, message_type_str):
        """
        Dynamically resolve a ROS message type from its string representation.
        
        Args:
            message_type_str: The message type as a string (e.g., 'sensor_msgs/msg/BatteryState')
            
        Returns:
            The resolved message type class or None if resolution fails
        """
        try:
            # Split the message type string into its components
            parts = message_type_str.split('/')
            if len(parts) < 3:
                self.logger.error(f"Invalid message type format: {message_type_str}, expected 'package/msg/Type'")
                return None
            
            package_name = parts[0]
            module_name = parts[1]
            message_name = parts[2]
            
            # Import the module
            module_path = f"{package_name}.{module_name}"
            module = importlib.import_module(module_path)
            
            # Get the message class
            message_class = getattr(module, message_name)
            
            return message_class
        except (ImportError, AttributeError, ValueError) as e:
            self.logger.error(f"Failed to resolve message type {message_type_str}: {e}")
            return None
    
    def _create_subscriber(self, mapping):
        """
        Create a ROS2 subscriber for a topic mapping.
        
        Args:
            mapping: The topic mapping configuration
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            ros_topic = mapping['ros_topic']
            ott_topic = mapping['ott']
            message_type_str = mapping['message_type']
            priority = mapping.get('priority', self.defaults.get('priority', 'STANDARD'))
            
            # Resolve the message type
            message_class = self._resolve_message_type(message_type_str)
            if message_class is None:
                self.logger.error(f"Could not resolve message type {message_type_str} for topic {ros_topic}")
                return False
            
            # Set up QoS profile
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
            
            self.logger.info(f"Creating subscriber for {ros_topic} -> {ott_topic} (priority: {priority})")
            
            # Create a generic callback
            def callback(msg):
                self.logger.info(f"Received message on {ros_topic}")
                self.handle_message(msg, ros_topic)
            
            # Create the subscriber using the dynamically resolved message type
            subscriber = self.node.create_subscription(
                message_class,
                ros_topic,
                callback,
                qos
            )
            
            # Store subscriber information
            self.subscriptions[ros_topic] = {
                'subscriber': subscriber,
                'ott_topic': ott_topic,
                'message_type': message_type_str,
                'priority': priority
            }
            
            self.logger.info(f"Successfully subscribed to {ros_topic} with message type {message_type_str}")
            return True
        except Exception as e:
            self.logger.error(f"Error creating subscriber for {mapping['ros_topic']}: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return False
    
    def handle_message(self, msg, topic_name):
        """
        Handle a message from a ROS topic and forward it to the controller.
        
        Args:
            msg: The ROS message
            topic_name: The name of the topic
        """
        self.logger.debug(f"handle_message invoked for topic: {topic_name}")
        
        if topic_name not in self.subscriptions:
            self.logger.warning(f"Received message on unknown topic: {topic_name}")
            return
        
        subscription = self.subscriptions[topic_name]
        ott_topic = subscription['ott_topic']
        
        self.logger.debug(f"Attempting to serialize ROS message for {topic_name}")
        try:
            # Serialize the ROS message to CDR format
            serialized_msg = serialize_message(msg)
            self.logger.debug(f"Serialization successful for {topic_name}, size: {len(serialized_msg)}")
            
            # Get current timestamp in nanoseconds
            timestamp_ns = self.node.get_clock().now().nanoseconds
            
            # Create FlatBuffer message
            builder = flatbuffers.Builder(1024)
            
            self.logger.debug(f"Creating FlatBuffer for {topic_name}")

            # Create the OTT topic string
            ott_fb = builder.CreateString(ott_topic)
            
            # Create the payload byte vector
            payload = builder.CreateByteVector(serialized_msg)
            
            # Start building the OttMessage
            OttMessage.OttMessageStart(builder)
            OttMessage.OttMessageAddVersion(builder, 1)
            OttMessage.OttMessageAddPayload(builder, payload)
            OttMessage.OttMessageAddContentType(builder, ContentType.ContentType.ROS2_MESSAGE)
            OttMessage.OttMessageAddOtt(builder, ott_fb)
            OttMessage.OttMessageAddTimestampNs(builder, timestamp_ns)
            
            # Finish the message
            message = OttMessage.OttMessageEnd(builder)
            builder.Finish(message)
            
            # Get the binary buffer and send it to the controller
            buf = builder.Output()
            self.logger.debug(f"FlatBuffer created for {topic_name}, size: {len(buf)}")
            
            self.logger.debug(f"Sending raw FlatBuffer ({len(buf)} bytes) for {ott_topic}")
            self.logger.debug(f"Attempting ZMQ send for {topic_name} -> {ott_topic}")
            reply_str = self.zmq_client.send_request_binary(buf) # New method
            self.logger.debug(f"ZMQ send attempted for {topic_name}")

            # Log the reply from the controller
            if reply_str:
                self.logger.info(f"Received ACK/reply from controller: {reply_str[:100]}...")
            else:
                self.logger.warning(f"Did not receive reply from controller for {ott_topic}")
            
            self.logger.info(f"Successfully forwarded message from {topic_name} to {ott_topic}")
            
        except Exception as e:
            self.logger.error(f"Error handling message from {topic_name} (type: {type(msg).__name__}): {e}")
            import traceback
            self.logger.error(traceback.format_exc())
    
    def shutdown(self):
        """Shut down all subscribers."""
        for topic, subscription in self.subscriptions.items():
            self.logger.info(f"Shutting down subscriber for {topic}")
            self.node.destroy_subscription(subscription['subscriber'])
        
        self.subscriptions.clear()
        self.logger.info("TopicManager shutdown complete") 