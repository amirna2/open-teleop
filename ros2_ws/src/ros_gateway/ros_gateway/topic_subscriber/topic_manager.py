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
        self.topic_mappings = topic_mappings  # Initial mappings
        self.defaults = defaults            # Initial defaults
        self.message_converter = message_converter
        self.zmq_client = zmq_client
        self.logger = logger or node.get_logger()
        self.subscriptions = {} # Key: ros_topic, Value: dict{subscriber, ott_topic, ...}
        
        self.logger.debug(f"TopicManager received {len(topic_mappings)} initial topic mappings")
        
        # Apply initial configuration
        self.update_subscriptions(topic_mappings, defaults, is_initial_setup=True)
        
        self.logger.info(f"TopicManager initialization complete")

    def update_subscriptions(self, new_topic_mappings, new_defaults, is_initial_setup=False):
        """Dynamically update ROS subscriptions based on new configuration."""
        if not is_initial_setup:
            self.logger.info("Updating ROS subscriptions based on new configuration...")
        else:
             self.logger.info("Applying initial ROS subscription configuration...")

        new_subscriptions_config = {} # Key: ros_topic, Value: mapping dict
        default_direction = new_defaults.get('direction', 'OUTBOUND')

        # 1. Filter new config for relevant outbound subscriptions
        for mapping in new_topic_mappings:
            direction = mapping.get('direction', default_direction)
            if direction == 'OUTBOUND':
                ros_topic = mapping.get('ros_topic')
                if ros_topic:
                    # Apply defaults to the mapping before storing
                    full_mapping = new_defaults.copy()
                    full_mapping.update(mapping)
                    new_subscriptions_config[ros_topic] = full_mapping
                else:
                    self.logger.warning(f"Skipping outbound mapping due to missing 'ros_topic': {mapping}")

        current_ros_topics = set(self.subscriptions.keys())
        new_ros_topics = set(new_subscriptions_config.keys())

        # 2. Identify changes
        topics_to_add = new_ros_topics - current_ros_topics
        topics_to_remove = current_ros_topics - new_ros_topics
        topics_to_check = current_ros_topics.intersection(new_ros_topics)

        # 3. Remove old/stale subscriptions
        for topic in topics_to_remove:
            self.logger.debug(f"Removing subscription for ROS topic: {topic}")
            if topic in self.subscriptions:
                try:
                    self.node.destroy_subscription(self.subscriptions[topic]['subscriber'])
                except Exception as e:
                    self.logger.error(f"Error destroying subscription for {topic}: {e}")
                del self.subscriptions[topic]
            else:
                self.logger.warning(f"Attempted to remove subscription for {topic}, but it was not found in the registry.")

        # 4. Add new subscriptions
        for topic in topics_to_add:
            self.logger.debug(f"Adding subscription for ROS topic: {topic}")
            mapping = new_subscriptions_config[topic]
            if not self._create_subscriber(mapping):
                 self.logger.error(f"Failed to create new subscriber for {topic}")

        # 5. Check existing subscriptions for changes (Treat as remove+add for now)
        for topic in topics_to_check:
            current_mapping = self.subscriptions[topic]
            new_mapping = new_subscriptions_config[topic]
            # Simple check: if ott_topic or message_type changes, recreate
            # More granular checks could be added (e.g., for QoS, priority if it affects subscription)
            if current_mapping['ott_topic'] != new_mapping['ott'] or \
               current_mapping['message_type'] != new_mapping['message_type']:
                self.logger.debug(f"Recreating subscription for modified ROS topic: {topic}")
                # Remove existing
                if topic in self.subscriptions:
                    try:
                        self.node.destroy_subscription(self.subscriptions[topic]['subscriber'])
                    except Exception as e:
                        self.logger.error(f"Error destroying subscription for {topic} during recreate: {e}")
                    del self.subscriptions[topic]
                else:
                    self.logger.warning(f"Attempted to recreate subscription for {topic}, but it was not found in the registry.")
                # Add new
                if not self._create_subscriber(new_mapping):
                    self.logger.error(f"Failed to recreate subscriber for {topic}")
            # else: # No significant change detected
            #    self.logger.debug(f"No change detected for existing subscription: {topic}")

        # 6. Update internal state
        self.topic_mappings = new_topic_mappings
        self.defaults = new_defaults
        self.logger.info(f"Subscription update complete. Active subscriptions: {len(self.subscriptions)}")
        self.logger.debug(f"Active subscription topics: {list(self.subscriptions.keys())}")

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
        ros_topic = mapping.get('ros_topic', '[unknown_topic]') # Get topic early for logging
        self.logger.debug(f"_create_subscriber called for mapping: {mapping}")
        try:
            # Ensure ott_topic exists in the potentially updated mapping
            ott_topic = mapping.get('ott')
            if not ott_topic:
                 self.logger.error(f"Mapping for ROS topic '{ros_topic}' is missing the 'ott' field.")
                 return False # Return False immediately

            message_type_str = mapping.get('message_type')
            if not message_type_str:
                self.logger.error(f"Mapping for ROS topic '{ros_topic}' is missing the 'message_type' field.")
                return False # Return False immediately

            # Use updated defaults access if necessary, though mapping should be complete here
            priority = mapping.get('priority', self.defaults.get('priority', 'STANDARD'))

            # Check if already subscribed
            if ros_topic in self.subscriptions:
                self.logger.warning(f"Attempted to create subscriber for existing topic: {ros_topic}. Skipping.")
                # Consider if this should return True or False, or update existing.
                # Returning True assuming the existing one is desired.
                return True # Return True as it exists

            # Resolve the message type
            self.logger.debug(f"Attempting to resolve message type '{message_type_str}' for topic '{ros_topic}'")
            message_class = self._resolve_message_type(message_type_str)
            if message_class is None:
                self.logger.error(f"Could not resolve message type {message_type_str} for topic {ros_topic}")
                return False # Return False if type resolution fails
            self.logger.debug(f"Resolved message type for {ros_topic}: {message_class}")

            # Set up QoS profile
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
            self.logger.debug(f"Using QoS profile for {ros_topic}: Reliability={qos.reliability}, History={qos.history}, Depth={qos.depth}")

            self.logger.debug(f"Creating subscriber for {ros_topic} -> {ott_topic} (priority: {priority})")

            # Create a generic callback
            def callback(msg):
                # Minimal log to reduce noise inside callback itself initially
                # self.logger.debug(f"Callback invoked for {ros_topic}") # Maybe too noisy
                self.handle_message(msg, ros_topic)

            # ADDED: Log before create_subscription
            self.logger.debug(f"Attempting self.node.create_subscription for {ros_topic}...")

            # Create the subscriber using the dynamically resolved message type
            subscriber = self.node.create_subscription(
                message_class,
                ros_topic,
                callback,
                qos
            )

            # ADDED: Log after create_subscription
            if subscriber is None:
                 self.logger.error(f"self.node.create_subscription returned None for {ros_topic}")
                 return False # Return False if creation failed

            self.logger.debug(f"Successfully returned from self.node.create_subscription for {ros_topic}")

            # Store subscriber information
            self.subscriptions[ros_topic] = {
                'subscriber': subscriber,
                'ott_topic': ott_topic,
                'message_type': message_type_str,
                'priority': priority
            }
            self.logger.info(f"Successfully created and registered subscriber for ROS topic: {ros_topic}") # Use INFO level for success confirmation
            return True # Explicitly return True on success
        except Exception as e:
            self.logger.error(f"Exception creating subscriber for {ros_topic}: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return False # Return False on any exception
    
    def handle_message(self, msg, topic_name):
        """
        Handle a message from a ROS topic and forward it to the controller.
        
        Args:
            msg: The ROS message
            topic_name: The name of the topic
        """
        self.logger.debug(f"handle_message invoked for topic: {topic_name}")
        
        if topic_name not in self.subscriptions:
            # This might happen briefly during reconfiguration, log as warning
            self.logger.warning(f"Received message on topic '{topic_name}' but no active subscription found (possibly stale).")
            return
        
        subscription_info = self.subscriptions[topic_name]
        ott_topic = subscription_info['ott_topic']
        
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
                # Reduce log spam, maybe log only periodically or on change?
                self.logger.debug(f"Received ACK/reply from controller for {ott_topic}: {reply_str[:50]}...")
                # self.logger.info(f"Successfully forwarded message from {topic_name} to {ott_topic}")
            else:
                self.logger.warning(f"Did not receive reply from controller for {ott_topic}")
            
        except Exception as e:
            self.logger.error(f"Error handling message from {topic_name} (type: {type(msg).__name__}): {e}")
            import traceback
            self.logger.error(traceback.format_exc())
    
    def shutdown(self):
        """Shut down all subscribers."""
        # Use list keys to avoid modifying dict during iteration
        topics_to_remove = list(self.subscriptions.keys())
        for topic in topics_to_remove:
            self.logger.info(f"Shutting down subscriber for {topic}")
            if topic in self.subscriptions:
                try:
                    self.node.destroy_subscription(self.subscriptions[topic]['subscriber'])
                except Exception as e:
                     self.logger.error(f"Error destroying subscription during shutdown for {topic}: {e}")
                # Keep item in dict until loop finishes?
            else:
                self.logger.warning(f"Attempted to shutdown subscriber for {topic}, but it was not found.")
        
        self.subscriptions.clear()
        self.logger.info("TopicManager shutdown complete") 