#!/usr/bin/env python3
# publisher.py - Enhanced stub implementation for topic publisher

import json
from geometry_msgs.msg import Twist
import importlib # Added for dynamic type resolution
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # <<< Import QoS

class TopicPublisher:
    """
    Handles publishing messages to ROS topics based on messages received from the controller via ZMQ.
    Supports dynamic reconfiguration of publishers.
    """
    
    def __init__(self, node, topic_mappings, defaults, message_converter, zmq_client, logger=None):
        """
        Initialize the topic publisher.
        
        Args:
            node: The ROS node
            topic_mappings: Initial list of topic mapping configurations
            defaults: Initial default values for topic mappings
            message_converter: The message converter to use
            zmq_client: The ZeroMQ client to use
            logger: Logger instance
        """
        self.node = node
        self.topic_mappings = topic_mappings # Store initial mappings
        self.defaults = defaults           # Store initial defaults
        self.message_converter = message_converter
        self.zmq_client = zmq_client
        self.logger = logger or node.get_logger()
        self.publishers = {} # Key: ott_topic, Value: ROS Publisher object
        
        self.logger.info("TopicPublisher initializing...")

        # Apply initial configuration
        self.update_publishers(topic_mappings, defaults, is_initial_setup=True)
        
        # Subscribe to ZMQ messages - using a broad prefix for simplicity now.
        # TODO: Consider dynamically updating ZMQ subscriptions if needed for performance/granularity
        if self.publishers: # Only subscribe if we expect to publish something
             self._subscribe_to_zmq_topics()
        
        self.logger.info(f"TopicPublisher initialization complete.")

    def _resolve_message_type(self, message_type_str):
        """Dynamically resolve a ROS message type from its string representation."""
        # (Same implementation as in TopicManager - consider moving to a shared util)
        try:
            parts = message_type_str.split('/')
            if len(parts) < 3:
                self.logger.error(f"Invalid message type format: {message_type_str}")
                return None
            package_name, module_name, message_name = parts[0], parts[1], parts[2]
            module_path = f"{package_name}.{module_name}"
            module = importlib.import_module(module_path)
            return getattr(module, message_name)
        except Exception as e:
            self.logger.error(f"Failed to resolve message type {message_type_str}: {e}")
            return None

    def _create_publisher(self, mapping):
        """Creates a single ROS publisher based on mapping configuration."""
        ott_topic = mapping.get('ott')
        ros_topic = mapping.get('ros_topic')
        msg_type_str = mapping.get('message_type')

        if not all([ott_topic, ros_topic, msg_type_str]):
            self.logger.error(f"Skipping publisher creation due to missing info in mapping: {mapping}")
            return False

        if ott_topic in self.publishers:
            self.logger.warning(f"Publisher for OTT topic '{ott_topic}' already exists. Skipping creation.")
            return True # Assume existing one is okay

        message_class = self._resolve_message_type(msg_type_str)
        if message_class is None:
            return False

        # Consider autodiscovery of QoS settings based on discovery of pub/sub QoS settings
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1 # Keep only the latest for velocity commands is often best
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10 # Default depth if needed
        )

        # TODO: Choose QoS based on mapping config or default?
        # For now, hardcode BEST_EFFORT for /cmd_vel, RELIABLE otherwise
        chosen_qos = reliable_qos # Default
        if ros_topic == "/cmd_vel": # Specific override for cmd_vel
            self.logger.info(f"Using BEST_EFFORT QoS for {ros_topic}")
            chosen_qos = best_effort_qos
        else:
             self.logger.debug(f"Using default RELIABLE QoS for {ros_topic}")

        try:
            publisher = self.node.create_publisher(message_class, ros_topic, chosen_qos)
            self.publishers[ott_topic] = publisher
            self.logger.debug(f"Created publisher for OTT='{ott_topic}' -> ROS='{ros_topic}' (Type: {msg_type_str}, QoS: {chosen_qos.reliability})")
            return True
        except Exception as e:
            self.logger.error(f"Error creating publisher for ROS topic '{ros_topic}': {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return False

    def update_publishers(self, new_topic_mappings, new_defaults, is_initial_setup=False):
        """Dynamically update ROS publishers based on new configuration."""
        action = "Applying initial" if is_initial_setup else "Updating"
        self.logger.info(f"{action} ROS publisher configuration...")

        new_publisher_config = {} # Key: ott_topic, Value: mapping dict
        default_direction = new_defaults.get('direction', 'OUTBOUND') # Default still OUTBOUND

        # 1. Filter new config for relevant inbound topics
        for mapping in new_topic_mappings:
            direction = mapping.get('direction', default_direction)
            if direction == 'INBOUND':
                ott_topic = mapping.get('ott')
                if ott_topic:
                    full_mapping = new_defaults.copy()
                    full_mapping.update(mapping)
                    new_publisher_config[ott_topic] = full_mapping
                else:
                    self.logger.warning(f"Skipping inbound mapping due to missing 'ott' topic: {mapping}")

        current_ott_topics = set(self.publishers.keys())
        new_ott_topics = set(new_publisher_config.keys())

        # 2. Identify changes
        topics_to_add = new_ott_topics - current_ott_topics
        topics_to_remove = current_ott_topics - new_ott_topics
        topics_to_check = current_ott_topics.intersection(new_ott_topics)

        # 3. Remove old/stale publishers
        for ott_topic in topics_to_remove:
            self.logger.debug(f"Removing publisher for OTT topic: {ott_topic}")
            if ott_topic in self.publishers:
                try:
                    self.node.destroy_publisher(self.publishers[ott_topic])
                except Exception as e:
                    self.logger.error(f"Error destroying publisher for {ott_topic}: {e}")
                del self.publishers[ott_topic]
            else:
                 self.logger.warning(f"Attempted to remove publisher for {ott_topic}, but it was not found.")

        # 4. Add new publishers
        for ott_topic in topics_to_add:
            self.logger.debug(f"Adding publisher for OTT topic: {ott_topic}")
            if not self._create_publisher(new_publisher_config[ott_topic]):
                 self.logger.error(f"Failed to create new publisher for {ott_topic}")

        # 5. Check existing publishers for changes (Treat as remove+add for now)
        for ott_topic in topics_to_check:
            current_publisher = self.publishers[ott_topic]
            new_mapping = new_publisher_config[ott_topic]
            # Recreate if ROS topic or message type changes
            # Note: current_publisher object doesn't easily expose original mapping details, 
            # we might need to store the mapping alongside the publisher if more complex checks are needed.
            # For now, let's check if the target ROS topic differs in the new mapping.
            current_ros_topic = current_publisher.topic_name # Get topic name from publisher
            if current_ros_topic != new_mapping.get('ros_topic') or \
               current_publisher.msg_type.__name__ != self._resolve_message_type(new_mapping.get('message_type')).__name__:
                 self.logger.debug(f"Recreating publisher for modified OTT topic: {ott_topic}")
                 # Remove
                 if ott_topic in self.publishers:
                     try:
                         self.node.destroy_publisher(self.publishers[ott_topic])
                     except Exception as e:
                         self.logger.error(f"Error destroying publisher for {ott_topic} during recreate: {e}")
                     del self.publishers[ott_topic]
                 # Add
                 if not self._create_publisher(new_mapping):
                     self.logger.error(f"Failed to recreate publisher for {ott_topic}")

        # 6. Update internal state
        self.topic_mappings = new_topic_mappings
        self.defaults = new_defaults
        self.logger.info(f"Publisher update complete. Active publishers: {len(self.publishers)}")
        self.logger.debug(f"Active publisher OTT topics: {list(self.publishers.keys())}")
        
        # 7. Ensure ZMQ subscription is active if needed
        if self.publishers and not self.zmq_client.receive_thread.is_alive(): # Check if thread running
             self.logger.warning("Publishers exist but ZMQ receive thread is not running. Attempting restart.")
             self._subscribe_to_zmq_topics()
        elif not self.publishers and self.zmq_client.receive_thread and self.zmq_client.receive_thread.is_alive():
             # Optional: Stop ZMQ receiving if no publishers are left? Depends on ZmqClient design.
             # self.zmq_client.stop_receiving() # Assuming such a method exists
             self.logger.info("No active publishers, ZMQ receive thread remains active (shared).")
             pass 

    def _subscribe_to_zmq_topics(self):
        """Subscribes to relevant ZMQ topics for receiving commands."""
        # Currently subscribes to a broad prefix. Refine if needed.
        zmq_topic_prefix = "teleop.control." # TODO: Make configurable or derive from mappings?
        try:
            self.logger.debug(f"Starting/Ensuring ZMQ subscription to '{zmq_topic_prefix}'")
            # Ensure ZeroMQ client is initialized before starting to receive
            self.zmq_client.start_receiving(zmq_topic_prefix, self.handle_controller_message)
        except Exception as e:
            self.logger.error(f"Failed to start/ensure ZMQ subscription to '{zmq_topic_prefix}': {e}")
            # Decide if this is fatal or recoverable

    def handle_controller_message(self, ott_message):
        """
        Handle a message from the controller.
        
        Args:
            ott_message: The OTT message from the controller (JSON string)
        """
        try:
            # Log the raw message with more details
            self.logger.debug(f"TopicPublisher: Received raw message: {ott_message}")
            
            # Parse the message
            message_dict = json.loads(ott_message)
            
            # Log the parsed message structure
            self.logger.debug(f"TopicPublisher: Parsed message keys: {list(message_dict.keys())}")
            
            # --- Adapt to potential Flatbuffers format --- 
            # Check if it looks like the structure sent by ControlWebSocketHandler
            topic = message_dict.get('ott_topic') # Check for ott_topic first
            data = message_dict.get('data')

            # Fallback to older (?) format if above fields aren't present
            if topic is None:
                 topic = message_dict.get('topic')
                 data = message_dict.get('data', {}) # Use original data field
            
            if not topic:
                self.logger.warning("TopicPublisher: Received message with no 'ott_topic' or 'topic' field")
                return
            
            self.logger.debug(f"TopicPublisher: Processing message for topic: {topic}")
            # self.logger.debug(f"TopicPublisher: Available publishers: {list(self.publishers.keys())}")
            
            if topic in self.publishers:
                publisher = self.publishers[topic]
                
                # Log the data in more detail
                # self.logger.debug(f"TopicPublisher: Message data: {json.dumps(data)}")
                
                # Find the corresponding mapping for this topic
                # Need to search self.topic_mappings as it's updated
                mapping = next((m for m in self.topic_mappings if m.get('ott') == topic and m.get('direction', self.defaults.get('direction')) == 'INBOUND'), None)
                
                if mapping:
                    # Determine target frame_id
                    default_frame_id = 'base_link' # Sensible default for velocity/stamped commands
                    target_frame_id = mapping.get('frame_id', default_frame_id)

                    # Convert JSON data to ROS message, passing the frame_id
                    ros_message = self.message_converter.convert_to_ros_message(
                        mapping['message_type'], 
                        data,
                        target_frame_id # Pass the determined frame_id
                    )
                    
                    # Publish the converted message
                    if ros_message:
                        publisher.publish(ros_message)
                        self.logger.debug(f"TopicPublisher: Message published successfully to {mapping['ros_topic']} using frame_id '{target_frame_id}'")
                    else:
                         self.logger.error(f"TopicPublisher: Failed to convert message data for topic {topic}")
                else:
                    self.logger.warning(f"TopicPublisher: No active mapping configuration found for topic: {topic}")
            else:
                # This might happen briefly during reconfiguration
                self.logger.warning(f"TopicPublisher: No publisher found for topic: {topic}")
                # self.logger.debug(f"TopicPublisher: Available publishers: {list(self.publishers.keys())}")
        
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
        topics_to_remove = list(self.publishers.keys())
        for topic in topics_to_remove:
            self.logger.info(f"Shutting down publisher for {topic}")
            if topic in self.publishers:
                try:
                    self.node.destroy_publisher(self.publishers[topic])
                except Exception as e:
                    self.logger.error(f"Error destroying publisher during shutdown for {topic}: {e}")
        self.publishers.clear()
        self.logger.info("TopicPublisher shutdown complete") 