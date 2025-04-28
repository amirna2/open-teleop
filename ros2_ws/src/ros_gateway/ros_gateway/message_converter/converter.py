#!/usr/bin/env python3
# converter.py - Stub implementation for message converter

import importlib
import json

class MessageConverter:
    """
    Stub implementation of the message converter for the ROS Gateway.
    Responsible for converting between ROS messages and OTT messages.
    """
    
    def __init__(self, node, logger=None):
        """Initialize the message converter."""
        self.node = node # Store node reference
        self.logger = logger
            
    def convert_to_ros_message(self, message_type_str, json_data):
        """
        Convert JSON data to a ROS message.
        
        Args:
            message_type_str: The ROS message type as a string (e.g., 'geometry_msgs/msg/Twist')
            json_data: The JSON data to convert to a ROS message
            
        Returns:
            A ROS message instance with fields populated from json_data
        """
        try:
            # Parse the message type string to get the module and class
            parts = message_type_str.split('/')
            if len(parts) != 3:
                raise ValueError(f"Invalid message type format: {message_type_str}. Expected format: 'package/msg/Type'")
                
            pkg_name, msg_dir, msg_type = parts
            
            # Import the message class
            module_name = f"{pkg_name}.{msg_dir}.{msg_type}"
            if self.logger:
                self.logger.debug(f"Importing message type: {module_name}")
            
            # Dynamic import of the message type
            module_path = f"{pkg_name}.{msg_dir}"
            module = importlib.import_module(module_path)
            message_class = getattr(module, msg_type)
            
            # Create an instance of the message
            ros_message = message_class()
            
            # --- Special handling for TwistStamped header ---
            if message_type_str == "geometry_msgs/msg/TwistStamped":
                if self.logger:
                     self.logger.debug(f"Populating header for TwistStamped")
                try:
                    frame_id = 'base_link' # TODO: Make this configurable
                    ros_message.header.stamp = self.node.get_clock().now().to_msg()
                    ros_message.header.frame_id = frame_id
                except Exception as e:
                    if self.logger:
                        self.logger.error(f"Failed to populate TwistStamped header: {e}")
            # --- End special handling ---
            
            # --- Populate the message with data from JSON --- 
            if message_type_str == "geometry_msgs/msg/TwistStamped":
                # For TwistStamped, populate the nested 'twist' field
                self._populate_ros_message(ros_message.twist, json_data)
            else:
                # For other types, populate the main message object
                self._populate_ros_message(ros_message, json_data)
            # --- End population logic ---
            
            return ros_message
            
        except ImportError as e:
            if self.logger:
                self.logger.error(f"Failed to import message type {message_type_str}: {e}")
            raise
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error converting JSON to ROS message: {e}")
            raise
    
    def _populate_ros_message(self, ros_message, json_data):
        """
        Recursively populate a ROS message with data from JSON.
        
        Args:
            ros_message: The ROS message instance to populate
            json_data: The JSON data to use for populating
        """
        if not json_data:
            return
            
        # Get message fields
        for field_name, field_value in json_data.items():
            if hasattr(ros_message, field_name):
                field = getattr(ros_message, field_name)
                
                # Handle nested messages
                if hasattr(field, '__slots__'):
                    # This is a nested message
                    self._populate_ros_message(field, field_value)
                else:
                    # This is a primitive field
                    try:
                        # Handle primitive types (cast to appropriate type)
                        if isinstance(field, float):
                            setattr(ros_message, field_name, float(field_value))
                        elif isinstance(field, int):
                            setattr(ros_message, field_name, int(field_value))
                        elif isinstance(field, bool):
                            setattr(ros_message, field_name, bool(field_value))
                        elif isinstance(field, str):
                            setattr(ros_message, field_name, str(field_value))
                        elif isinstance(field, list):
                            # Handle arrays/lists
                            if field and hasattr(field[0], '__slots__'):
                                # List of messages
                                new_list = []
                                for item in field_value:
                                    msg_instance = type(field[0])()
                                    self._populate_ros_message(msg_instance, item)
                                    new_list.append(msg_instance)
                                setattr(ros_message, field_name, new_list)
                            else:
                                # List of primitives
                                setattr(ros_message, field_name, field_value)
                        else:
                            # Default fallback
                            setattr(ros_message, field_name, field_value)
                    except (ValueError, TypeError) as e:
                        if self.logger:
                            self.logger.warning(f"Failed to set field {field_name}: {e}")
            elif self.logger:
                self.logger.warning(f"Field {field_name} not found in message type {type(ros_message).__name__}") 