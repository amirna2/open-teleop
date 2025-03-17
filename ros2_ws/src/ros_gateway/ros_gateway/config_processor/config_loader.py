#!/usr/bin/env python3
# config_loader.py - Configuration loading and validation for ROS Gateway

import os
import yaml
from typing import Dict, Any, List, Optional
import time

class ConfigValidationError(Exception):
    """Exception raised for config validation errors"""
    pass

class ConfigLoader:
    """
    Configuration loader for the ROS Gateway.
    
    Responsible for:
    - Loading YAML configuration
    - Validating configuration structure
    - Providing access to configuration values
    - Supporting dynamic config reloading
    """
    
    def __init__(self, config_path: str):
        """
        Initialize the config loader.
        
        Args:
            config_path: Path to the YAML configuration file
        """
        self.config_path = config_path
        self.last_load_time = 0
        self.config = {}
        
    def load_config(self) -> Dict[str, Any]:
        """
        Load the configuration from file.
        
        Returns:
            Dict containing the parsed configuration
        
        Raises:
            ConfigValidationError: If the configuration is invalid
            FileNotFoundError: If the config file is not found
            yaml.YAMLError: If the YAML parsing fails
        """
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Configuration file not found: {self.config_path}")
        
        with open(self.config_path, 'r') as file:
            self.config = yaml.safe_load(file)
            self.last_load_time = time.time()
        
        # Validate the configuration
        self._validate_config(self.config)
        
        return self.config
    
    def reload_if_changed(self) -> bool:
        """
        Reload the configuration if the file has changed since last load.
        
        Returns:
            True if the configuration was reloaded, False otherwise
        """
        if not os.path.exists(self.config_path):
            return False
        
        mtime = os.path.getmtime(self.config_path)
        if mtime > self.last_load_time:
            self.load_config()
            return True
        
        return False
    
    def _validate_config(self, config: Dict[str, Any]) -> None:
        """
        Validate the configuration structure.
        
        Args:
            config: The configuration dictionary to validate
            
        Raises:
            ConfigValidationError: If the configuration is invalid
        """
        # Check for required metadata fields
        required_metadata = ['version', 'config_id', 'lastUpdated', 'robot_id']
        for field in required_metadata:
            if field not in config:
                raise ConfigValidationError(f"Missing required metadata field: {field}")
        
        # Check for topic_mappings
        if 'topic_mappings' not in config:
            raise ConfigValidationError("Missing 'topic_mappings' section in configuration")
            
        topic_mappings = config['topic_mappings']
        if not isinstance(topic_mappings, list):
            raise ConfigValidationError("'topic_mappings' must be a list")
            
        # Check defaults section
        if 'defaults' not in config:
            raise ConfigValidationError("Missing 'defaults' section in configuration")
            
        defaults = config['defaults']
        for key in ['priority', 'direction', 'source_type']:
            if key not in defaults:
                raise ConfigValidationError(f"Missing '{key}' in defaults section")
                
        # Validate each topic mapping
        for idx, mapping in enumerate(topic_mappings):
            self._validate_topic_mapping(mapping, idx, defaults)
        
        # Check settings
        if 'settings' not in config:
            raise ConfigValidationError("Missing 'settings' section in configuration")
            
        settings = config['settings']
        required_settings = ['message_buffer_size', 'reconnect_interval_ms']
        for key in required_settings:
            if key not in settings:
                raise ConfigValidationError(f"Missing required setting: {key}")
    
    def _validate_topic_mapping(self, mapping: Dict[str, Any], idx: int, defaults: Dict[str, Any]) -> None:
        """
        Validate a single topic mapping.
        
        Args:
            mapping: The topic mapping dictionary to validate
            idx: The index of the mapping in the list (for error reporting)
            defaults: The default values to use for optional fields
            
        Raises:
            ConfigValidationError: If the mapping is invalid
        """
        # All mappings must have these fields
        required_keys = ['ott', 'priority', 'direction']
        for key in required_keys:
            if key not in mapping and key not in defaults:
                raise ConfigValidationError(f"Missing required key '{key}' in topic mapping {idx}")
        
        # Check source_type
        if 'source_type' not in mapping and 'source_type' not in defaults:
            raise ConfigValidationError(f"Missing 'source_type' in topic mapping {idx}")
            
        source_type = mapping.get('source_type', defaults.get('source_type'))
        valid_source_types = ['ROS2_CDR', 'OPEN_TELEOP']
        if source_type not in valid_source_types:
            raise ConfigValidationError(
                f"Invalid source_type '{source_type}' in topic mapping {idx}. "
                f"Must be one of {valid_source_types}"
            )
            
        # ROS2_CDR mappings must have ros_topic and message_type
        if source_type == 'ROS2_CDR':
            for key in ['ros_topic', 'message_type']:
                if key not in mapping:
                    raise ConfigValidationError(
                        f"Missing required key '{key}' for ROS2_CDR topic mapping {idx}"
                    )
        
        # Validate priority if present
        if 'priority' in mapping:
            valid_priorities = ['HIGH', 'STANDARD', 'LOW']
            if mapping['priority'] not in valid_priorities:
                raise ConfigValidationError(
                    f"Invalid priority '{mapping['priority']}' in topic mapping {idx}. "
                    f"Must be one of {valid_priorities}"
                )
        
        # Validate direction if present
        if 'direction' in mapping:
            valid_directions = ['INBOUND', 'OUTBOUND']
            if mapping['direction'] not in valid_directions:
                raise ConfigValidationError(
                    f"Invalid direction '{mapping['direction']}' in topic mapping {idx}. "
                    f"Must be one of {valid_directions}"
                )

    def get_topic_config(self, ros_topic: str) -> Optional[Dict[str, Any]]:
        """
        Get the configuration for a specific ROS topic.
        
        Args:
            ros_topic: The ROS topic name
            
        Returns:
            Configuration dictionary for the topic, or None if not found
        """
        if not self.config:
            return None
            
        topic_mappings = self.config.get('topic_mappings', [])
        defaults = self.config.get('defaults', {})
        
        for mapping in topic_mappings:
            if mapping.get('source_type', defaults.get('source_type')) == 'ROS2_CDR' and mapping.get('ros_topic') == ros_topic:
                # Apply defaults for missing values
                result = defaults.copy()
                result.update(mapping)
                return result
                
        return None
    
    def get_topic_mappings_by_direction(self, direction: str) -> List[Dict[str, Any]]:
        """
        Get all topic mappings with the specified direction.
        
        Args:
            direction: The direction to filter by ('INBOUND' or 'OUTBOUND')
            
        Returns:
            List of topic mappings with the specified direction
        """
        if not self.config:
            return []
            
        topic_mappings = self.config.get('topic_mappings', [])
        defaults = self.config.get('defaults', {})
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
        
    def get_topic_mappings_by_source_type(self, source_type: str) -> List[Dict[str, Any]]:
        """
        Get all topic mappings with the specified source type.
        
        Args:
            source_type: The source type to filter by ('ROS2_CDR' or 'OPEN_TELEOP')
            
        Returns:
            List of topic mappings with the specified source type
        """
        if not self.config:
            return []
            
        topic_mappings = self.config.get('topic_mappings', [])
        defaults = self.config.get('defaults', {})
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
        
    def get_zmq_config(self) -> Dict[str, Any]:
        """
        Get the ZeroMQ configuration.
        
        Returns:
            Dictionary with ZeroMQ configuration or empty dict if not found
        """
        settings = self.config.get('settings', {})
        zmq_config = {
            'message_buffer_size': settings.get('message_buffer_size', 1000),
            'reconnect_interval_ms': settings.get('reconnect_interval_ms', 1000),
            # Hardcoded values for now, to be replaced with bootstrap config
            'controller_address': 'tcp://localhost:5555',
            'publish_address': 'tcp://*:5556'
        }
        return zmq_config 