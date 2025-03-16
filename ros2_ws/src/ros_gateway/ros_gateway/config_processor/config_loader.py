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
        # Check for required top-level sections
        if not config:
            raise ConfigValidationError("Configuration is empty")
        
        if 'gateway' not in config:
            raise ConfigValidationError("Missing 'gateway' section in configuration")
            
        gateway = config['gateway']
        
        # Check ZMQ configuration
        if 'zmq' not in gateway:
            raise ConfigValidationError("Missing 'zmq' section in gateway configuration")
            
        zmq_config = gateway['zmq']
        required_zmq_keys = ['controller_address', 'publish_address']
        for key in required_zmq_keys:
            if key not in zmq_config:
                raise ConfigValidationError(f"Missing required ZMQ config key: {key}")
        
        # Check topic mappings
        if 'topic_mappings' not in gateway:
            raise ConfigValidationError("Missing 'topic_mappings' section in gateway configuration")
            
        topic_mappings = gateway['topic_mappings']
        if not isinstance(topic_mappings, list):
            raise ConfigValidationError("'topic_mappings' must be a list")
            
        for idx, mapping in enumerate(topic_mappings):
            self._validate_topic_mapping(mapping, idx)
    
    def _validate_topic_mapping(self, mapping: Dict[str, Any], idx: int) -> None:
        """
        Validate a single topic mapping.
        
        Args:
            mapping: The topic mapping dictionary to validate
            idx: The index of the mapping in the list (for error reporting)
            
        Raises:
            ConfigValidationError: If the mapping is invalid
        """
        required_keys = ['ros_topic', 'ott', 'message_type']
        for key in required_keys:
            if key not in mapping:
                raise ConfigValidationError(f"Missing required key '{key}' in topic mapping {idx}")
        
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
            
        topic_mappings = self.config['gateway'].get('topic_mappings', [])
        defaults = self.config['gateway'].get('defaults', {})
        
        for mapping in topic_mappings:
            if mapping['ros_topic'] == ros_topic:
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
            
        topic_mappings = self.config['gateway'].get('topic_mappings', [])
        defaults = self.config['gateway'].get('defaults', {})
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