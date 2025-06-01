"""
Configuration Manager for Media Gateway.

Handles requesting and parsing media stream configuration from the Controller,
using ZeroMQ exactly like ros-gateway does.
"""

import logging
import time
import json
from typing import Optional

from media_gateway.config.internal_config import MediaGatewayInternalConfig
from media_gateway.config.models import MediaConfig
from media_gateway.core.zmq_client import ZmqClient


logger = logging.getLogger(__name__)


class ConfigManager:
    """
    Manages configuration for the Media Gateway.
    
    Requests media stream configuration from the Controller using ZeroMQ
    exactly like ros-gateway does.
    """
    
    def __init__(self, internal_config: MediaGatewayInternalConfig):
        """Initialize ConfigManager with internal configuration."""
        self.internal_config = internal_config
        self.logger = logging.getLogger(__name__)
        
        # Initialize ZMQ client using internal config (same as ros-gateway)
        self.zmq_client = ZmqClient(
            controller_address=internal_config.zmq.controller_address,
            publish_address=internal_config.zmq.publish_address,
            buffer_size=internal_config.zmq.message_buffer_size,
            reconnect_interval_ms=internal_config.zmq.reconnect_interval_ms,
            logger=self.logger
        )
        
    def request_config(self) -> Optional[MediaConfig]:
        """
        Request media stream configuration from the Controller via ZeroMQ.
        
        This follows the EXACT same pattern as ros-gateway.
        
        Returns:
            MediaConfig if successful, None on error
        """
        try:
            self.logger.info("Requesting configuration from controller via ZeroMQ...")
            
            # Request config via ZMQ (same as ros-gateway)
            config_dict = self.zmq_client.request_config()
            
            if config_dict is not None:
                self.logger.info("Successfully received configuration from controller")
                self.logger.debug(f"Received config: {json.dumps(config_dict, indent=2)}")
                
                # Subscribe to configuration updates (same as ros-gateway)
                self.zmq_client.subscribe_to_config_updates(self.handle_config_update)
                
                # Parse the media_mappings section (parallel to topic_mappings)
                media_config = self._parse_media_config(config_dict)
                return media_config
                
            else:
                self.logger.warning("Failed to get configuration from controller, using fallback")
                return self._get_fallback_config()
                
        except Exception as e:
            self.logger.error(f"Failed to request config: {e}")
            return self._get_fallback_config()
            
    def _parse_media_config(self, config_dict: dict) -> MediaConfig:
        """
        Parse media configuration from controller config.
        
        Looks for 'media_mappings' section parallel to 'topic_mappings'.
        """
        # Extract media mappings (parallel to topic_mappings in ros-gateway)
        media_mappings = config_dict.get('media_mappings', {})
        video_streams = media_mappings.get('video', [])
        audio_streams = media_mappings.get('audio', [])
        
        # Apply defaults to streams  
        defaults = config_dict.get('defaults', {})
        
        for stream in video_streams + audio_streams:
            # Apply defaults for missing values (same pattern as ros-gateway)
            for key, value in defaults.items():
                if key not in stream:
                    stream[key] = value
        
        self.logger.info(f"Parsed {len(video_streams)} video streams, {len(audio_streams)} audio streams")
        
        # Create MediaConfig object
        return MediaConfig(video_streams=video_streams, audio_streams=audio_streams)
        
    def _get_fallback_config(self) -> MediaConfig:
        """
        Get fallback configuration when controller request fails.
        
        Returns empty config (same pattern as ros-gateway fallback).
        """
        self.logger.warning("Using fallback media configuration (empty)")
        return MediaConfig(video_streams=[], audio_streams=[])
        
    def handle_config_update(self, message: str):
        """
        Handle configuration updates from the controller.
        
        This follows the EXACT same pattern as ros-gateway.
        
        Args:
            message: The configuration update message (JSON string) from ZMQ SUB socket
        """
        try:
            data = json.loads(message)
            msg_type = data.get('type')
            topic = data.get('topic')
            
            if msg_type == 'CONFIG_UPDATED' or (topic and topic.startswith('configuration.notification')):
                self.logger.info("Received notification that configuration has changed on controller.")
                self.logger.info("Requesting full updated configuration...")
                
                # Request new configuration (same as ros-gateway)
                new_config_dict = self.zmq_client.request_config()
                
                if new_config_dict is not None:
                    self.logger.info("Successfully received updated configuration from controller.")
                    
                    # Parse and apply new configuration
                    new_media_config = self._parse_media_config(new_config_dict)
                    self._apply_new_configuration(new_media_config)
                    
                else:
                    self.logger.error("Failed to retrieve updated configuration after notification.")
                    
            elif msg_type == 'CONFIG_RESPONSE' or (topic and topic.startswith('configuration.update')):
                self.logger.info("Received full configuration update directly via ZMQ publish.")
                new_config_dict = data.get('data')
                if new_config_dict:
                    new_media_config = self._parse_media_config(new_config_dict)
                    self._apply_new_configuration(new_media_config)
                else:
                    self.logger.warning("Received CONFIG_RESPONSE/update message but no 'data' field found.")
                    
            else:
                self.logger.warning(f"Received unexpected message type '{msg_type}' on config subscription topic '{topic}'. Ignoring.")
                
        except json.JSONDecodeError as e:
            self.logger.error(f"Error decoding JSON from ZMQ config update message: {e}")
        except Exception as e:
            self.logger.error(f"Error processing configuration update: {e}")
            
    def _apply_new_configuration(self, new_media_config: MediaConfig):
        """
        Apply new media configuration.
        
        TODO: Implement dynamic stream reconfiguration
        """
        self.logger.info("Applying new media configuration...")
        # TODO: Update stream manager, pipeline manager, etc.
        # This will be implemented when we add dynamic reconfiguration
        
    def poll_for_updates(self) -> bool:
        """
        Poll for configuration updates.
        
        Returns:
            True if configuration was updated, False otherwise
        """
        # Configuration updates are handled via ZMQ subscriptions
        # This method is kept for compatibility but not used
        return False 