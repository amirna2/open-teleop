#!/usr/bin/env python3
"""
Media Gateway - Main Entry Point

Follows the EXACT same pattern as ros-gateway/gateway_node.py
"""

import os
import sys
import argparse
import yaml
import json
import time
import logging
from pathlib import Path

from media_gateway.zeromq_client.zmq_client import ZmqClient


def load_config_from_yaml(config_path, logger):
    """Load configuration from a YAML file. (Same as ros-gateway)"""
    logger.info(f"Attempting to load configuration from: {config_path}")
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            if config:
                logger.info(f"Successfully loaded configuration from {config_path}")
                return config
            else:
                logger.warning(f"Configuration file {config_path} is empty.")
                return {}
    except FileNotFoundError:
        logger.error(f"Configuration file not found: {config_path}")
        return {}
    except yaml.YAMLError as e:
        logger.error(f"Error parsing configuration file {config_path}: {e}")
        return {}
    except Exception as e:
        logger.error(f"Unexpected error loading configuration file {config_path}: {e}")
        return {}


def setup_logging(bootstrap_config):
    """Setup logging based on bootstrap config. (Same pattern as ros-gateway)"""
    logging_config = bootstrap_config.get('media_gateway', {}).get('logging', {})
    
    log_level = getattr(logging, logging_config.get('level', 'INFO').upper())
    log_to_file = logging_config.get('log_to_file', True)
    log_path = logging_config.get('log_path', '/tmp/open_teleop_logs')
    
    if log_to_file:
        log_dir = Path(log_path)
        log_dir.mkdir(parents=True, exist_ok=True)
        
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_dir / "media-gateway.log"),
                logging.StreamHandler(sys.stdout)
            ]
        )
    else:
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[logging.StreamHandler(sys.stdout)]
        )


class MediaGateway:
    """
    Media Gateway for Open-Teleop.
    
    Follows the EXACT same pattern as ros-gateway/gateway_node.py
    """
    
    def __init__(self, config_path):
        self.logger = logging.getLogger(__name__)
        
        # Load bootstrap configuration from YAML file (same as ros-gateway)
        self.bootstrap_config = load_config_from_yaml(config_path, self.logger)
        
        # Configure logging based on bootstrap config
        setup_logging(self.bootstrap_config)
        
        self.logger.info('='*80)
        self.logger.info('Hello, I am the Media Gateway for Open-Teleop!')
        self.logger.info('='*80)
        self.logger.info('Starting Media Gateway for Open-Teleop')
        self.logger.info(f'Using bootstrap config file: {config_path}')
        
        # Get ZeroMQ configuration (ENV > YAML > Default) - same as ros-gateway
        zmq_config = self.bootstrap_config.get('media_gateway', {}).get('zmq', {})
        controller_address = os.environ.get('TELEOP_ZMQ_CONTROLLER_ADDRESS') or zmq_config.get('controller_address', 'tcp://localhost:5555')
        publish_address = os.environ.get('TELEOP_ZMQ_PUBLISH_ADDRESS') or zmq_config.get('publish_address', 'tcp://localhost:5556')
        message_buffer_size = int(os.environ.get('TELEOP_ZMQ_BUFFER_SIZE') or zmq_config.get('message_buffer_size', 1000))
        reconnect_interval_ms = int(os.environ.get('TELEOP_ZMQ_RECONNECT_INTERVAL_MS') or zmq_config.get('reconnect_interval_ms', 1000))
        
        self.logger.info(f"Using ZMQ controller address: {controller_address}")
        self.logger.info(f"Using ZMQ publish address: {publish_address}")
        
        # Initialize the ZeroMQ client (same as ros-gateway)
        self.zmq_client = ZmqClient(
            controller_address=controller_address,
            publish_address=publish_address,
            buffer_size=message_buffer_size,
            reconnect_interval_ms=reconnect_interval_ms,
            logger=self.logger
        )
        
        # Request operational configuration from the controller (same as ros-gateway)
        self.config = self.request_configuration()
        
        # Check if fallback was used (same as ros-gateway)
        if self.config.get("config_id") == "default":
            self.logger.warning("Operational config fetched using fallback mechanism.")
        else:
            self.logger.info(f"Loaded operational configuration version: {self.config.get('version')}")
            self.logger.info(f"Config ID: {self.config.get('config_id')}")
            self.logger.info(f"Robot ID: {self.config.get('robot_id')}")
            
            # Log media mappings (parallel to topic_mappings in ros-gateway)
            media_mappings = self.config.get('media_mappings', {})
            video_streams = media_mappings.get('video', [])
            audio_streams = media_mappings.get('audio', [])
            
            self.logger.info(f"Found {len(video_streams)} video streams and {len(audio_streams)} audio streams")
            
            for i, stream in enumerate(video_streams):
                self.logger.debug(f"Video stream {i+1}: {json.dumps(stream)}")
            for i, stream in enumerate(audio_streams):
                self.logger.debug(f"Audio stream {i+1}: {json.dumps(stream)}")
        
        # TODO: Initialize other components (device manager, pipeline manager, etc.)
        # For now, just log that we're ready
        self.logger.info('Media Gateway components initialized.')
        
    def request_configuration(self):
        """
        Request configuration from the controller. (Same as ros-gateway)
        
        Returns:
            Dict containing the configuration received from the controller
        """
        self.logger.info("Requesting configuration from controller...")
        
        # Try to get configuration directly from the controller using our ZmqClient
        config = self.zmq_client.request_config()
        
        if config is not None:
            self.logger.info("Successfully received configuration from controller")
            self.logger.debug(f"DEBUG: Received config from controller: {json.dumps(config, indent=2)}")
            
            # Subscribe to configuration updates (same as ros-gateway)
            self.zmq_client.subscribe_to_config_updates(self.handle_config_update)
            
            return config
        
        # Fallback to local configuration if controller request fails (same as ros-gateway)
        self.logger.warning("Failed to get configuration from controller, using default configuration")
        
        # Use empty default configuration as fallback
        return {
            "version": "1.0",
            "config_id": "default",
            "lastUpdated": time.time(),
            "robot_id": os.environ.get('TELEOP_ROBOT_ID', 'unknown'),
            "media_mappings": {"video": [], "audio": []},
            "defaults": {
                "priority": "STANDARD",
                "encoding_format": "video/h264"
            }
        }
        
    def handle_config_update(self, message):
        """
        Handle configuration updates from the controller. (Same as ros-gateway)
        
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
                
                # Proactively request the new configuration
                new_config = self.zmq_client.request_config()
                
                if new_config is not None:
                    self.logger.info("Successfully received updated configuration from controller.")
                    
                    # Apply the new configuration dynamically
                    self.apply_new_configuration(new_config)
                    
                else:
                    self.logger.error("Failed to retrieve updated configuration after notification.")
            
            elif msg_type == 'CONFIG_RESPONSE' or (topic and topic.startswith('configuration.update')):
                self.logger.info("Received full configuration update directly via ZMQ publish.")
                new_config = data.get('data')
                if new_config:
                    self.apply_new_configuration(new_config)
                else:
                    self.logger.warning("Received CONFIG_RESPONSE/update message but no 'data' field found.")
                    
            else:
                self.logger.warning(f"Received unexpected message type '{msg_type}' on config subscription topic '{topic}'. Ignoring.")
                
        except json.JSONDecodeError as e:
            self.logger.error(f"Error decoding JSON from ZMQ config update message: {e}")
        except Exception as e:
            self.logger.error(f"Error processing configuration update: {e}")
            
    def apply_new_configuration(self, new_config):
        """Apply the received configuration to relevant components. (Same pattern as ros-gateway)"""
        self.logger.info(f"Applying new configuration (ID: {new_config.get('config_id', 'N/A')}, Version: {new_config.get('version', 'N/A')})")
        
        # Update the gateway's stored configuration
        self.config = new_config
        self.logger.info("Gateway configuration state updated.")
        
        # TODO: Update media components (device manager, pipeline manager, etc.)
        
    def run(self):
        """Main run loop for the Media Gateway. (Same pattern as ros-gateway)"""
        try:
            self.logger.info("Media Gateway ready - entering main loop...")
            
            # Main processing loop
            while True:
                time.sleep(1)
                # TODO: Add periodic health checks, stream monitoring, etc.
                
        except KeyboardInterrupt:
            self.logger.info("Shutdown requested by user")
        except Exception as e:
            self.logger.error(f"Fatal error: {e}")
            raise
        finally:
            self.shutdown()
            
    def shutdown(self):
        """Shutdown the Media Gateway gracefully."""
        self.logger.info("Shutting down Media Gateway...")
        
        if hasattr(self, 'zmq_client') and self.zmq_client:
            self.zmq_client.shutdown()
            
        self.logger.info("Media Gateway shutdown complete")


def main():
    """Main entry point. (Same pattern as ros-gateway)"""
    parser = argparse.ArgumentParser(description='Open Teleop Media Gateway')
    parser.add_argument(
        '--config-path', 
        type=str, 
        required=True,
        help='Path to the gateway bootstrap configuration file (YAML)'
    )
    
    # Parse only known arguments, allowing other arguments to pass through
    parsed_args, _ = parser.parse_known_args()
    
    # Check if config file exists before trying to init gateway
    if not os.path.exists(parsed_args.config_path):
        print(f"Bootstrap configuration file not found: {parsed_args.config_path}")
        sys.exit(1)
        
    try:
        gateway = MediaGateway(config_path=parsed_args.config_path)
        gateway.run()
        
    except (KeyboardInterrupt, Exception) as e:
        if isinstance(e, KeyboardInterrupt):
            print("Shutdown signal received.")
        else:
            print(f"Fatal error during gateway execution: {e}")
            
    print("Media Gateway process finished.")


if __name__ == '__main__':
    main() 