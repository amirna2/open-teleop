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
import asyncio
import logging
from pathlib import Path

from .zeromq_client.zmq_client import ZmqClient
from .devices.manager import DeviceManager
from .pipelines.manager import PipelineManager


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
        
        # Initialize device manager for hardware discovery
        self.device_manager = DeviceManager()
        
        # Initialize pipeline manager for GStreamer pipelines
        self.pipeline_manager = None
        
        # Device discovery state
        self._last_device_state = None
        self._discovery_task = None
        
        # Store reference to main event loop for cross-thread async calls
        self._main_loop = None
        
        # Store previous config for change detection
        self.previous_config = None
        
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
        
        # Initialize pipeline manager
        self.pipeline_manager = PipelineManager(
            device_manager=self.device_manager,
            zmq_client=self.zmq_client,
            logger=self.logger
        )
        
        self.logger.info('Media Gateway components initialized.')
        
    async def initialize_devices(self):
        """Initialize and discover hardware devices."""
        self.logger.info("Initializing device discovery...")
        await self.device_manager.discover_devices()
        
        # Log discovered devices
        sources = self.device_manager.get_available_sources()
        video_sources = sources.get('video_sources', [])
        audio_sources = sources.get('audio_sources', [])
        
        self.logger.info(f"Device discovery completed: {len(video_sources)} video devices, {len(audio_sources)} audio devices")
        
        for device in video_sources:
            self.logger.info(f"Video: {device['name']} ({device['device_id']}) - {len(device['resolutions'])} resolutions")
            
        for device in audio_sources:
            self.logger.info(f"Audio: {device['name']} ({device['device_id']})")
            
        # Print summary for easy visibility
        print("\n" + "="*60)
        print("DISCOVERED DEVICES")
        print("="*60)
        print(json.dumps(sources, indent=2))
        print("="*60)
        
        # Start device discovery monitoring after initial discovery
        self._start_device_discovery_monitoring()
        
        return sources
    
    async def start_pipeline_manager(self):
        """Start the pipeline manager and create initial streams."""
        if not self.pipeline_manager:
            self.logger.error("Pipeline manager not initialized")
            return
            
        self.logger.info("Starting pipeline manager...")
        
        # Start the pipeline manager
        await self.pipeline_manager.start_manager()
        
        # Create streams from configuration media mappings
        media_mappings = self.config.get('media_mappings', {})
        if media_mappings:
            await self.pipeline_manager.create_streams_from_config(media_mappings)
        else:
            self.logger.info("No media mappings found in configuration")
            
        self.logger.info("Pipeline manager started successfully")

    def _start_device_discovery_monitoring(self):
        """Start monitoring for device changes and publish updates."""
        discovery_config = self.config.get('media_gateway', {}).get('device_discovery', {})
        
        if not discovery_config.get('enabled', True):
            self.logger.info("Device discovery publishing is disabled")
            return
            
        interval_seconds = discovery_config.get('interval_seconds', 10)
        self.logger.info(f"Starting device discovery monitoring every {interval_seconds} seconds")
        
        # Start the monitoring task
        self._discovery_task = asyncio.create_task(self._discovery_monitor_loop(interval_seconds))
        
    async def _discovery_monitor_loop(self, interval_seconds):
        """Monitor devices for changes and publish updates."""
        while True:
            try:
                await asyncio.sleep(interval_seconds)
                await self._check_and_publish_device_changes()
            except asyncio.CancelledError:
                self.logger.info("Device discovery monitoring cancelled")
                break
            except Exception as e:
                self.logger.error(f"Error in device discovery monitoring: {e}")
                
    async def _check_and_publish_device_changes(self):
        """Check for device changes and publish if changed."""
        try:
            # Re-discover devices
            await self.device_manager.discover_devices()
            current_sources = self.device_manager.get_available_sources()
            
            # Create device state hash for change detection
            current_state = self._create_device_state_hash(current_sources)
            
            # Check if devices changed
            if self._last_device_state != current_state:
                self.logger.info("Device changes detected, publishing update")
                await self._publish_device_discovery(current_sources)
                self._last_device_state = current_state
            else:
                self.logger.debug("No device changes detected")
                
        except Exception as e:
            self.logger.error(f"Error checking device changes: {e}")
            
    def _create_device_state_hash(self, sources):
        """Create a hash representing the current device state."""
        import hashlib
        
        # Create a sorted, deterministic representation
        video_devices = sorted([
            (d['device_id'], d['name'], len(d['resolutions']))
            for d in sources.get('video_sources', [])
        ])
        
        audio_devices = sorted([
            (d['device_id'], d['name'])
            for d in sources.get('audio_sources', [])
        ])
        
        state_str = json.dumps({'video': video_devices, 'audio': audio_devices}, sort_keys=True)
        return hashlib.sha256(state_str.encode()).hexdigest()
        
    async def _publish_device_discovery(self, sources):
        """Publish device discovery message to ZeroMQ topic."""
        discovery_config = self.config.get('media_gateway', {}).get('device_discovery', {})
        
        # Build discovery message
        message = {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "video_sources": sources.get('video_sources', []),
            "audio_sources": sources.get('audio_sources', [])
        }
        
        # Get topic configuration
        topic = discovery_config.get('topic', 'teleop.media.available_sources')
        topic_id = discovery_config.get('topic_id', 'f0000001-aaaa-bbbb-cccc-d701748f0010')
        direction = discovery_config.get('direction', 'OUTBOUND')
        priority = discovery_config.get('priority', 'LOW')
        
        self.logger.info(f"Publishing device discovery to topic: {topic}")
        self.logger.debug(f"Discovery message: {json.dumps(message, indent=2)}")
        
        # Create OTT message
        ott_message = {
            'version': 1,
            'ott': topic,
            'topic_id': topic_id,
            'direction': direction,
            'priority': priority,
            'timestamp_ns': int(time.time() * 1_000_000_000),
            'content_type': 'application/json',
            'payload': json.dumps(message).encode('utf-8')
        }
        
        # Publish via ZMQ
        if hasattr(self, 'zmq_client') and self.zmq_client:
            success = self.zmq_client.publish_message(topic, "DEVICE_DISCOVERY", message)
            if success:
                self.logger.info("Successfully published device discovery message")
            else:
                self.logger.warning("Failed to publish device discovery message")
        else:
            self.logger.warning("ZMQ client not available, cannot publish discovery message")
        
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
        
        # Check if media mappings changed before updating streams
        media_config_changed = self.has_media_config_changed(self.config, new_config)
        
        # Update the gateway's stored configuration
        self.previous_config = self.config.copy() if self.config else None
        self.config = new_config
        self.logger.info("Gateway configuration state updated.")
        
        # Update pipeline manager only if media mappings actually changed
        if media_config_changed:
            if self.pipeline_manager and self._main_loop:
                try:
                    # Schedule the coroutine to run in the main event loop thread
                    future = asyncio.run_coroutine_threadsafe(
                        self.pipeline_manager.update_streams(new_config), 
                        self._main_loop
                    )
                    self.logger.info("Successfully scheduled pipeline stream update")
                except Exception as e:
                    self.logger.error(f"Error scheduling pipeline stream update: {e}")
            elif self.pipeline_manager:
                self.logger.warning("Pipeline manager available but no main event loop reference")
            else:
                self.logger.debug("No pipeline manager to update")
        else:
            self.logger.info("Media mappings unchanged, skipping pipeline stream update")
        

            
    def has_media_config_changed(self, old_config, new_config):
        """Check if media mappings configuration has changed."""
        if old_config is None:
            # First time configuration, always update
            return True
            
        # Extract media mappings from both configs
        old_media_mappings = old_config.get('media_mappings', {})
        new_media_mappings = new_config.get('media_mappings', {})
        
        # Compare video mappings
        old_video = old_media_mappings.get('video', [])
        new_video = new_media_mappings.get('video', [])
        
        if len(old_video) != len(new_video):
            self.logger.info(f"Video mapping count changed: {len(old_video)} -> {len(new_video)}")
            return True
            
        # Create lookup dictionaries by device_id for video comparison
        old_video_by_device = {mapping.get('device_id'): mapping for mapping in old_video}
        new_video_by_device = {mapping.get('device_id'): mapping for mapping in new_video}
        
        # Check if any video device IDs changed
        if set(old_video_by_device.keys()) != set(new_video_by_device.keys()):
            self.logger.info("Video mapping device IDs changed")
            return True
            
        # Check if any video mapping parameters changed
        for device_id in old_video_by_device:
            old_mapping = old_video_by_device[device_id]
            new_mapping = new_video_by_device[device_id]
            
            # Compare relevant fields for video mappings
            fields_to_compare = ['ott', 'encoding_format', 'encoder_params', 'priority', 'direction']
            for field in fields_to_compare:
                if old_mapping.get(field) != new_mapping.get(field):
                    self.logger.info(f"Video mapping {device_id} field '{field}' changed: {old_mapping.get(field)} -> {new_mapping.get(field)}")
                    return True
        
        # Compare audio mappings
        old_audio = old_media_mappings.get('audio', [])
        new_audio = new_media_mappings.get('audio', [])
        
        if len(old_audio) != len(new_audio):
            self.logger.info(f"Audio mapping count changed: {len(old_audio)} -> {len(new_audio)}")
            return True
            
        # Create lookup dictionaries by device_id for audio comparison
        old_audio_by_device = {mapping.get('device_id'): mapping for mapping in old_audio}
        new_audio_by_device = {mapping.get('device_id'): mapping for mapping in new_audio}
        
        # Check if any audio device IDs changed
        if set(old_audio_by_device.keys()) != set(new_audio_by_device.keys()):
            self.logger.info("Audio mapping device IDs changed")
            return True
            
        # Check if any audio mapping parameters changed
        for device_id in old_audio_by_device:
            old_mapping = old_audio_by_device[device_id]
            new_mapping = new_audio_by_device[device_id]
            
            # Compare relevant fields for audio mappings
            fields_to_compare = ['ott', 'encoding_format', 'encoder_params', 'priority', 'direction']
            for field in fields_to_compare:
                if old_mapping.get(field) != new_mapping.get(field):
                    self.logger.info(f"Audio mapping {device_id} field '{field}' changed: {old_mapping.get(field)} -> {new_mapping.get(field)}")
                    return True
                    
        self.logger.debug("No media mapping changes detected")
        return False

    def shutdown(self):
        """Shutdown the Media Gateway gracefully. (Same as ros-gateway)"""
        self.logger.info("Shutting down Media Gateway...")
        
        # Stop device discovery monitoring
        if self._discovery_task and not self._discovery_task.done():
            self._discovery_task.cancel()
            self.logger.info("Cancelled device discovery monitoring task")
        
        if hasattr(self, 'zmq_client'):
            self.zmq_client.shutdown()
            
        self.logger.info("Media Gateway shutdown complete.")


async def async_main():
    """Async main function for the Media Gateway."""
    parser = argparse.ArgumentParser(description='Media Gateway for Open-Teleop')
    parser.add_argument('--config-path', type=str, 
                       default='/home/amir/projects/open-teleop/media-gateway/config/media-gateway-config.yaml',
                       help='Path to the configuration file')
    
    args = parser.parse_args()
    
    gateway = None
    try:
        # Initialize gateway
        gateway = MediaGateway(args.config_path)
        
        # Store the current event loop for cross-thread async calls
        gateway._main_loop = asyncio.get_running_loop()
        
        # Initialize devices
        await gateway.initialize_devices()
        
        # Start pipeline manager and create streams
        await gateway.start_pipeline_manager()
        
        # Keep running (async version of main loop)
        gateway.logger.info("Media Gateway is running. Press Ctrl+C to stop.")
        while True:
            await asyncio.sleep(1.0)
        
    except KeyboardInterrupt:
        print("\nReceived interrupt signal, shutting down...")
        if gateway:
            if gateway.pipeline_manager:
                await gateway.pipeline_manager.stop_manager()
            gateway.shutdown()
        return 0
    except Exception as e:
        print(f"Fatal error: {e}")
        if gateway:
            if gateway.pipeline_manager:
                await gateway.pipeline_manager.stop_manager()
            gateway.shutdown()
        return 1
        
    return 0

def main():
    """
    Main entry point for the Media Gateway.
    """
    return asyncio.run(async_main())


if __name__ == '__main__':
    sys.exit(main()) 