#!/usr/bin/env python3
# gateway_node.py - Main ROS Gateway node for Open-Teleop

import rclpy
from rclpy.node import Node
import os
import sys
import signal
import yaml
import json
import time
import argparse
import asyncio # Keep asyncio import needed for the class method
import threading # Might need for future handling, keep import
from rclpy.executors import ExternalShutdownException # For cleaner shutdown

# Import custom logger
import open_teleop_logger as log

# Import our gateway components
from ros_gateway.config_processor.config_loader import ConfigLoader
from ros_gateway.zeromq_client.zmq_client import ZmqClient
from ros_gateway.topic_subscriber.topic_manager import TopicManager
from ros_gateway.message_converter.converter import MessageConverter
from ros_gateway.topic_publisher.publisher import TopicPublisher
# Defer AV import to where it's used to avoid potential init order issues
# from ros_gateway.av_integration import EncodedFrameSubscriber

def load_config_from_yaml(config_path, logger):
    """Load configuration from a YAML file."""
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

def get_config_value(env_var, config_dict, default_value):
    """Get config value with precedence: ENV > YAML > Default."""
    value = os.environ.get(env_var)
    if value is not None:
        return value
    if config_dict is not None:
        # Assume config_dict keys match env var names for simplicity, or adjust logic
        # Example: if env_var is TELEOP_ZMQ_CONTROLLER_ADDRESS, look for 'gateway.zmq.controller_address'
        # This part needs refinement based on YAML structure
        pass # Placeholder - Need actual key mapping
    return default_value

class RosGateway(Node):
    """
    ROS Gateway for Open-Teleop.
    
    This node serves as the bridge between ROS2 and the Open-Teleop Controller, managing:
    - Dynamic topic subscriptions based on configuration
    - Message conversion between ROS and OttMessage format
    - ZeroMQ communication with the controller
    - Command reception and publishing to ROS
    """
    
    def __init__(self, config_path):
        super().__init__('ros_gateway')
        
        # Determine project root path (adjust levels if needed)
        self.project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../'))
        
        # Basic logger setup first, will be reconfigured later if possible
        self.logger = log.get_logger(
            name=f"{self.get_name()}",
            log_dir="/tmp/open_teleop_logs", # Temporary initial path
            console_level=log.DEBUG,
            file_level=log.DEBUG
        )
        
        # Load bootstrap configuration from YAML file
        self.bootstrap_config = load_config_from_yaml(config_path, self.logger)
        
        # Configure logging based on bootstrap config (or keep initial if fails)
        self._configure_logging()

        # Display welcome message
        self.logger.info('='*80)
        self.logger.info('Hello, I am the ROS Gateway for Open-Teleop!')
        self.logger.info('This is a minimal implementation with stub components.')
        self.logger.info('='*80)
        self.logger.info('Starting ROS Gateway for Open-Teleop')
        self.logger.info(f'Project root: {self.project_root}')
        self.logger.info(f'Using bootstrap config file: {config_path}')

        # Initialize the message converter
        self.converter = MessageConverter(node=self, logger=self.logger)

        # Get ZeroMQ configuration (ENV > YAML > Default)
        zmq_config = self.bootstrap_config.get('gateway', {}).get('zmq', {})
        controller_address = os.environ.get('TELEOP_ZMQ_CONTROLLER_ADDRESS') or zmq_config.get('controller_address', 'tcp://localhost:5555')
        publish_address = os.environ.get('TELEOP_ZMQ_PUBLISH_ADDRESS') or zmq_config.get('publish_address', 'tcp://localhost:5556')
        message_buffer_size = int(os.environ.get('TELEOP_ZMQ_BUFFER_SIZE') or zmq_config.get('message_buffer_size', 1000))
        reconnect_interval_ms = int(os.environ.get('TELEOP_ZMQ_RECONNECT_INTERVAL_MS') or zmq_config.get('reconnect_interval_ms', 1000))

        self.logger.info(f"Using ZMQ controller address: {controller_address}")
        self.logger.info(f"Using ZMQ publish address: {publish_address}")

        # Initialize the ZeroMQ client early with bootstrap configuration
        self.zmq_client = ZmqClient(
            controller_address=controller_address,
            publish_address=publish_address,
            buffer_size=message_buffer_size,
            reconnect_interval_ms=reconnect_interval_ms,
            logger=self.logger
        )

        # Request operational configuration from the controller
        self.config = self.request_configuration()
        
        # Check if fallback was used
        if self.config.get("config_id") == "default":
             self.logger.warning("Operational config fetched using fallback mechanism.")
        else:
            self.logger.info(f"Loaded operational configuration version: {self.config.get('version')}")
            self.logger.info(f"Config ID: {self.config.get('config_id')}")
            self.logger.info(f"Robot ID: {self.config.get('robot_id')}")

            # Log topic mappings at DEBUG level
            topic_mappings = self.config.get('topic_mappings', [])
            for i, mapping in enumerate(topic_mappings):
                self.logger.debug(f"Topic mapping {i+1}: {json.dumps(mapping)}")
            
            # Get defaults
            defaults = self.config.get('defaults', {})
            self.logger.debug(f"Default values: {json.dumps(defaults)}")
            
            # Extract topic lists for logging (keep summary at INFO)
            ros2_topics = [m.get('ros_topic') for m in topic_mappings]
            ott_topics = [m.get('ott') for m in topic_mappings]
            self.logger.info(f"Found {len(ros2_topics)} ROS2 topics and {len(ott_topics)} Open-Teleop topics")
            
            # Filter topic mappings by direction (keep summary at INFO)
            inbound_topics = self.filter_topic_mappings_by_direction(topic_mappings, defaults, 'INBOUND')
            outbound_topics = self.filter_topic_mappings_by_direction(topic_mappings, defaults, 'OUTBOUND')
            self.logger.info(f"Found {len(inbound_topics)} inbound topics and {len(outbound_topics)} outbound topics")
            
            # Log detailed information about inbound topics at DEBUG
            for i, mapping in enumerate(inbound_topics):
                self.logger.debug(f"Inbound topic {i+1}: {mapping['ott']} -> {mapping['ros_topic']}")

        # Initialize the topic manager (pass initial config)
        outbound_mappings = self.filter_topic_mappings_by_direction(self.config.get('topic_mappings', []), self.config.get('defaults', {}), 'OUTBOUND') # <-- Fix from previous step
        self.topic_manager = TopicManager(
            node=self,
            topic_mappings=outbound_mappings, # Initial filtered mappings
            defaults=self.config.get('defaults', {}),
            message_converter=self.converter,
            zmq_client=self.zmq_client,
            logger=self.logger
        )
        
        # Initialize the topic publisher (pass initial config)
        inbound_mappings = self.filter_topic_mappings_by_direction(self.config.get('topic_mappings', []), self.config.get('defaults', {}), 'INBOUND')
        self.topic_publisher = TopicPublisher(
            node=self,
            topic_mappings=inbound_mappings, # Initial filtered mappings
            defaults=self.config.get('defaults', {}),
            message_converter=self.converter,
            zmq_client=self.zmq_client,
            logger=self.logger
        )
        
        # Initialize AV subscriber attribute (will be created later)
        self.encoded_frame_subscriber = None 
        self.av_init_task = None # To hold the reference to the running async task
        
        # Setup a timer for heartbeat/monitoring
        self.create_timer(1.0, self.heartbeat_callback)
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Create a timer to trigger AV init once the node is spinning
        self.init_timer = self.create_timer(0.1, self.schedule_av_initialization)
        
        self.logger.info('ROS Gateway components base initialized.')
        self.logger.info('AV Initialization timer scheduled.') # New log
    
    def schedule_av_initialization(self):
        """
        Timer callback to schedule the asynchronous AV initialization.
        This runs once after the node starts spinning.
        """
        if self.init_timer is not None:
            self.logger.info("One-shot timer triggered for AV initialization.")
            self.init_timer.cancel()
            self.init_timer = None
        else:
            # Avoid re-scheduling if called again (e.g., during config update)
            self.logger.debug("AV initialization already scheduled or timer cancelled.")
            return

        # Ensure we have an executor associated with the node
        if self.executor is None:
            self.logger.error("Node executor not available, cannot schedule AV initialization!")
            # This might happen if the node hasn't been added to an executor yet
            # which is unlikely if this timer callback is firing.
            # Try to get it implicitly? Risky.
            # For now, just log the error.
            return

        self.logger.info("Scheduling async initialize_av_integration task...")
        # Schedule the coroutine to run via the node's executor
        # Note: Use self.executor which rclpy.spin() uses internally
        # We store the task future to potentially check its status or cancel it later
        self.av_init_task = self.executor.create_task(self.initialize_av_integration())
        # Alternatively, if create_task isn't directly available or suitable on the
        # implicit executor used by rclpy.spin(), consider adding a future completion callback:
        # future = self.executor.submit(self.initialize_av_integration) # Requires initialize_av_integration NOT to be async
        # future.add_done_callback(self._handle_av_init_completion)

    def _configure_logging(self):
        """Configure the logger based on the bootstrap configuration file."""
        try:
            log_config = self.bootstrap_config.get('gateway', {}).get('logging', {})
            log_level_str = os.environ.get('TELEOP_LOG_LEVEL') or log_config.get('level', 'INFO')
            log_to_file = os.environ.get('TELEOP_LOG_TO_FILE') 
            if log_to_file is None:
                log_to_file = log_config.get('log_to_file', False)
            else:
                log_to_file = log_to_file.lower() in ['true', '1', 'yes']
                
            log_path = os.environ.get('TELEOP_LOG_PATH') or log_config.get('log_path', '/tmp/open_teleop_logs') # Default to /tmp if not set
            log_rotation = int(os.environ.get('TELEOP_LOG_ROTATION_DAYS') or log_config.get('log_rotation_days', 7))

            # Map string level to logging level constant
            log_level = getattr(log, log_level_str.upper(), log.INFO)

            # Reconfigure the logger by getting a new instance with updated settings
            self.logger = log.get_logger(
                name=self.logger.name, # Keep the original name
                log_dir=log_path if log_to_file else None,
                console_level=log_level,
                file_level=log_level if log_to_file else None
            )
            self.logger.info(f"Logger reconfigured: Level={log_level_str}, FileOutput={log_to_file}, Path={log_path if log_to_file else 'N/A'}")

        except Exception as e:
            self.logger.error(f"Error configuring logger from bootstrap config: {e}. Using initial basic logger.")
    
    def heartbeat_callback(self):
        """Periodic heartbeat to monitor system status"""
        #self.logger.debug('Gateway heartbeat')
        x = 1
        # Could add metrics reporting here
    
    def signal_handler(self, sig, frame):
        """Handle shutdown signals gracefully"""
        self.logger.info(f'Received signal {sig}, shutting down...')
        # The shutdown logic will be handled by the finally block in main
        # when using an explicit executor. We just need to trigger the shutdown.
        # If using rclpy.spin(), this might need to call self.shutdown() directly.
        # For now, assume the executor handles graceful exit.
        pass # Let the executor handle shutdown via KeyboardInterrupt/ExternalShutdownException

    def shutdown(self):
        """Clean shutdown of all resources (called from main's finally block)."""
        self.logger.info('Shutting down ROS Gateway components...')
        if hasattr(self, 'topic_manager'):
            self.topic_manager.shutdown()
        if hasattr(self, 'zmq_client'):
            self.zmq_client.shutdown()
        if hasattr(self, 'topic_publisher'):
            self.topic_publisher.shutdown()
        if hasattr(self, 'encoded_frame_subscriber') and self.encoded_frame_subscriber:
            self.encoded_frame_subscriber.shutdown()
        # Cancel init timer if it's still active during shutdown
        if hasattr(self, 'init_timer') and self.init_timer and not self.init_timer.cancelled():
            self.init_timer.cancel()
            self.logger.debug("Cancelled AV init timer during shutdown.")
        # Cancel the async task if it's still running?
        if hasattr(self, 'av_init_task') and self.av_init_task and not self.av_init_task.done():
             self.logger.info("Cancelling potentially running AV initialization task...")
             self.av_init_task.cancel()
             # TODO: Optionally await cancellation if needed, though risky during shutdown

        self.logger.info('Component shutdown complete.')
    
    def request_configuration(self):
        """
        Request configuration from the controller.
        
        Returns:
            Dict containing the configuration received from the controller
        """
        self.logger.info("Requesting configuration from controller...")
        
        # Try to get configuration directly from the controller using our ZmqClient
        config = self.zmq_client.request_config()
        
        if config is not None:
            self.logger.info("Successfully received configuration from controller")
            self.logger.debug(f"DEBUG: Received config from controller: {json.dumps(config, indent=2)}")
            
            # Subscribe to configuration updates
            self.zmq_client.subscribe_to_config_updates(self.handle_config_update)
            
            return config
        
        # Fallback to local configuration if controller request fails
        self.logger.warning("Failed to get configuration from controller, using default configuration")
        
        # Use empty default configuration as fallback
        return {
            "version": "1.0",
            "config_id": "default",
            "lastUpdated": time.time(),
            "robot_id": os.environ.get('TELEOP_ROBOT_ID', 'unknown'),
            "topic_mappings": [],
            "defaults": {
                "priority": "STANDARD",
                "direction": "OUTBOUND",
                "source_type": "ROS2_CDR"
            },
            "settings": {
                "message_buffer_size": 1000,
                "reconnect_interval_ms": 1000
            }
        }
    
    def handle_config_update(self, message):
        """
        Handle configuration updates from the controller.
        This is the callback registered with zmq_client.subscribe_to_config_updates.
        
        Args:
            message: The configuration update message (JSON string) from ZMQ SUB socket
        """
        try:
            # The message might be the full config or just a notification
            data = json.loads(message)
            msg_type = data.get('type')
            topic = data.get('topic') # ZMQ SUB socket includes the topic
            
            # We expect notifications on 'configuration.notification'
            # or potentially full updates on 'configuration.update'
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
                 # Handle case where controller pushes the full config directly (optional)
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
             self.logger.error(f"Raw message: {message[:200]}...") # Log raw message for debugging
        except Exception as e:
            self.logger.error(f"Error processing configuration update: {e}")
            import traceback
            self.logger.error(traceback.format_exc())

    def apply_new_configuration(self, new_config):
        """Applies the received configuration to relevant components."""
        self.logger.info(f"Applying new configuration (ID: {new_config.get('config_id', 'N/A')}, Version: {new_config.get('version', 'N/A')})" )
        
        # Extract mappings and defaults
        new_topic_mappings = new_config.get('topic_mappings', [])
        new_defaults = new_config.get('defaults', {})
        
        # Update Topic Manager (Subscriptions)
        if hasattr(self, 'topic_manager') and self.topic_manager:
            try:
                 self.topic_manager.update_subscriptions(new_topic_mappings, new_defaults)
                 self.logger.info("TopicManager subscriptions updated.")
            except Exception as e:
                 self.logger.error(f"Error updating TopicManager subscriptions: {e}")
                 import traceback
                 self.logger.error(traceback.format_exc())
        else:
            self.logger.warning("TopicManager not initialized, cannot update subscriptions.")

        # Update Topic Publisher
        if hasattr(self, 'topic_publisher') and self.topic_publisher:
            try:
                 self.topic_publisher.update_publishers(new_topic_mappings, new_defaults)
                 self.logger.info("TopicPublisher publishers updated.")
            except Exception as e:
                 self.logger.error(f"Error updating TopicPublisher publishers: {e}")
                 import traceback
                 self.logger.error(traceback.format_exc())
        else:
            self.logger.warning("TopicPublisher not initialized, cannot update publishers.")
            
        # Update the node's stored configuration
        self.config = new_config
        self.logger.info("Gateway configuration state updated.")
        
        # Re-trigger AV initialization if config changes (important!)
        # Ensure executor exists before trying to create task
        if self.executor:
            self.logger.info("Configuration updated, re-scheduling AV initialization task...")
            # Cancel previous task if it's still running? Risky, could interrupt service calls.
            # Let's schedule it again - the initialize_av_integration needs to be idempotent
            # or handle being called multiple times.
            self.executor.create_task(self.initialize_av_integration())
        else:
            self.logger.error("Node executor not available, cannot re-schedule AV initialization on config update!")

    def filter_topic_mappings_by_source_type(self, topic_mappings, defaults, source_type):
        """Filter topic mappings by source type."""
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
    
    def filter_topic_mappings_by_direction(self, topic_mappings, defaults, direction):
        """Filter topic mappings by direction."""
        default_dir = defaults.get('direction', 'OUTBOUND')
        result = []
        
        for mapping in topic_mappings:
            # Get direction, treat empty string as needing default
            map_direction = mapping.get('direction') 
            if map_direction is None or map_direction == "":
                map_direction = default_dir
            
            # Check if it matches the requested direction
            if map_direction == direction:
                # Apply defaults for missing values
                config = defaults.copy()
                config.update(mapping)
                result.append(config)
                
        return result

    async def initialize_av_integration(self):
        """Initialize integration with the AV node for video/audio streaming."""
        self.logger.info("Initializing AV integration...")
        
        # Import EncodedFrameSubscriber here to avoid circular imports
        try:
            # The timer trigger in schedule_av_initialization should provide sufficient delay.
            # Service availability is handled by wait_for_service in EncodedFrameSubscriber.
            self.logger.info("Proceeding with AV subscriber/client creation.")
            
            from ros_gateway.av_integration import EncodedFrameSubscriber
            
            # Create the encoded frame subscriber IF NOT ALREADY CREATED
            # This method might be called multiple times (init + config update)
            if self.encoded_frame_subscriber is None:
                self.logger.info("Creating new EncodedFrameSubscriber instance.")
                self.encoded_frame_subscriber = EncodedFrameSubscriber(
                    self,
                    self.zmq_client,
                    self.logger
                )
            else:
                 self.logger.info("EncodedFrameSubscriber already exists, reusing instance.")
                 # TODO: Ideally, EncodedFrameSubscriber should handle config updates internally
                 # For now, we rely on re-calling configure/enable below.

            # Check if the subscriber object was actually created (handles potential errors in its __init__)
            if self.encoded_frame_subscriber is None:
                 self.logger.error("Failed to create or reuse EncodedFrameSubscriber instance. Aborting AV integration.")
                 return
                 
            # Ensure the critical manage_stream_client was created within the subscriber
            if not hasattr(self.encoded_frame_subscriber, 'manage_stream_client') or self.encoded_frame_subscriber.manage_stream_client is None:
                self.logger.error("EncodedFrameSubscriber exists, but its ManageStream client is invalid. Aborting AV configuration.")
                return

            # Find AV stream configurations in topic_mappings
            topic_mappings = self.config.get('topic_mappings', [])
            av_configs = []
            
            # Find all topic mappings that have encoder_params (indicates they are AV streams)
            for mapping in topic_mappings:
                if 'encoder_params' in mapping:
                    self.logger.info(f"Found AV stream in topic_mappings: {mapping['ros_topic']}")
                    av_configs.append(mapping)
            
            if not av_configs:
                self.logger.info("No AV streams found in topic_mappings, skipping AV configuration/enabling")
                # TODO: Should we disable/remove existing AV streams if config changes?
                # Requires more complex state management in EncodedFrameSubscriber
                return
                
            self.logger.info(f"Found {len(av_configs)} AV streams in current config")
            
            # TODO: Diff current streams vs new streams for more granular updates
            # For now, just re-configure and re-enable all found streams
            
            # Configure each stream
            stream_ids = []
            for stream_config in av_configs:
                self.logger.info(f"Configuring AV stream from config: {stream_config}")
                success, result = await self.encoded_frame_subscriber.configure_av_stream(stream_config)
                if success:
                    stream_ids.append(result)  # result is the stream_id
                    self.logger.info(f"Successfully configured AV stream: {result}")
                else:
                    self.logger.error(f"Failed to configure AV stream: {result}")
                    
            # Enable all successfully configured streams
            self.logger.info(f"Attempting to enable {len(stream_ids)} successfully configured streams...")
            for stream_id in stream_ids:
                success, message = await self.encoded_frame_subscriber.enable_av_stream(stream_id)
                if success:
                    self.logger.info(f"Enabled AV stream: {stream_id}")
                else:
                    self.logger.error(f"Failed to enable AV stream {stream_id}: {message}")
            
            self.logger.info("AV integration initialization/update finished")
            
        except ImportError as e:
            self.logger.error(f"Failed to import required modules for AV integration: {e}")
            # Don't raise, just log the error and potentially disable AV features
        except Exception as e:
            self.logger.error(f"Error initializing AV integration: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            # Don't raise, allow gateway to continue without AV

def main(args=None):
    rclpy.init(args=args)
    gateway_node = None # Initialize to prevent potential UnboundLocalError
    executor = None # Define executor in main scope
    
    parser = argparse.ArgumentParser(description='ROS Gateway for Open-Teleop')
    parser.add_argument('--config-path', type=str, required=True, 
                        help='Path to the gateway bootstrap configuration file (YAML)')
    
    # Parse only known arguments, allowing ROS arguments to pass through
    parsed_args, _ = parser.parse_known_args()
    
    # Check if config file exists before trying to init node
    if not os.path.exists(parsed_args.config_path):
        # Use a temporary basic logger for this initial error message
        temp_logger = log.get_logger('ros_gateway_init', log.ERROR)
        temp_logger.fatal(f"Bootstrap configuration file not found: {parsed_args.config_path}")
        sys.exit(1)
        
    try:
        # Use SingleThreadedExecutor explicitly so we can access it
        executor = rclpy.executors.SingleThreadedExecutor()
        gateway_node = RosGateway(config_path=parsed_args.config_path)
        executor.add_node(gateway_node)
        gateway_node.logger.info("Node added to executor. Spinning...")
        # Spin the executor indefinitely until shutdown
        executor.spin()
            
    except (KeyboardInterrupt, ExternalShutdownException):
        if gateway_node:
             gateway_node.logger.info("Shutdown signal received.")
    except Exception as e:
        # Log fatal error before exiting
        if gateway_node:
            gateway_node.logger.fatal(f"Fatal error during gateway execution: {e}")
            import traceback
            gateway_node.logger.fatal(traceback.format_exc())
        else:
            # Use temporary logger if node logger isn't available
            temp_logger = log.get_logger(name='ros_gateway_init_error', console_level=log.ERROR, file_level=None)
            temp_logger.fatal(f"Fatal error during gateway initialization or pre-spin: {e}")
            import traceback
            temp_logger.fatal(traceback.format_exc())
            
    finally:
        # Shutdown sequence
        if gateway_node:
            gateway_node.logger.info("Destroying node...")
            # Call the node's explicit shutdown method first
            gateway_node.shutdown()
            # Important: Destroy node BEFORE shutting down executor
            gateway_node.destroy_node()
            gateway_node.logger.info("Node destroyed.")
            
        if executor:
             executor.shutdown()
             print("Executor shutdown complete.") # Use print as logger might be gone
             
        # rclpy shutdown happens automatically if using context manager,
        # but call explicitly otherwise if needed.
        # if rclpy.ok():
        #     rclpy.shutdown()
        print("ROS Gateway process finished.")


if __name__ == '__main__':
    main() 