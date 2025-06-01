#!/usr/bin/env python3
"""
Media Gateway Main Entry Point

This module provides the main entry point and orchestration for the Media Gateway.
"""

import argparse
import asyncio
import logging
import signal
import sys
from typing import Optional

import structlog

from .config.manager import ConfigManager
from .devices.manager import DeviceManager
from .pipelines.manager import PipelineManager
from .routing.frame_router import FrameRouter
from .transport.connection_manager import ConnectionManager


class MediaGateway:
    """
    Main Media Gateway class that orchestrates all components.
    
    This class provides the high-level coordination between configuration,
    device discovery, pipeline management, and frame routing.
    """
    
    def __init__(self, controller_url: str, debug: bool = False):
        """Initialize the Media Gateway.
        
        Args:
            controller_url: URL of the Open Teleop Controller
            debug: Enable debug logging
        """
        self.controller_url = controller_url
        self.debug = debug
        self.logger = structlog.get_logger(__name__)
        self.is_running = False
        
        # Core components (initialized in start())
        self.config_manager: Optional[ConfigManager] = None
        self.device_manager: Optional[DeviceManager] = None
        self.pipeline_manager: Optional[PipelineManager] = None
        self.connection_manager: Optional[ConnectionManager] = None
        self.frame_router: Optional[FrameRouter] = None
        
        # Setup signal handling
        self._setup_signal_handlers()
        
    def _setup_signal_handlers(self):
        """Setup graceful shutdown signal handlers."""
        def signal_handler(signum, frame):
            self.logger.info("Received shutdown signal", signal=signum)
            asyncio.create_task(self.stop())
            
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
    async def start(self) -> bool:
        """Start the Media Gateway.
        
        Returns:
            True if startup was successful, False otherwise
        """
        try:
            self.logger.info("Starting Media Gateway", controller_url=self.controller_url)
            
            # Initialize components
            self.logger.info("Initializing components...")
            
            # 1. Initialize device manager first (discover hardware)
            self.device_manager = DeviceManager()
            await self.device_manager.discover_devices()
            self.logger.info("Device discovery completed", 
                           video_devices=len(self.device_manager.video_devices),
                           audio_devices=len(self.device_manager.audio_devices))
            
            # 2. Initialize config manager and fetch configuration
            self.config_manager = ConfigManager(self.controller_url)
            config = await self.config_manager.request_config()
            self.logger.info("Configuration loaded", streams=len(config.streams))
            
            # 3. Validate configuration against discovered devices
            if not self.device_manager.validate_config(config):
                self.logger.error("Configuration validation failed")
                return False
                
            # 4. Initialize connection manager
            self.connection_manager = ConnectionManager(config.controller_endpoint)
            await self.connection_manager.connect()
            self.logger.info("Connected to Controller")
            
            # 5. Initialize frame router
            self.frame_router = FrameRouter(self.connection_manager)
            
            # 6. Initialize pipeline manager
            self.pipeline_manager = PipelineManager(self.frame_router)
            
            # 7. Create and start pipelines based on configuration
            for stream_config in config.streams:
                success = await self.pipeline_manager.create_stream(stream_config)
                if not success:
                    self.logger.error("Failed to create stream", 
                                    device_id=stream_config.device_id)
                    return False
                    
            # 8. Setup configuration update monitoring
            self.config_manager.set_update_callback(self._on_config_update)
            
            self.is_running = True
            self.logger.info("Media Gateway started successfully")
            return True
            
        except Exception as e:
            self.logger.error("Failed to start Media Gateway", error=str(e))
            return False
            
    async def stop(self):
        """Stop the Media Gateway gracefully."""
        if not self.is_running:
            return
            
        self.logger.info("Stopping Media Gateway...")
        self.is_running = False
        
        try:
            # Stop pipelines first
            if self.pipeline_manager:
                await self.pipeline_manager.stop_all_streams()
                
            # Disconnect from controller
            if self.connection_manager:
                await self.connection_manager.disconnect()
                
            self.logger.info("Media Gateway stopped")
            
        except Exception as e:
            self.logger.error("Error during shutdown", error=str(e))
            
    async def run(self):
        """Main run loop for the Media Gateway."""
        if not await self.start():
            sys.exit(1)
            
        try:
            # Keep running until stopped
            while self.is_running:
                await asyncio.sleep(1.0)
                
                # Periodic health checks
                if self.pipeline_manager:
                    await self.pipeline_manager.health_check()
                    
        except KeyboardInterrupt:
            self.logger.info("Keyboard interrupt received")
        finally:
            await self.stop()
            
    async def _on_config_update(self, new_config):
        """Handle configuration updates."""
        self.logger.info("Configuration update received")
        
        if self.pipeline_manager:
            await self.pipeline_manager.update_streams(new_config)


def setup_logging(debug: bool = False):
    """Setup structured logging configuration."""
    log_level = logging.DEBUG if debug else logging.INFO
    
    structlog.configure(
        processors=[
            structlog.stdlib.filter_by_level,
            structlog.stdlib.add_logger_name,
            structlog.stdlib.add_log_level,
            structlog.stdlib.PositionalArgumentsFormatter(),
            structlog.processors.TimeStamper(fmt="iso"),
            structlog.processors.StackInfoRenderer(),
            structlog.processors.format_exc_info,
            structlog.processors.UnicodeDecoder(),
            structlog.processors.JSONRenderer()
        ],
        context_class=dict,
        logger_factory=structlog.stdlib.LoggerFactory(),
        wrapper_class=structlog.stdlib.BoundLogger,
        cache_logger_on_first_use=True,
    )
    
    logging.basicConfig(
        format="%(message)s",
        stream=sys.stdout,
        level=log_level,
    )


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Open Teleop Media Gateway",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                                    # Use default controller URL
  %(prog)s --controller-url http://localhost:8080
  %(prog)s --debug                           # Enable debug logging
        """
    )
    
    parser.add_argument(
        "--controller-url",
        default="http://localhost:8080",
        help="URL of the Open Teleop Controller (default: http://localhost:8080)"
    )
    
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging"
    )
    
    parser.add_argument(
        "--version",
        action="version",
        version=f"Media Gateway {__import__('media_gateway').__version__}"
    )
    
    return parser.parse_args()


async def main_async():
    """Async main function."""
    args = parse_arguments()
    setup_logging(args.debug)
    
    logger = structlog.get_logger(__name__)
    logger.info("Starting Media Gateway", version=__import__('media_gateway').__version__)
    
    gateway = MediaGateway(args.controller_url, args.debug)
    await gateway.run()


def main():
    """Main entry point."""
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
        sys.exit(0)
    except Exception as e:
        print(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main() 