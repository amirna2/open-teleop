"""
Configuration Manager for Media Gateway.

This module handles configuration requests, parsing, and update monitoring
using the same patterns as other Open Teleop components.
"""

import asyncio
from typing import Callable, Optional
import requests
import structlog

from .models import MediaConfig


class ConfigManager:
    """
    Manages configuration for the Media Gateway.
    
    Follows the same pattern as A/V Node for requesting configuration
    from the Controller and handling updates.
    """
    
    def __init__(self, controller_url: str):
        """Initialize the ConfigManager.
        
        Args:
            controller_url: Base URL of the Open Teleop Controller
        """
        self.controller_url = controller_url.rstrip('/')
        self.config_endpoint = f"{self.controller_url}/config"
        self.current_config: Optional[MediaConfig] = None
        self.config_version: Optional[str] = None
        self.update_callback: Optional[Callable[[MediaConfig], None]] = None
        self.logger = structlog.get_logger(__name__)
        
        # Update monitoring
        self._monitoring = False
        self._monitor_task: Optional[asyncio.Task] = None
        
    async def request_config(self) -> MediaConfig:
        """
        Request configuration from the Controller.
        
        Uses the same HTTP endpoint pattern as other components.
        
        Returns:
            MediaConfig: Parsed media configuration
            
        Raises:
            Exception: If configuration request or parsing fails
        """
        try:
            self.logger.info("Requesting configuration from Controller", 
                           endpoint=self.config_endpoint)
            
            # Make HTTP request to Controller (same pattern as A/V Node)
            response = requests.get(self.config_endpoint, timeout=30)
            response.raise_for_status()
            
            config_data = response.json()
            self.logger.debug("Received configuration", data=config_data)
            
            # Parse configuration
            config = MediaConfig.from_dict(config_data)
            
            # Update internal state
            self.current_config = config
            self.config_version = config.config_version
            
            self.logger.info("Configuration parsed successfully", 
                           streams=len(config.streams),
                           version=config.config_version)
            
            # Start monitoring for updates if not already running
            if not self._monitoring and self.update_callback:
                await self._start_monitoring()
                
            return config
            
        except requests.RequestException as e:
            self.logger.error("Failed to request configuration", error=str(e))
            raise Exception(f"Configuration request failed: {e}")
            
        except Exception as e:
            self.logger.error("Failed to parse configuration", error=str(e))
            raise Exception(f"Configuration parsing failed: {e}")
            
    def set_update_callback(self, callback: Callable[[MediaConfig], None]):
        """
        Set callback for configuration updates.
        
        Args:
            callback: Function to call when configuration is updated
        """
        self.update_callback = callback
        self.logger.info("Configuration update callback set")
        
    async def _start_monitoring(self):
        """Start monitoring for configuration updates."""
        if self._monitoring:
            return
            
        self._monitoring = True
        self._monitor_task = asyncio.create_task(self._monitor_config_updates())
        self.logger.info("Configuration monitoring started")
        
    async def _stop_monitoring(self):
        """Stop monitoring for configuration updates."""
        if not self._monitoring:
            return
            
        self._monitoring = False
        if self._monitor_task:
            self._monitor_task.cancel()
            try:
                await self._monitor_task
            except asyncio.CancelledError:
                pass
            self._monitor_task = None
            
        self.logger.info("Configuration monitoring stopped")
        
    async def _monitor_config_updates(self):
        """
        Monitor for configuration updates.
        
        Polls the Controller periodically to check for configuration changes.
        This is a simple implementation - could be enhanced with webhooks later.
        """
        poll_interval = 30  # seconds
        
        while self._monitoring:
            try:
                await asyncio.sleep(poll_interval)
                
                if not self._monitoring:
                    break
                    
                # Check for configuration updates
                response = requests.get(self.config_endpoint, timeout=10)
                response.raise_for_status()
                
                config_data = response.json()
                new_version = config_data.get("config_version")
                
                # Check if configuration has changed
                if new_version and new_version != self.config_version:
                    self.logger.info("Configuration update detected", 
                                   old_version=self.config_version,
                                   new_version=new_version)
                    
                    # Parse new configuration
                    new_config = MediaConfig.from_dict(config_data)
                    
                    # Update internal state
                    self.current_config = new_config
                    self.config_version = new_version
                    
                    # Notify callback
                    if self.update_callback:
                        try:
                            await self.update_callback(new_config)
                        except Exception as e:
                            self.logger.error("Configuration update callback failed", 
                                            error=str(e))
                            
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.logger.warning("Configuration monitoring error", error=str(e))
                # Continue monitoring despite errors
                
    async def shutdown(self):
        """Shutdown the ConfigManager gracefully."""
        await self._stop_monitoring()
        self.logger.info("ConfigManager shutdown complete") 