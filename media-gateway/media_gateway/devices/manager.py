"""
Device Manager for Media Gateway.

This module handles hardware device discovery, capability querying,
and validation of configuration against available devices.
"""

import asyncio
from typing import List
import structlog

from .models import VideoDevice, AudioDevice, Resolution
from ..config.models import MediaConfig


class DeviceManager:
    """
    Manages hardware device discovery and validation.
    
    This is a stub implementation that will be expanded to handle:
    - V4L2 video device discovery and capability querying
    - ALSA audio device discovery
    - Configuration validation against device capabilities
    """
    
    def __init__(self):
        """Initialize the DeviceManager."""
        self.logger = structlog.get_logger(__name__)
        self.video_devices: List[VideoDevice] = []
        self.audio_devices: List[AudioDevice] = []
        
    async def discover_devices(self):
        """
        Discover available video and audio devices.
        
        This is a stub implementation. In the full implementation, this will:
        - Enumerate /dev/video* devices using pyudev
        - Query V4L2 capabilities for each video device
        - Enumerate ALSA audio devices
        - Query audio device capabilities
        """
        self.logger.info("Starting device discovery...")
        
        # TODO: Implement actual device discovery
        # For now, create mock devices for testing
        await self._discover_video_devices()
        await self._discover_audio_devices()
        
        self.logger.info("Device discovery completed",
                        video_devices=len(self.video_devices),
                        audio_devices=len(self.audio_devices))
        
    async def _discover_video_devices(self):
        """Discover video capture devices (stub implementation)."""
        # TODO: Implement V4L2 device discovery
        # import pyudev
        # import v4l2py
        
        # Mock device for testing
        mock_device = VideoDevice(
            device_id="video0",
            device_path="/dev/video0",
            name="Mock Camera",
            resolutions=[
                Resolution(640, 480, 30),
                Resolution(1280, 720, 30),
                Resolution(1920, 1080, 30),
            ]
        )
        self.video_devices = [mock_device]
        self.logger.debug("Mock video device created", device=mock_device.name)
        
    async def _discover_audio_devices(self):
        """Discover audio capture devices (stub implementation)."""
        # TODO: Implement ALSA device discovery
        # import alsaaudio
        
        # Mock device for testing
        mock_device = AudioDevice(
            device_id="audio0",
            device_path="hw:0,0",
            name="Mock Microphone"
        )
        self.audio_devices = [mock_device]
        self.logger.debug("Mock audio device created", device=mock_device.name)
        
    def get_video_device(self, device_id: str) -> VideoDevice:
        """Get video device by ID."""
        for device in self.video_devices:
            if device.device_id == device_id:
                return device
        raise ValueError(f"Video device not found: {device_id}")
        
    def get_audio_device(self, device_id: str) -> AudioDevice:
        """Get audio device by ID."""
        for device in self.audio_devices:
            if device.device_id == device_id:
                return device
        raise ValueError(f"Audio device not found: {device_id}")
        
    def validate_config(self, config: MediaConfig) -> bool:
        """
        Validate configuration against discovered devices.
        
        Args:
            config: Media configuration to validate
            
        Returns:
            True if configuration is valid, False otherwise
        """
        self.logger.info("Validating configuration against devices")
        
        for stream in config.streams:
            try:
                if stream.encoding_format.startswith("video/"):
                    device = self.get_video_device(stream.device_id)
                    if not self._validate_video_stream(device, stream):
                        return False
                elif stream.encoding_format.startswith("audio/"):
                    device = self.get_audio_device(stream.device_id)
                    if not self._validate_audio_stream(device, stream):
                        return False
                        
            except ValueError as e:
                self.logger.error("Device validation failed", error=str(e))
                return False
                
        self.logger.info("Configuration validation passed")
        return True
        
    def _validate_video_stream(self, device: VideoDevice, stream) -> bool:
        """Validate video stream configuration against device capabilities."""
        params = stream.encoder_params
        
        if params.width and params.height and params.framerate:
            if not device.supports_resolution(params.width, params.height, params.framerate):
                self.logger.error("Unsupported video resolution",
                                device=device.device_id,
                                width=params.width,
                                height=params.height,
                                fps=params.framerate)
                return False
                
        return True
        
    def _validate_audio_stream(self, device: AudioDevice, stream) -> bool:
        """Validate audio stream configuration against device capabilities."""
        params = stream.encoder_params
        
        if params.sample_rate and params.channels:
            if not device.supports_format(params.sample_rate, params.channels):
                self.logger.error("Unsupported audio format",
                                device=device.device_id,
                                sample_rate=params.sample_rate,
                                channels=params.channels)
                return False
                
        return True 