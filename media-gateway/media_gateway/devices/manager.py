"""
Device Manager for Media Gateway.

This module handles hardware device discovery, capability querying,
and validation of configuration against available devices.
"""

import asyncio
import os
import subprocess
import logging
from pathlib import Path
from typing import List, Dict, Optional

# Import v4l2py types first
from v4l2py.device import Device
HAS_V4L2_SUPPORT = False

try:
    import pyudev
    import v4l2py
    HAS_V4L2_SUPPORT = True
except ImportError:
    pass

from .models import VideoDevice, AudioDevice, Resolution


class DeviceManager:
    """
    Manages hardware device discovery and validation.
    
    Discovers video devices using V4L2 and audio devices using ALSA,
    queries their capabilities, and validates configuration against
    available hardware.
    """
    
    def __init__(self):
        """Initialize the DeviceManager."""
        self.logger = logging.getLogger(__name__)
        self.video_devices: List[VideoDevice] = []
        self.audio_devices: List[AudioDevice] = []
        self._udev_context = None
        
    async def discover_devices(self):
        """
        Discover available video and audio devices.
        
        Enumerates V4L2 video devices and ALSA audio devices,
        queries their capabilities, and populates device lists.
        """
        self.logger.info("Starting device discovery...")
        
        await self._discover_video_devices()
        await self._discover_audio_devices()
        
        self.logger.info("Device discovery completed: %d video devices, %d audio devices", 
                        len(self.video_devices), len(self.audio_devices))
        
    async def _discover_video_devices(self):
        """Discover video capture devices using V4L2."""
        self.video_devices.clear()
        
        if not HAS_V4L2_SUPPORT:
            self.logger.warning("V4L2 support not available, using mock devices")
            await self._create_mock_video_devices()
            return
            
        try:
            self._udev_context = pyudev.Context()
            
            # Find all video devices and group by USB parent device
            video_devices_by_parent = {}
            
            for device in self._udev_context.list_devices(subsystem='video4linux'):
                device_path = device.device_node
                if not device_path:
                    continue
                    
                self.logger.info("Processing udev device: %s", device_path)
                
                # Find USB parent device for grouping
                parent = device.find_parent('usb', 'usb_device')
                if parent is not None:
                    # Group by USB device identity
                    key = (
                        parent.properties.get('ID_VENDOR_ID'),
                        parent.properties.get('ID_MODEL_ID'),
                        parent.properties.get('ID_SERIAL_SHORT')
                    )
                    device_name = parent.properties.get('ID_MODEL') or 'Unknown USB Camera'
                    self.logger.info("Found USB camera: %s, parent key: %s", device_name, key)
                else:
                    # Fallback for non-USB devices (built-in cameras, etc.)
                    key = (
                        device.properties.get('ID_VENDOR_ID'),
                        device.properties.get('ID_MODEL_ID'), 
                        device.properties.get('ID_SERIAL_SHORT')
                    )
                    device_name = device.properties.get('ID_MODEL') or 'Unknown Camera'
                    self.logger.info("Found non-USB camera: %s, device key: %s", device_name, key)
                
                # Only store the first (lowest numbered) device for each physical camera
                if key not in video_devices_by_parent:
                    video_devices_by_parent[key] = {
                        'name': device_name,
                        'device_path': device_path,
                        'device_id': device_path.split('/')[-1]  # Extract "video0" from "/dev/video0"
                    }
                    self.logger.info("First interface for camera '%s': %s", device_name, device_path)
                else:
                    self.logger.info("Skipping additional interface for camera '%s': %s", 
                                   device_name, device_path)
            
            # Create VideoDevice objects for each unique physical camera
            for camera_info in video_devices_by_parent.values():
                device_path = camera_info['device_path']
                
                # Check if this is actually a capture device
                if await self._is_video_capture_device(device_path):
                    self.logger.info("Creating video device for: %s", device_path)
                    video_device = await self._create_video_device(device_path)
                    if video_device:
                        self.video_devices.append(video_device)
                        self.logger.info("Successfully added video device: %s (%s) - %d resolutions", 
                                       video_device.device_id, video_device.name, len(video_device.resolutions))
                    else:
                        self.logger.warning("Failed to create video device: %s", device_path)
                else:
                    self.logger.info("Device failed capture check: %s", device_path)
                        
        except Exception as e:
            self.logger.error("Failed to discover video devices: %s", str(e))
            await self._create_mock_video_devices()
            
    async def _is_video_capture_device(self, device_path: str) -> bool:
        """Check if a V4L2 device supports video capture."""
        try:
            with v4l2py.Device(device_path) as device:
                # Use the correct v4l2py API to get capabilities
                caps = device.info
                self.logger.info("Checking device capabilities: %s", device_path)
                
                # V4L2 capability constants (from linux/videodev2.h)
                V4L2_CAP_VIDEO_CAPTURE = 0x00000001
                V4L2_CAP_VIDEO_CAPTURE_MPLANE = 0x00001000
                
                # Check for various video capture capabilities
                has_video_capture = caps.capabilities & V4L2_CAP_VIDEO_CAPTURE
                has_video_capture_mplane = caps.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE
                
                self.logger.info("Capability check results: has_video_capture=%s, has_video_capture_mplane=%s",
                               bool(has_video_capture), bool(has_video_capture_mplane))
                
                # Accept either single-plane or multi-plane capture
                is_capture_device = has_video_capture or has_video_capture_mplane
                
                self.logger.info("Device decision: %s", is_capture_device)
                
                return is_capture_device
        except Exception as e:
            self.logger.warning("Failed to check device capabilities: %s", str(e))
            return False
            
    async def _create_video_device(self, device_path: str) -> Optional[VideoDevice]:
        """Create VideoDevice from V4L2 device path."""
        try:
            with v4l2py.Device(device_path) as device:
                # Use the correct v4l2py API to get device info
                caps = device.info
                
                # Extract device info
                device_id = Path(device_path).name  # e.g., "video0"
                name = caps.card or f"Camera {device_id}"
                
                # Query supported resolutions and frame rates
                resolutions = await self._query_video_resolutions(device)
                
                return VideoDevice(
                    device_id=device_id,
                    device_path=device_path,
                    name=name,
                    resolutions=resolutions
                )
                
        except Exception as e:
            self.logger.warning("Failed to query video device: %s", str(e))
            return None
            
    async def _query_video_resolutions(self, device: Device) -> List[Resolution]:
        """Query supported video resolutions and frame rates."""
        resolutions = []
        
        try:
            # Get supported formats
            for fmt in device.query_video_capture_formats():
                if fmt.pixel_format in [v4l2py.PixelFormat.YUYV, 
                                       v4l2py.PixelFormat.MJPEG,
                                       v4l2py.PixelFormat.H264]:
                    
                    # Get frame sizes for this format
                    for frame_size in device.query_video_capture_frame_sizes(fmt.pixel_format):
                        if hasattr(frame_size, 'discrete'):
                            width = frame_size.discrete.width
                            height = frame_size.discrete.height
                            
                            # Get frame rates for this size
                            max_fps = await self._query_max_framerate(device, fmt.pixel_format, width, height)
                            
                            resolution = Resolution(width, height, max_fps)
                            if resolution not in resolutions:
                                resolutions.append(resolution)
                                
        except Exception as e:
            self.logger.debug("Error querying resolutions: %s", str(e))
            # Fallback to common resolutions
            resolutions = [
                Resolution(640, 480, 30),
                Resolution(1280, 720, 30),
            ]
            
        return sorted(resolutions, key=lambda r: (r.width * r.height, r.max_fps))
        
    async def _query_max_framerate(self, device: Device, pixel_format, width: int, height: int) -> int:
        """Query maximum framerate for a given format and resolution."""
        try:
            for frame_rate in device.query_video_capture_frame_rates(pixel_format, width, height):
                if hasattr(frame_rate, 'discrete'):
                    # Return the highest discrete framerate
                    return int(frame_rate.discrete.numerator / frame_rate.discrete.denominator)
        except Exception:
            pass
            
        # Default fallback
        return 30
        
    async def _create_mock_video_devices(self):
        """Create mock video devices for testing."""
        mock_device = VideoDevice(
            device_id="video0",
            device_path="/dev/video0",
            name="Mock Camera (V4L2 unavailable)",
            resolutions=[
                Resolution(640, 480, 30),
                Resolution(1280, 720, 30),
                Resolution(1920, 1080, 30),
            ]
        )
        self.video_devices = [mock_device]
        self.logger.debug("Created mock video device: %s", mock_device.name)
        
    async def _discover_audio_devices(self):
        """Discover audio capture devices using ALSA."""
        self.audio_devices.clear()
        
        try:
            # Use arecord to list capture devices
            result = subprocess.run(['arecord', '-l'], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                await self._parse_arecord_output(result.stdout)
            else:
                raise subprocess.CalledProcessError(result.returncode, 'arecord')
                
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError) as e:
            self.logger.warning("Failed to discover audio devices: %s", str(e))
            await self._create_mock_audio_devices()
            
    async def _parse_arecord_output(self, output: str):
        """Parse arecord -l output to find audio devices."""
        device_count = 0
        seen_devices = set()  # Track device paths to avoid duplicates
        
        for line in output.split('\n'):
            if line.startswith('card '):
                try:
                    # Parse line like: "card 0: PCH [HDA Intel PCH], device 0: ALC3246 Analog [ALC3246 Analog]"
                    parts = line.split(':')
                    card_info = parts[0].strip()  # "card 0"
                    card_num = card_info.split()[1]
                    
                    if len(parts) >= 2:
                        name_part = parts[1].split(',')[0].strip()  # "PCH [HDA Intel PCH]"
                        name = name_part.split('[')[0].strip() if '[' in name_part else name_part
                        
                        device_path = f"hw:{card_num},0"
                        
                        # Skip if we've already seen this device path
                        if device_path in seen_devices:
                            self.logger.info("Skipping duplicate audio device: %s", device_path)
                            continue
                            
                        seen_devices.add(device_path)
                        device_id = f"audio{device_count}"
                        
                        audio_device = AudioDevice(
                            device_id=device_id,
                            device_path=device_path,
                            name=f"{name} Microphone"
                        )
                        self.audio_devices.append(audio_device)
                        device_count += 1
                        
                        self.logger.info("Discovered audio device: %s (%s)",
                                       device_id, audio_device.name)
                        
                except Exception as e:
                    self.logger.debug("Failed to parse audio device line: %s", line)
                    
        if not self.audio_devices:
            await self._create_mock_audio_devices()
            
    async def _create_mock_audio_devices(self):
        """Create mock audio devices for testing."""
        mock_device = AudioDevice(
            device_id="audio0",
            device_path="hw:0,0",
            name="Mock Microphone (ALSA unavailable)"
        )
        self.audio_devices = [mock_device]
        self.logger.debug("Created mock audio device: %s", mock_device.name)
        
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
        
    def get_available_sources(self) -> Dict:
        """
        Get available sources in the format expected by the discovery message.
        
        Returns:
            Dictionary containing video_sources and audio_sources arrays
        """
        video_sources = []
        for device in self.video_devices:
            resolutions = []
            for res in device.resolutions:
                resolutions.append({
                    "width": res.width,
                    "height": res.height,
                    "max_fps": res.max_fps
                })
                
            video_sources.append({
                "device_id": device.device_id,
                "device_path": device.device_path,
                "name": device.name,
                "resolutions": resolutions
            })
            
        audio_sources = []
        for device in self.audio_devices:
            audio_sources.append({
                "device_id": device.device_id,
                "device_path": device.device_path,
                "name": device.name
            })
            
        return {
            "video_sources": video_sources,
            "audio_sources": audio_sources
        }
        
    def validate_config(self, config: Dict) -> bool:
        """
        Validate configuration against discovered devices.
        
        Args:
            config: Media configuration dictionary to validate
            
        Returns:
            True if configuration is valid, False otherwise
        """
        self.logger.info("Validating configuration against devices")
        
        # For now, just return True - full validation can be added later
        # when we have the proper config structure defined
        self.logger.info("Configuration validation passed (stubbed)")
        return True
 