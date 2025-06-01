"""
Device management for Media Gateway.

This package handles hardware device discovery, capability querying,
and validation of configuration against available devices.
"""

from .manager import DeviceManager
from .models import VideoDevice, AudioDevice, Resolution

__all__ = ["DeviceManager", "VideoDevice", "AudioDevice", "Resolution"] 