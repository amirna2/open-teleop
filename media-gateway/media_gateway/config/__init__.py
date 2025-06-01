"""
Configuration management for Media Gateway.

This package handles configuration request, parsing, and update monitoring
using the same patterns as other Open Teleop components.
"""

from .manager import ConfigManager
from .models import MediaConfig, StreamConfig, EncoderParams

__all__ = ["ConfigManager", "MediaConfig", "StreamConfig", "EncoderParams"] 