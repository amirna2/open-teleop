"""
Device models for Media Gateway.

These models represent hardware devices (cameras, microphones) and their capabilities.
"""

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class Resolution:
    """Video resolution capability."""
    width: int
    height: int
    max_fps: int
    
    def __str__(self) -> str:
        return f"{self.width}x{self.height}@{self.max_fps}fps"


@dataclass
class VideoDevice:
    """Video capture device (camera)."""
    device_id: str       # e.g., "video0"
    device_path: str     # e.g., "/dev/video0"
    name: str           # e.g., "Logitech C920"
    resolutions: List[Resolution]
    
    def supports_resolution(self, width: int, height: int, fps: int = 30) -> bool:
        """Check if device supports a specific resolution and frame rate."""
        for res in self.resolutions:
            if res.width == width and res.height == height and res.max_fps >= fps:
                return True
        return False
        
    def get_best_resolution(self) -> Optional[Resolution]:
        """Get the highest resolution supported by the device."""
        if not self.resolutions:
            return None
        return max(self.resolutions, key=lambda r: r.width * r.height)


@dataclass
class AudioDevice:
    """Audio capture device (microphone)."""
    device_id: str       # e.g., "audio0"
    device_path: str     # e.g., "hw:0,0"
    name: str           # e.g., "Logitech C920 Microphone"
    sample_rates: List[int] = None  # Supported sample rates
    channels: List[int] = None      # Supported channel counts
    
    def __post_init__(self):
        """Set default capabilities if not provided."""
        if self.sample_rates is None:
            self.sample_rates = [8000, 16000, 44100, 48000]  # Common rates
        if self.channels is None:
            self.channels = [1, 2]  # Mono and stereo
            
    def supports_format(self, sample_rate: int, channels: int) -> bool:
        """Check if device supports a specific audio format."""
        return sample_rate in self.sample_rates and channels in self.channels 