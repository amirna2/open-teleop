"""
Configuration data models for Media Gateway.

These models define the structure of configuration data that the Media Gateway
receives from the Controller.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Any


@dataclass
class EncoderParams:
    """Encoder parameters for video/audio streams."""
    
    # Video parameters
    width: Optional[int] = None
    height: Optional[int] = None
    framerate: Optional[int] = None
    bitrate: Optional[int] = None  # kbps for video, bps for audio
    gop_size: Optional[int] = None  # For H.264
    quality: Optional[int] = None  # For MJPEG (0-100)
    profile: Optional[str] = None  # "baseline", "main", "high"
    preset: Optional[str] = None  # "ultrafast", "fast", "medium"
    
    # Audio parameters
    sample_rate: Optional[int] = None  # Hz
    channels: Optional[int] = None
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "EncoderParams":
        """Create EncoderParams from dictionary."""
        return cls(
            width=data.get("width"),
            height=data.get("height"),
            framerate=data.get("framerate"),
            bitrate=data.get("bitrate"),
            gop_size=data.get("gop_size"),
            quality=data.get("quality"),
            profile=data.get("profile"),
            preset=data.get("preset"),
            sample_rate=data.get("sample_rate"),
            channels=data.get("channels"),
        )
        
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary, excluding None values."""
        return {k: v for k, v in self.__dict__.items() if v is not None}


@dataclass
class StreamConfig:
    """Configuration for a single media stream."""
    
    device_id: str  # e.g., "video0", "audio0"
    ott_topic: str  # Open Teleop topic name
    topic_id: str  # Unique topic identifier
    encoding_format: str  # e.g., "video/h264", "audio/opus"
    encoder_params: EncoderParams
    priority: str = "STANDARD"  # "HIGH", "STANDARD", "LOW"
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "StreamConfig":
        """Create StreamConfig from dictionary."""
        encoder_params_data = data.get("encoder_params", {})
        encoder_params = EncoderParams.from_dict(encoder_params_data)
        
        return cls(
            device_id=data["device_id"],
            ott_topic=data["ott"],
            topic_id=data["topic_id"],
            encoding_format=data["encoding_format"],
            encoder_params=encoder_params,
            priority=data.get("priority", "STANDARD"),
        )
        
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "device_id": self.device_id,
            "ott": self.ott_topic,
            "topic_id": self.topic_id,
            "encoding_format": self.encoding_format,
            "encoder_params": self.encoder_params.to_dict(),
            "priority": self.priority,
        }


@dataclass
class MediaConfig:
    """Complete media configuration from Controller."""
    
    streams: List[StreamConfig]
    controller_endpoint: str = "tcp://localhost:5555"  # ZeroMQ endpoint
    config_version: Optional[str] = None
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MediaConfig":
        """Create MediaConfig from dictionary."""
        streams = []
        
        # Parse media_mappings section
        media_mappings = data.get("media_mappings", {})
        
        # Parse video streams
        for video_config in media_mappings.get("video", []):
            streams.append(StreamConfig.from_dict(video_config))
            
        # Parse audio streams
        for audio_config in media_mappings.get("audio", []):
            streams.append(StreamConfig.from_dict(audio_config))
            
        return cls(
            streams=streams,
            controller_endpoint=data.get("controller_endpoint", "tcp://localhost:5555"),
            config_version=data.get("config_version"),
        )
        
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        video_streams = []
        audio_streams = []
        
        for stream in self.streams:
            if stream.encoding_format.startswith("video/"):
                video_streams.append(stream.to_dict())
            elif stream.encoding_format.startswith("audio/"):
                audio_streams.append(stream.to_dict())
                
        return {
            "media_mappings": {
                "video": video_streams,
                "audio": audio_streams,
            },
            "controller_endpoint": self.controller_endpoint,
            "config_version": self.config_version,
        }
        
    def get_video_streams(self) -> List[StreamConfig]:
        """Get all video stream configurations."""
        return [s for s in self.streams if s.encoding_format.startswith("video/")]
        
    def get_audio_streams(self) -> List[StreamConfig]:
        """Get all audio stream configurations."""
        return [s for s in self.streams if s.encoding_format.startswith("audio/")]
        
    def get_stream_by_device_id(self, device_id: str) -> Optional[StreamConfig]:
        """Get stream configuration by device ID."""
        for stream in self.streams:
            if stream.device_id == device_id:
                return stream
        return None 