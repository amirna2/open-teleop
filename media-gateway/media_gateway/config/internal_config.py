"""
Internal configuration loader for Media Gateway.

This handles the media-gateway-config.yaml file which contains operational
settings, separate from the media stream configuration from the Controller.
"""

import os
import yaml
from dataclasses import dataclass
from typing import Optional


@dataclass
class ZMQConfig:
    controller_address: str = "tcp://localhost:5555"
    publish_address: str = "tcp://localhost:5556"
    message_buffer_size: int = 1000
    reconnect_interval_ms: int = 1000


@dataclass
class DefaultsConfig:
    priority: str = "STANDARD"
    encoding_format: str = "video/h264"
    video_bitrate: int = 2000
    audio_bitrate: int = 64000


@dataclass
class DeviceDiscoveryConfig:
    auto_discover_on_startup: bool = True
    publish_device_list: bool = True
    device_list_topic: str = "teleop.media.available_sources"
    hotplug_monitoring: bool = False


@dataclass
class GStreamerConfig:
    debug_level: int = 2
    debug_dump_dot_files: bool = False
    buffer_time_ms: int = 200
    latency_ms: int = 100


@dataclass
class PerformanceConfig:
    enable_monitoring: bool = True
    log_frame_stats: bool = False
    stats_interval_seconds: int = 60


@dataclass
class LoggingConfig:
    level: str = "INFO"
    log_to_file: bool = True
    log_path: str = "/tmp/open_teleop_logs"
    log_rotation_days: int = 7


@dataclass
class MediaGatewayInternalConfig:
    """Complete internal configuration for Media Gateway."""
    
    zmq: ZMQConfig
    defaults: DefaultsConfig
    device_discovery: DeviceDiscoveryConfig
    gstreamer: GStreamerConfig
    performance: PerformanceConfig
    logging: LoggingConfig
    
    @classmethod
    def load_from_file(cls, config_path: str) -> "MediaGatewayInternalConfig":
        """Load configuration from YAML file."""
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")
            
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
            
        gateway_config = data.get('media_gateway', {})
        
        return cls(
            zmq=ZMQConfig(**gateway_config.get('zmq', {})),
            defaults=DefaultsConfig(**gateway_config.get('defaults', {})),
            device_discovery=DeviceDiscoveryConfig(**gateway_config.get('device_discovery', {})),
            gstreamer=GStreamerConfig(**gateway_config.get('gstreamer', {})),
            performance=PerformanceConfig(**gateway_config.get('performance', {})),
            logging=LoggingConfig(**gateway_config.get('logging', {}))
        )
        
    @classmethod
    def load_default(cls) -> "MediaGatewayInternalConfig":
        """Load default configuration."""
        return cls(
            zmq=ZMQConfig(),
            defaults=DefaultsConfig(),
            device_discovery=DeviceDiscoveryConfig(),
            gstreamer=GStreamerConfig(),
            performance=PerformanceConfig(),
            logging=LoggingConfig()
        ) 