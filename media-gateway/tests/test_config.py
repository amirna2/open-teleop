"""
Tests for configuration management.
"""

import pytest
from media_gateway.config.models import EncoderParams, StreamConfig, MediaConfig


def test_encoder_params_from_dict():
    """Test EncoderParams creation from dictionary."""
    data = {
        "width": 1920,
        "height": 1080,
        "framerate": 30,
        "bitrate": 4000,
        "gop_size": 15
    }
    
    params = EncoderParams.from_dict(data)
    
    assert params.width == 1920
    assert params.height == 1080
    assert params.framerate == 30
    assert params.bitrate == 4000
    assert params.gop_size == 15


def test_stream_config_from_dict():
    """Test StreamConfig creation from dictionary."""
    data = {
        "device_id": "video0",
        "ott": "teleop.video.main_camera",
        "topic_id": "test-topic-id",
        "encoding_format": "video/h264",
        "encoder_params": {
            "width": 1920,
            "height": 1080,
            "framerate": 30,
            "bitrate": 4000
        },
        "priority": "HIGH"
    }
    
    config = StreamConfig.from_dict(data)
    
    assert config.device_id == "video0"
    assert config.ott_topic == "teleop.video.main_camera"
    assert config.encoding_format == "video/h264"
    assert config.encoder_params.width == 1920
    assert config.priority == "HIGH"


def test_media_config_from_dict():
    """Test MediaConfig creation from dictionary."""
    data = {
        "media_mappings": {
            "video": [
                {
                    "device_id": "video0",
                    "ott": "teleop.video.main_camera",
                    "topic_id": "test-topic-id",
                    "encoding_format": "video/h264",
                    "encoder_params": {
                        "width": 1920,
                        "height": 1080,
                        "framerate": 30,
                        "bitrate": 4000
                    }
                }
            ]
        },
        "controller_endpoint": "tcp://localhost:5555",
        "config_version": "1.0"
    }
    
    config = MediaConfig.from_dict(data)
    
    assert len(config.streams) == 1
    assert config.controller_endpoint == "tcp://localhost:5555"
    assert config.config_version == "1.0"
    
    video_streams = config.get_video_streams()
    assert len(video_streams) == 1
    assert video_streams[0].device_id == "video0" 