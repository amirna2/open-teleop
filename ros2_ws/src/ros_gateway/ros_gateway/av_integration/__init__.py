"""
AV Integration module for the ROS Gateway.

This module provides integration with the Open Teleop AV Node for handling
video and audio streaming.
"""

from .encoded_frame_subscriber import EncodedFrameSubscriber

__all__ = ['EncodedFrameSubscriber'] 