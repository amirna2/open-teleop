"""
Open Teleop Media Gateway

Provides direct hardware audio/video capture for Open Teleop systems,
bypassing ROS2 transport limitations for production teleoperation.
"""

__version__ = "0.1.0"

# Import main entry point function instead of MediaGateway class
from .main import main

__all__ = ["main", "__version__"] 