#!/usr/bin/env python3
# gateway_launch.py - Development environment launch file for ROS Gateway
from launch import LaunchDescription
from launch.actions import LogInfo

import sys
import os
# Add the parent launch directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'common'))
from base_gateway import generate_base_launch_description

def generate_launch_description():
    """Development environment launch file"""
    
    # Log the environment being launched
    log_info = LogInfo(msg="Starting ROS Gateway in DEVELOPMENT environment")
    
    # Create base launch description with development-specific settings
    base_components = generate_base_launch_description(
        zmq_controller_address='tcp://localhost:5555',
        zmq_publish_address='tcp://*:5556'
    )
    
    # Development-specific additional components could be added here
    
    return LaunchDescription([log_info] + base_components) 