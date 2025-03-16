#!/usr/bin/env python3
# launch_gateway.py - Convenience launcher for ROS Gateway with environment selection
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    """Launch descriptor with environment selection"""
    
    # Declare the environment argument with default value
    env_arg = DeclareLaunchArgument(
        'environment',
        default_value='development',
        description='Launch environment (development, testing, production)'
    )
    
    # Get the environment from the launch argument
    environment = LaunchConfiguration('environment')
    
    # Get the path to this launch file
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    
    # Include the appropriate launch file based on environment
    include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            launch_dir, '/', environment, '/gateway_launch.py'
        ])
    )
    
    return LaunchDescription([
        env_arg,
        include_launch
    ]) 