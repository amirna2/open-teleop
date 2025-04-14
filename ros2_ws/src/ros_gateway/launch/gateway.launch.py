#!/usr/bin/env python3
# gateway.launch.py - Launch file for the ROS Gateway

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Generate launch description for the ROS Gateway node."""
    
    # Get the package share directory
    pkg_dir = get_package_share_directory('ros_gateway')
    
    # Default config path
    default_config_path = os.path.join(pkg_dir, 'config', 'gateway_config.yaml')
    
    # Declare arguments
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Path to the ROS Gateway configuration file'
    )
    
    # Create the gateway node using Python module directly
    gateway_process = ExecuteProcess(
        cmd=[
            'python3', 
            '-m', 
            'ros_gateway.gateway_node', 
            '--config-path', 
            LaunchConfiguration('config_path')
        ],
        name='ros_gateway',
        output='screen',
    )
    
    # Return the launch description
    return LaunchDescription([
        config_path_arg,
        gateway_process
    ]) 