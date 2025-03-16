#!/usr/bin/env python3
# gateway.launch.py - Launch file for the ROS Gateway

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
    
    # Create the gateway node
    gateway_node = Node(
        package='ros_gateway',
        executable='gateway_node',
        name='ros_gateway',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'config_path': LaunchConfiguration('config_path')
        }],
        namespace=''
    )
    
    # Return the launch description
    return LaunchDescription([
        config_path_arg,
        gateway_node
    ]) 