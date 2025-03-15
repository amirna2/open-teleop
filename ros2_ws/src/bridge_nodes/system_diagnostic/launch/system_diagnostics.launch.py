#!/usr/bin/env python3
# Copyright (c) 2023 Open-Teleop
# License: MIT

"""
Launch file for the system diagnostics bridge node.

This launch file starts the system diagnostics node with configurable parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for system diagnostics bridge."""
    # Declare arguments
    robot_id = LaunchConfiguration('robot_id', default='default_robot')
    metrics_update_period = LaunchConfiguration('metrics_update_period', default='1.0')
    nodes_update_period = LaunchConfiguration('nodes_update_period', default='5.0')
    controller_address = LaunchConfiguration('controller_address', default='localhost:8080')
    
    # Declare arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='default_robot',
        description='Robot ID for diagnostics reporting'
    )
    
    metrics_period_arg = DeclareLaunchArgument(
        'metrics_update_period',
        default_value='1.0',
        description='Period in seconds for updating system metrics'
    )
    
    nodes_period_arg = DeclareLaunchArgument(
        'nodes_update_period',
        default_value='5.0',
        description='Period in seconds for updating ROS node status'
    )
    
    controller_address_arg = DeclareLaunchArgument(
        'controller_address',
        default_value='localhost:8080',
        description='Address of the Go Controller'
    )
    
    # Create node
    system_diagnostics_node = Node(
        package='system_diagnostic',
        executable='system_diagnostics_node',
        name='system_diagnostics_node',
        parameters=[{
            'robot_id': robot_id,
            'metrics_update_period': metrics_update_period,
            'nodes_update_period': nodes_update_period,
            'controller_address': controller_address,
        }],
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        robot_id_arg,
        metrics_period_arg,
        nodes_period_arg,
        controller_address_arg,
        system_diagnostics_node
    ])
