#!/usr/bin/env python3
# base_gateway.py - Common launch configuration for ROS Gateway
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_base_launch_description(zmq_controller_address, zmq_publish_address):
    """Generate base launch description with configurable ZeroMQ settings"""
    
    # Set environment variables for ZeroMQ configuration
    controller_address_env = SetEnvironmentVariable(
        name='TELEOP_ZMQ_CONTROLLER_ADDRESS', 
        value=zmq_controller_address
    )
    
    publish_address_env = SetEnvironmentVariable(
        name='TELEOP_ZMQ_PUBLISH_ADDRESS',
        value=zmq_publish_address
    )
    
    # Launch the gateway node with these environment variables
    gateway_node = Node(
        package='ros_gateway',
        executable='gateway_node',
        name='ros_gateway',
        output='screen',
        # Additional output for debugging
        emulate_tty=True
    )
    
    return [
        controller_address_env,
        publish_address_env,
        gateway_node
    ] 