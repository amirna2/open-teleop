#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

def generate_launch_description():
    # Get environment variables with defaults
    robot_id = EnvironmentVariable('ROBOT_ID', default_value='default_robot')
    
    # Launch arguments
    video_topic = LaunchConfiguration('video_topic', default='/camera/image_raw')
    audio_topic = LaunchConfiguration('audio_topic', default='/microphone/audio')
    teleop_topic = LaunchConfiguration('teleop_topic', default='/cmd_vel')
    sensor_topic = LaunchConfiguration('sensor_topic', default='/sensors/data')
    navigation_topic = LaunchConfiguration('navigation_topic', default='/move_base/goal')
    diagnostic_period = LaunchConfiguration('diagnostic_period', default='1.0')
    
    # Declare launch arguments
    declare_video_topic = DeclareLaunchArgument(
        'video_topic',
        default_value='/camera/image_raw',
        description='ROS topic for camera feed'
    )
    
    declare_audio_topic = DeclareLaunchArgument(
        'audio_topic',
        default_value='/microphone/audio',
        description='ROS topic for audio feed'
    )
    
    declare_teleop_topic = DeclareLaunchArgument(
        'teleop_topic',
        default_value='/cmd_vel',
        description='ROS topic for robot movement commands'
    )
    
    declare_sensor_topic = DeclareLaunchArgument(
        'sensor_topic',
        default_value='/sensors/data',
        description='ROS topic for sensor data'
    )
    
    declare_navigation_topic = DeclareLaunchArgument(
        'navigation_topic',
        default_value='/move_base/goal',
        description='ROS topic for navigation goals'
    )
    
    declare_diagnostic_period = DeclareLaunchArgument(
        'diagnostic_period',
        default_value='1.0',
        description='Period in seconds for diagnostic updates'
    )
    
    # Video streaming bridge node
    video_bridge_node = Node(
        package='video_streaming_bridge',
        executable='video_bridge_node',
        name='video_bridge',
        parameters=[{
            'video_topic': video_topic,
            'frame_rate': 30,
            'quality': 80,
        }],
        output='screen'
    )
    
    # Audio streaming bridge node
    audio_bridge_node = Node(
        package='audio_streaming_bridge',
        executable='audio_bridge_node',
        name='audio_bridge',
        parameters=[{
            'audio_topic': audio_topic,
            'sample_rate': 16000,
            'channels': 1,
        }],
        output='screen'
    )
    
    # Teleop command bridge node
    teleop_bridge_node = Node(
        package='teleop_command_bridge',
        executable='teleop_bridge_node',
        name='teleop_bridge',
        parameters=[{
            'teleop_topic': teleop_topic,
            'max_linear_speed': 0.5,
            'max_angular_speed': 1.0,
        }],
        output='screen'
    )
    
    # Sensor bridge node
    sensor_bridge_node = Node(
        package='sensor_bridge',
        executable='sensor_bridge_node',
        name='sensor_bridge',
        parameters=[{
            'sensor_topic': sensor_topic,
            'update_rate': 10,
        }],
        output='screen'
    )
    
    # Navigation bridge node
    navigation_bridge_node = Node(
        package='navigation_bridge',
        executable='navigation_bridge_node',
        name='navigation_bridge',
        parameters=[{
            'navigation_topic': navigation_topic,
        }],
        output='screen'
    )
    
    # System metrics node
    system_metrics_node = Node(
        package='diagnostic_bridge',
        executable='system_metrics_node',
        name='system_metrics_node',
        parameters=[{
            'robot_id': robot_id,
        }],
        output='screen'
    )
    
    # ROS node status node
    ros_node_status_node = Node(
        package='diagnostic_bridge',
        executable='ros_node_status_node',
        name='ros_node_status_node',
        parameters=[{
            'robot_id': robot_id,
        }],
        output='screen'
    )
    
    # Create and return launch description
    return LaunchDescription([
        declare_video_topic,
        declare_audio_topic,
        declare_teleop_topic,
        declare_sensor_topic,
        declare_navigation_topic,
        declare_diagnostic_period,
        video_bridge_node,
        audio_bridge_node,
        teleop_bridge_node,
        sensor_bridge_node,
        navigation_bridge_node,
        system_metrics_node,
        ros_node_status_node
    ]) 