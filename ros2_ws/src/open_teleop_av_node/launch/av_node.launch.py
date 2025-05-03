from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generates the launch description for the Open Teleop AV Node."""
    return LaunchDescription([
        Node(
            package='open_teleop_av_node',
            executable='av_node',
            name='open_teleop_av_node',
            output='screen',
            emulate_tty=True, # Ensures logger formatting works
            parameters=[
                # TODO: Add parameter file loading if needed
                # {'config_file': 'path/to/your/config.yaml'}
            ]
        )
    ]) 