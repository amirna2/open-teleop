#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
# import time # No longer needed

# Import custom logger
import open_teleop_logger as log

# Import message types
from std_msgs.msg import String # Placeholder
# from sensor_msgs.msg import Image, CompressedImage
from open_teleop_msgs.msg import EncodedFrame, StreamStatus
from open_teleop_msgs.srv import ManageStream, GetStatus

# Import GStreamer and GLib
# import gi
# gi.require_version('Gst', '1.0')
# from gi.repository import Gst, GLib

class OpenTeleopAvNode(Node):
    """ROS2 Node for handling A/V encoding and service calls."""

    def __init__(self):
        """Node initialization."""
        super().__init__('open_teleop_av_node')

        # --- Logger Setup ---
        # Declare parameters for logger configuration
        self.declare_parameter('log.level', 'DEBUG')
        self.declare_parameter('log.path', '/tmp/open_teleop_logs') # Default path
        self.declare_parameter('log.to_file', False)
        self.declare_parameter('log.rotation_days', 7)

        self._configure_logging() # Configure logger based on parameters

        self.logger.info(f'{self.get_name()} starting up...')

        # TODO: Initialize GStreamer
        # Gst.init(None)

        # TODO: Setup Publishers (for EncodedFrame)
        # self.encoded_frame_publisher = self.create_publisher(EncodedFrame, '~/encoded_frames', 10)

        # Setup Service Servers (ManageStream, GetStatus)
        self.manage_stream_service = self.create_service(ManageStream, '~/manage_stream', self.handle_manage_stream)
        self.get_status_service = self.create_service(GetStatus, '~/get_status', self.handle_get_status)

        # TODO: Setup Subscribers (to input sensor_msgs/Image topics - likely managed dynamically)

        # TODO: Placeholder timer for testing
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.logger.info(f'{self.get_name()} initialized.')

    def timer_callback(self):
        # Placeholder action
        # self.logger.info('Timer tick')
        pass

    # --- Service Callbacks ---
    def handle_manage_stream(self, request: ManageStream.Request, response: ManageStream.Response):
        self.logger.info("--- Entered handle_manage_stream ---")
        # Restore simpler logging
        self.logger.info(f"ManageStream request received: Action={request.action}, StreamID='{request.stream_id}'")
        if request.action in [ManageStream.Request.ACTION_ADD, ManageStream.Request.ACTION_UPDATE]:
             self.logger.info(
                 f"  Input='{request.input_ros_topic}', Output='{request.output_ott_topic}', " 
                 f"Format='{request.encoding_format}', Params='{request.encoder_params}'"
             )

        # Log the full object representation at DEBUG level for completeness
        self.logger.debug(f"Full request object representation: {request}")

        # TODO: Implement GStreamer logic based on request.action

        # Minimal placeholder response (can be expanded later)
        response.success = True
        response.message = "Request received (placeholder logic)"
        response.assigned_stream_id = request.stream_id # Echo back provided ID
        if request.action == ManageStream.Request.ACTION_ADD and not request.stream_id:
            import uuid
            response.assigned_stream_id = str(uuid.uuid4())
            response.message += f", Assigned ID: {response.assigned_stream_id}"
            
        self.logger.info("--- Exiting handle_manage_stream ---")
        return response

    def handle_get_status(self, request: GetStatus.Request, response: GetStatus.Response):
        self.logger.info('GetStatus request received')
        # TODO: Implement logic to gather status from managed pipelines
        
        # Placeholder status
        response.node_status = "OK (placeholder)"
        
        # Example placeholder stream status
        example_status = StreamStatus(
            stream_id="placeholder_stream_1",
            input_ros_topic="/dev/null",
            output_ott_topic="teleop.video.placeholder",
            encoding_format="video/h264",
            is_enabled=False,
            status_message="Placeholder status",
            frame_rate_actual=0.0,
            bitrate_actual=0
        )
        response.active_streams = [example_status]
        
        return response

    # --- GStreamer Pipeline Management (Example placeholders) ---
    # def add_pipeline(...):
    #     # Create Gst elements programmatically
    #     # Link elements
    #     # Add to internal state management
    #     pass

    # def remove_pipeline(...):
    #     # Set pipeline state to NULL
    #     # Remove from internal state
    #     pass

    # --- ROS Topic Callbacks (Example placeholder) ---
    # def image_callback(self, msg, stream_id):
    #     # Called when a new image arrives for a specific stream
    #     # TODO: Push buffer into the corresponding GStreamer appsrc
    #     pass

    def destroy_node(self):
        self.logger.info(f'{self.get_name()} shutting down...')
        # TODO: Clean up GStreamer resources (pipelines, etc.)
        super().destroy_node()

    # --- Helper Methods ---
    def _configure_logging(self):
        """Configure the logger based on ROS parameters."""
        try:
            log_level_str = self.get_parameter('log.level').get_parameter_value().string_value
            log_path = self.get_parameter('log.path').get_parameter_value().string_value
            log_to_file = self.get_parameter('log.to_file').get_parameter_value().bool_value
            # log_rotation = self.get_parameter('log.rotation_days').get_parameter_value().integer_value # Not yet used by logger module

            # Map string level to logging level constant
            log_level = getattr(log, log_level_str.upper(), log.INFO)

            # Configure the logger
            self.logger = log.get_logger(
                name=self.get_name(),
                log_dir=log_path if log_to_file else None,
                console_level=log_level,
                file_level=log_level if log_to_file else None
            )
            # Initial log message might be lost if reconfiguration happens after initial log calls
            self.get_logger().info(f"Logger configured: Level={log_level_str}, FileOutput={log_to_file}, Path={log_path if log_to_file else 'N/A'}")

        except Exception as e:
            self.get_logger().error(f"Error configuring logger from parameters: {e}. Using basic configuration.")
            # Fallback basic configuration
            self.logger = log.get_logger(self.get_name())

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OpenTeleopAvNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"Unhandled exception: {e}")
            import traceback
            node.get_logger().error(traceback.format_exc())
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 