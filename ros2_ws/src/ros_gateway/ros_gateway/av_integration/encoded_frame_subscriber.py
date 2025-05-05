#!/usr/bin/env python3
# encoded_frame_subscriber.py

import time
import flatbuffers
import json
from rclpy.node import Node
from open_teleop_msgs.msg import EncodedFrame
from open_teleop_msgs.srv import ManageStream, GetStatus

# Import the generated FlatBuffer code
from ros_gateway.flatbuffers.open_teleop.message import ContentType, OttMessage # Import class and enum
# Import builder functions DIRECTLY from the OttMessage module file
from ros_gateway.flatbuffers.open_teleop.message.OttMessage import (
    OttMessageStart,
    OttMessageAddVersion,
    OttMessageAddPayload,
    OttMessageAddContentType,
    OttMessageAddOtt,
    OttMessageAddTimestampNs,
    OttMessageEnd
)

class EncodedFrameSubscriber:
    """
    Subscribes to EncodedFrame messages from the AV node and forwards them to the controller.
    Also provides methods to manage AV node streams.
    """
    
    def __init__(self, node, zmq_client, logger=None):
        """
        Initialize the encoded frame subscriber.
        
        Args:
            node: The ROS node
            zmq_client: The ZeroMQ client to use
            logger: Logger instance
        """
        self.node = node
        self.zmq_client = zmq_client
        self.logger = logger or node.get_logger()
        
        # Create subscription to AV node's encoded frame topic
        self.logger.info("Creating subscription to encoded frames topic...")
        self.encoded_frame_subscription = self.node.create_subscription(
            EncodedFrame,
            '/open_teleop_av_node/encoded_frames',
            self.handle_encoded_frame,
            10  # QoS depth
        )
        
        # Create service clients
        self.logger.info("Creating service clients for AV node...")
        self.manage_stream_client = None # Initialize to None
        self.get_status_client = None # Initialize to None
        
        # Try ManageStream client
        self.logger.info("Attempting to create ManageStream client for service '/open_teleop_av_node/manage_stream'...")
        try:
            self.manage_stream_client = self.node.create_client(
                ManageStream,
                '/open_teleop_av_node/manage_stream'
            )
            if self.manage_stream_client is None:
                 self.logger.error("!!! node.create_client returned None for ManageStream!")
            else:
                 self.logger.info("ManageStream client object created successfully.")
                 # Now wait for it
                 self.logger.info("Waiting up to 5s for ManageStream service to become available...")
                 if not self.manage_stream_client.wait_for_service(timeout_sec=5.0):
                     self.logger.error("!!! ManageStream service wait timed out.")
                 else:
                     self.logger.info("ManageStream service is available.")
        except Exception as e:
             self.logger.error(f"!!! Exception during ManageStream client creation/wait: {e}")
             import traceback
             self.logger.error(traceback.format_exc())
             self.manage_stream_client = None # Ensure it's None on failure
        
        # Try GetStatus client (less critical for startup)
        self.logger.info("Attempting to create GetStatus client for service '/open_teleop_av_node/get_status'...")
        try:
            self.get_status_client = self.node.create_client(
                GetStatus,
                '/open_teleop_av_node/get_status'
            )
            if self.get_status_client is None:
                 self.logger.error("!!! node.create_client returned None for GetStatus!")
            else:
                self.logger.info("GetStatus client object created successfully.")
                # Optional: Add a short wait here if needed, but less critical for init
                # if not self.get_status_client.wait_for_service(timeout_sec=2.0):
                #     self.logger.warning("GetStatus service wait timed out (non-critical).")
                # else:
                #     self.logger.info("GetStatus service is available.")
        except Exception as e:
             self.logger.error(f"!!! Exception during GetStatus client creation: {e}")
             self.get_status_client = None
        
        # Check if the critical client was created successfully before proceeding
        if self.manage_stream_client is None:
             self.logger.error("ManageStream client was not created successfully. AV stream configuration will likely fail.")
             # Depending on requirements, you might want to raise an error here
             # raise RuntimeError("Failed to create critical ManageStream service client")
            
        self.streams = {}  # To track configured streams
        self.logger.info("EncodedFrameSubscriber initialized (client creation attempted).")
    
    def handle_encoded_frame(self, msg):
        """
        Handle an EncodedFrame message from the AV node.
        
        Args:
            msg: The EncodedFrame message
        """
        self.logger.debug(f"Received EncodedFrame for OTT topic: {msg.ott_topic}")
        
        try:
            # Get current timestamp in nanoseconds
            timestamp_ns = self.node.get_clock().now().nanoseconds
            
            # Create FlatBuffer message
            builder = flatbuffers.Builder(1024 + len(msg.data))  # Pre-allocate enough space for the data
            
            # Create the OTT topic string
            ott_fb = builder.CreateString(msg.ott_topic)
            
            # Create the payload byte vector
            payload = builder.CreateByteVector(bytes(msg.data))
            
            # Start building the OttMessage
            OttMessageStart(builder)
            OttMessageAddVersion(builder, 1)
            OttMessageAddPayload(builder, payload)
            OttMessageAddContentType(builder, ContentType.ENCODED_VIDEO_FRAME)
            OttMessageAddOtt(builder, ott_fb)
            OttMessageAddTimestampNs(builder, timestamp_ns)
            
            # NOTE: Media-specific fields are not part of the current OttMessage schema
            # If added to schema later, uncomment and ensure direct function call style:
            # media_format_fb = builder.CreateString(msg.encoding_format)
            # OttMessageAddMediaFormat(builder, media_format_fb)
            # OttMessageAddFrameType(builder, msg.frame_type)
            # OttMessageAddWidth(builder, msg.width)
            # OttMessageAddHeight(builder, msg.height)
            
            # Finish the message
            message = OttMessageEnd(builder)
            builder.Finish(message)
            
            # Get the binary buffer and send it via ZeroMQ
            buf = builder.Output()
            
            # Send to controller with high priority
            self.logger.debug(f"Sending encoded frame ({len(buf)} bytes) for {msg.ott_topic}")
            self.zmq_client.send_request_binary(buf, is_high_priority=True)
            
        except Exception as e:
            self.logger.error(f"Error handling encoded frame: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
    
    async def configure_av_stream(self, stream_config):
        """
        Configure an AV stream according to the provided configuration.
        
        Args:
            stream_config: Stream configuration dictionary
        
        Returns:
            bool: True if successful, False otherwise
            str: Stream ID if successful, error message otherwise
        """
        self.logger.info(f"Configuring AV stream: {stream_config}")
        
        # Wait for the service to be available
        while not self.manage_stream_client.wait_for_service(timeout_sec=1.0):
            self.logger.info('ManageStream service not available, waiting...')
        
        # Create the request
        request = ManageStream.Request()
        request.action = ManageStream.Request.ACTION_ADD
        
        # Stream ID (optional, will be generated if not provided)
        if 'stream_id' in stream_config:
            request.stream_id = stream_config['stream_id']
        
        # Set required fields
        request.input_ros_topic = stream_config['ros_topic']
        request.output_ott_topic = stream_config['ott']
        
        # Set encoder parameters if provided
        encoder_params = {}
        if 'encoder_params' in stream_config:
            try:
                # Parse encoder_params JSON if it's a string
                if isinstance(stream_config['encoder_params'], str):
                    encoder_params = json.loads(stream_config['encoder_params'])
                else:
                    encoder_params = stream_config['encoder_params']
                
                # Extract encoding_format from encoder_params
                if 'encoding_format' in encoder_params:
                    request.encoding_format = encoder_params['encoding_format']
                else:
                    request.encoding_format = 'video/h264'  # Default
                    self.logger.warning("No encoding_format found in encoder_params, using default: video/h264")
                
                # Set the encoder_params as a JSON string
                request.encoder_params = json.dumps(encoder_params)
                
            except json.JSONDecodeError as e:
                self.logger.error(f"Failed to parse encoder_params JSON: {e}")
                return False, f"Invalid encoder_params JSON: {e}"
        else:
            self.logger.warning("No encoder_params provided, using default encoding_format: video/h264")
            request.encoding_format = 'video/h264'  # Default
        
        # Send the request
        self.logger.info(f"Sending ManageStream request: {request}")
        future = self.manage_stream_client.call_async(request)
        
        # Wait for response
        await future
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                stream_id = response.assigned_stream_id
                self.logger.info(f"Stream configured successfully, ID: {stream_id}")
                
                # Store stream info
                self.streams[stream_id] = {
                    'config': stream_config,
                    'status': 'CREATED'
                }
                
                return True, stream_id
            else:
                self.logger.error(f"Failed to configure stream: {response.message}")
                return False, response.message
        else:
            self.logger.error("Service call failed")
            return False, "Service call failed"
    
    async def enable_av_stream(self, stream_id):
        """
        Enable a configured AV stream.
        
        Args:
            stream_id: The ID of the stream to enable
        
        Returns:
            bool: True if successful, False otherwise
            str: Success/error message
        """
        if stream_id not in self.streams:
            return False, f"Stream ID {stream_id} not found"
        
        # Wait for the service to be available
        while not self.manage_stream_client.wait_for_service(timeout_sec=1.0):
            self.logger.info('ManageStream service not available, waiting...')
        
        # Create the request
        request = ManageStream.Request()
        request.action = ManageStream.Request.ACTION_ENABLE
        request.stream_id = stream_id
        
        # Send the request
        self.logger.info(f"Sending ENABLE request for stream: {stream_id}")
        future = self.manage_stream_client.call_async(request)
        
        # Wait for response
        await future
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.logger.info(f"Stream {stream_id} enabled successfully")
                self.streams[stream_id]['status'] = 'ENABLED'
                return True, "Stream enabled"
            else:
                self.logger.error(f"Failed to enable stream {stream_id}: {response.message}")
                return False, response.message
        else:
            self.logger.error("Service call failed")
            return False, "Service call failed"
    
    async def disable_av_stream(self, stream_id):
        """Disable a configured AV stream."""
        if stream_id not in self.streams:
            return False, f"Stream ID {stream_id} not found"
        
        # Create the request
        request = ManageStream.Request()
        request.action = ManageStream.Request.ACTION_DISABLE
        request.stream_id = stream_id
        
        # Send the request
        self.logger.info(f"Sending DISABLE request for stream: {stream_id}")
        future = self.manage_stream_client.call_async(request)
        
        # Wait for response
        await future
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.logger.info(f"Stream {stream_id} disabled successfully")
                self.streams[stream_id]['status'] = 'DISABLED'
                return True, "Stream disabled"
            else:
                self.logger.error(f"Failed to disable stream {stream_id}: {response.message}")
                return False, response.message
        else:
            self.logger.error("Service call failed")
            return False, "Service call failed"
    
    async def remove_av_stream(self, stream_id):
        """Remove a configured AV stream."""
        if stream_id not in self.streams:
            return False, f"Stream ID {stream_id} not found"
        
        # Create the request
        request = ManageStream.Request()
        request.action = ManageStream.Request.ACTION_REMOVE
        request.stream_id = stream_id
        
        # Send the request
        self.logger.info(f"Sending REMOVE request for stream: {stream_id}")
        future = self.manage_stream_client.call_async(request)
        
        # Wait for response
        await future
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.logger.info(f"Stream {stream_id} removed successfully")
                # Remove from our tracking
                del self.streams[stream_id]
                return True, "Stream removed"
            else:
                self.logger.error(f"Failed to remove stream {stream_id}: {response.message}")
                return False, response.message
        else:
            self.logger.error("Service call failed")
            return False, "Service call failed"
    
    async def get_av_status(self):
        """Get the status of all configured AV streams."""
        # Wait for the service to be available
        while not self.get_status_client.wait_for_service(timeout_sec=1.0):
            self.logger.info('GetStatus service not available, waiting...')
        
        # Create the request
        request = GetStatus.Request()
        
        # Send the request
        self.logger.info("Sending GetStatus request")
        future = self.get_status_client.call_async(request)
        
        # Wait for response
        await future
        
        if future.result() is not None:
            response = future.result()
            # Update our internal tracking with the latest status
            for stream_status in response.active_streams:
                if stream_status.stream_id in self.streams:
                    self.streams[stream_status.stream_id]['status'] = stream_status.status_message
            
            return response
        else:
            self.logger.error("GetStatus service call failed")
            return None
    
    def shutdown(self):
        """Clean up resources."""
        if hasattr(self, 'encoded_frame_subscription'):
            self.node.destroy_subscription(self.encoded_frame_subscription)
        self.logger.info("EncodedFrameSubscriber shutdown complete") 