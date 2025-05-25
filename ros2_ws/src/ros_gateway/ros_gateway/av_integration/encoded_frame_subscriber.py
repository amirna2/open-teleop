#!/usr/bin/env python3
# encoded_frame_subscriber.py

import time
import flatbuffers
import json
from rclpy.node import Node
from open_teleop_msgs.msg import EncodedFrame
from open_teleop_msgs.srv import ManageStream, GetStatus

# Import the generated FlatBuffer code
from ros_gateway.flatbuffers.open_teleop.message import ContentType
from ros_gateway.flatbuffers.open_teleop.message.FrameType import FrameType
from ros_gateway.flatbuffers.open_teleop.message.OttMessage import (
    OttMessageStart,
    OttMessageAddVersion,
    OttMessageAddPayload,
    OttMessageAddContentType,
    OttMessageAddOtt,
    OttMessageAddTimestampNs,
    OttMessageAddVideoMetadata,
    OttMessageEnd
)
from ros_gateway.flatbuffers.open_teleop.message.VideoFrameMetadata import (
    VideoFrameMetadataStart,
    VideoFrameMetadataAddSequenceNumber,
    VideoFrameMetadataAddOriginalTimestampNs,
    VideoFrameMetadataAddFrameType,
    VideoFrameMetadataAddEncodingFormat,
    VideoFrameMetadataAddWidth,
    VideoFrameMetadataAddHeight,
    VideoFrameMetadataAddFrameId,
    VideoFrameMetadataEnd
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
        
        # Sequence number tracking per stream
        self.sequence_numbers = {}  # Key: ott_topic, Value: current sequence number
        
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
        self.manage_stream_client = None
        self.get_status_client = None
        
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
             self.manage_stream_client = None
        
        # Try GetStatus client
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
        except Exception as e:
             self.logger.error(f"!!! Exception during GetStatus client creation: {e}")
             self.get_status_client = None
        
        # Check if the critical client was created successfully before proceeding
        if self.manage_stream_client is None:
             self.logger.error("ManageStream client was not created successfully. AV stream configuration will likely fail.")
            
        self.streams = {}  # To track configured streams
        self.logger.info("EncodedFrameSubscriber initialized (client creation attempted).")
    
    def _get_next_sequence_number(self, ott_topic):
        """Get the next sequence number for a given stream."""
        if ott_topic not in self.sequence_numbers:
            self.sequence_numbers[ott_topic] = 0
        self.sequence_numbers[ott_topic] += 1
        return self.sequence_numbers[ott_topic]
    
    def handle_encoded_frame(self, msg):
        """
        Handle an EncodedFrame message from the AV node.
        
        Args:
            msg: The EncodedFrame message
        """
        self.logger.debug(f"Received EncodedFrame for OTT topic: {msg.ott_topic}")
        
        try:
            # Get current timestamp in nanoseconds (Gateway processing time)
            gateway_timestamp_ns = self.node.get_clock().now().nanoseconds
            
            # Extract metadata from EncodedFrame
            original_timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            sequence_number = self._get_next_sequence_number(msg.ott_topic)
            frame_id = msg.header.frame_id if msg.header.frame_id else ""
            
            # Map ROS frame type to FlatBuffer frame type
            if msg.frame_type == EncodedFrame.FRAME_TYPE_KEY:
                fb_frame_type = FrameType.FRAME_TYPE_KEY
            elif msg.frame_type == EncodedFrame.FRAME_TYPE_DELTA:
                fb_frame_type = FrameType.FRAME_TYPE_DELTA
            else:
                fb_frame_type = FrameType.FRAME_TYPE_UNKNOWN
            
            # Create OTT FlatBuffer message with metadata
            builder = flatbuffers.Builder(1024 + len(msg.data))
            
            # Create string fields
            ott_fb = builder.CreateString(msg.ott_topic)
            encoding_format_fb = builder.CreateString(msg.encoding_format)
            frame_id_fb = builder.CreateString(frame_id)
            
            # Create VideoFrameMetadata
            VideoFrameMetadataStart(builder)
            VideoFrameMetadataAddSequenceNumber(builder, sequence_number)
            VideoFrameMetadataAddOriginalTimestampNs(builder, original_timestamp_ns)
            VideoFrameMetadataAddFrameType(builder, fb_frame_type)
            VideoFrameMetadataAddEncodingFormat(builder, encoding_format_fb)
            VideoFrameMetadataAddWidth(builder, msg.width)
            VideoFrameMetadataAddHeight(builder, msg.height)
            VideoFrameMetadataAddFrameId(builder, frame_id_fb)
            video_metadata = VideoFrameMetadataEnd(builder)
            
            # Create the payload byte vector
            payload = builder.CreateByteVector(bytes(msg.data))
            
            # Start building the OttMessage
            OttMessageStart(builder)
            OttMessageAddVersion(builder, 1)
            OttMessageAddPayload(builder, payload)
            OttMessageAddContentType(builder, ContentType.ENCODED_VIDEO_FRAME)
            OttMessageAddOtt(builder, ott_fb)
            OttMessageAddTimestampNs(builder, gateway_timestamp_ns)
            OttMessageAddVideoMetadata(builder, video_metadata)
            
            # Finish the message
            message = OttMessageEnd(builder)
            builder.Finish(message)
            
            # Get the binary buffer and send it via ZeroMQ
            buf = builder.Output()
            
            # Send to controller
            self.logger.debug(f"Sending encoded frame ({len(buf)} bytes) for {msg.ott_topic} "
                            f"[seq={sequence_number}, type={'KEY' if fb_frame_type == FrameType.FRAME_TYPE_KEY else 'DELTA'}, "
                            f"{msg.width}x{msg.height}, {msg.encoding_format}]")
            reply_str = self.zmq_client.send_request_binary(buf)
            
            # Log the reply (or lack thereof)
            if reply_str:
                self.logger.debug(f"Reply from ZeroMQ: {reply_str}")
            
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
        
        # Topic ID (previously stream_id) is mandatory in config for AV streams
        try:
            # Use 'topic_id' from the config mapping
            topic_id = stream_config['topic_id'] 
            if not topic_id: # Basic check for empty string
                raise ValueError("topic_id cannot be empty")
            # Assign it to the service request's stream_id field
            request.stream_id = topic_id 
        except KeyError:
            self.logger.error(f"AV Stream config missing mandatory 'topic_id': {stream_config}")
            return False, "Missing mandatory 'topic_id' in configuration"
        except ValueError as e:
             self.logger.error(f"Invalid 'topic_id' in AV stream config: {e}. Config: {stream_config}")
             return False, f"Invalid topic_id: {e}"
        
        # Set required fields
        request.input_ros_topic = stream_config['ros_topic']
        request.output_ott_topic = stream_config['ott']
        
        # Set encoder parameters if provided
        if 'encoder_params' in stream_config:
            try:
                encoder_params = stream_config['encoder_params']
                
                # Directly set the dedicated encoding_format field
                request.encoding_format = encoder_params['encoding_format']
                
                # Create a dictionary for the JSON blob, excluding encoding_format
                params_for_json = {
                    k: v for k, v in encoder_params.items() 
                    if k != 'encoding_format'
                }
                
                # Set the remaining encoder_params as a JSON string
                request.encoder_params = json.dumps(params_for_json)
                
            except KeyError as e:
                self.logger.error(f"Missing required key in encoder_params: {e}. Config validation failed?")
                return False, f"Missing required encoder_params key: {e}"
            except Exception as e:
                self.logger.error(f"Error processing encoder_params: {e}")
                return False, f"Error processing encoder_params: {e}"
        else:
            self.logger.error("Stream config is missing the required 'encoder_params' field.")
            return False, "Missing required 'encoder_params' field"
        
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
        """Clean up resources and disable active streams."""
        self.logger.info("Shutting down EncodedFrameSubscriber...")
        
        # Perform synchronous removal of active streams
        if hasattr(self, 'streams') and self.streams and self.manage_stream_client:
            stream_ids = list(self.streams.keys())
            self.logger.info(f"Removing {len(stream_ids)} active AV streams during shutdown: {stream_ids}")
            
            for stream_id in stream_ids:
                try:
                    # Create and send a synchronous REMOVE request
                    request = ManageStream.Request()
                    request.action = ManageStream.Request.ACTION_REMOVE
                    request.stream_id = stream_id
                    
                    # Use synchronous call with short timeout
                    self.logger.info(f"Sending synchronous REMOVE request for stream: {stream_id}")
                    future = self.manage_stream_client.call_async(request)
                    
                    # Wait with timeout
                    if hasattr(self.node, 'executor') and self.node.executor:
                        try:
                            # Spin until future completes or timeout (max 0.5 seconds per stream)
                            spin_count = 0
                            max_spins = 10  # ~0.5 seconds with 0.05s per spin
                            while not future.done() and spin_count < max_spins:
                                self.node.executor.spin_once(timeout_sec=0.05)
                                spin_count += 1
                            
                            if future.done():
                                result = future.result()
                                if result.success:
                                    self.logger.info(f"Successfully removed stream {stream_id} during shutdown")
                                else:
                                    self.logger.warning(f"AV node reported error removing stream {stream_id}: {result.message}")
                            else:
                                self.logger.warning(f"Timeout waiting for stream {stream_id} removal response")
                        except Exception as e:
                            self.logger.warning(f"Error spinning executor for stream {stream_id} removal: {e}")
                except Exception as e:
                    self.logger.warning(f"Error removing stream {stream_id} during shutdown: {e}")
            
            # Clear the streams dictionary regardless of success
            self.streams.clear()
        
        # Clean up ROS resources
        if hasattr(self, 'encoded_frame_subscription'):
            try:
                self.node.destroy_subscription(self.encoded_frame_subscription)
                self.logger.info("Destroyed encoded frame subscription")
            except Exception as e:
                self.logger.warning(f"Error destroying subscription: {e}")
        
        # Clean up service clients
        if hasattr(self, 'manage_stream_client') and self.manage_stream_client:
            try:
                self.manage_stream_client.destroy()
                self.logger.info("Destroyed manage stream client")
            except Exception as e:
                self.logger.warning(f"Error destroying manage stream client: {e}")
        
        if hasattr(self, 'get_status_client') and self.get_status_client:
            try:
                self.get_status_client.destroy()
                self.logger.info("Destroyed get status client")
            except Exception as e:
                self.logger.warning(f"Error destroying get status client: {e}")
        
        self.logger.info("EncodedFrameSubscriber shutdown complete") 