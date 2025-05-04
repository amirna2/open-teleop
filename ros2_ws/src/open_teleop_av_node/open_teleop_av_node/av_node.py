#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
# import time # No longer needed
import uuid # For generating stream IDs
import functools # For passing args to subscriber callback

# Import custom logger
import open_teleop_logger as log

# Import message types
from std_msgs.msg import String # Placeholder
from sensor_msgs.msg import Image # Assuming Image for now, might need CompressedImage later
from open_teleop_msgs.msg import EncodedFrame, StreamStatus
from open_teleop_msgs.srv import ManageStream, GetStatus
from builtin_interfaces.msg import Time # For header timestamp

# Import GStreamer and GLib
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class GstPipeline:
    """Manages a GStreamer pipeline for video encoding."""
    
    def __init__(self, logger, stream_id, input_topic, output_topic, encoding_format, encoder_params=None):
        self.logger = logger
        self.stream_id = stream_id
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.encoding_format = encoding_format
        self.encoder_params = encoder_params or {}
        
        self.pipeline = None
        self.appsrc = None
        self.appsink = None
        self.encoder = None
        self.ros_subscriber = None
        self.status = 'CREATED'
        self.caps_set = False
        self.width = 0
        self.height = 0
        
        # Create pipeline
        self._create_pipeline()
    
    def _create_pipeline(self):
        """Creates the GStreamer pipeline with all elements."""
        try:
            self.pipeline = Gst.Pipeline.new(f"pipeline_{self.stream_id}")
            
            # Create elements
            self.appsrc = Gst.ElementFactory.make("appsrc", f"appsrc_{self.stream_id}")
            videoconvert = Gst.ElementFactory.make("videoconvert", f"vconv_{self.stream_id}")
            
            # Select encoder based on encoding_format
            # Currently only supporting H.264
            self.encoder = Gst.ElementFactory.make("x264enc", f"enc_{self.stream_id}")
            
            self.appsink = Gst.ElementFactory.make("appsink", f"appsink_{self.stream_id}")
            if self.appsink:
                self.appsink.set_property("max-buffers", 5) 
                self.appsink.set_property("drop", False)
                self.appsink.set_property("sync", False)
                self.appsink.set_property("emit-signals", True)

            if not all([self.pipeline, self.appsrc, videoconvert, self.encoder, self.appsink]):
                raise RuntimeError(f"Failed to create one or more GStreamer elements for {self.stream_id}")
            
            self.logger.debug(f"Elements created for {self.stream_id}")

            # Add elements to pipeline
            self.pipeline.add(self.appsrc)
            self.pipeline.add(videoconvert)
            self.pipeline.add(self.encoder)
            self.pipeline.add(self.appsink)

            # Link elements
            if not self.appsrc.link(videoconvert):
                raise RuntimeError(f"Failed to link appsrc to videoconvert for {self.stream_id}")
            if not videoconvert.link(self.encoder):
                raise RuntimeError(f"Failed to link videoconvert to encoder for {self.stream_id}")
            if not self.encoder.link(self.appsink):
                raise RuntimeError(f"Failed to link encoder to appsink for {self.stream_id}")
                
            self.logger.info(f"GStreamer elements linked successfully for {self.stream_id}")
            
        except Exception as e:
            self.logger.error(f"Failed to create pipeline: {e}")
            if self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)
            raise
    
    def set_state(self, state):
        """Set the pipeline state (PLAYING, PAUSED, READY, NULL)."""
        if not self.pipeline:
            self.logger.error(f"Cannot set state for {self.stream_id}: Pipeline not initialized")
            return Gst.StateChangeReturn.FAILURE
            
        ret = self.pipeline.set_state(state)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.logger.error(f"Failed to set pipeline {self.stream_id} to {state.value_name}")
        elif ret == Gst.StateChangeReturn.ASYNC:
            self.logger.info(f"Pipeline {self.stream_id} state change to {state.value_name} is ASYNC")
            if state == Gst.State.PLAYING:
                self.status = 'PLAYING_ASYNC'
            elif state == Gst.State.PAUSED:
                self.status = 'PAUSED_ASYNC'
        else:
            self.logger.info(f"Pipeline {self.stream_id} state set to {state.value_name}")
            if state == Gst.State.PLAYING:
                self.status = 'PLAYING'
            elif state == Gst.State.PAUSED:
                self.status = 'PAUSED'
            elif state == Gst.State.NULL:
                self.status = 'STOPPED'
        
        return ret
    
    def push_ros_image(self, msg):
        """Process a ROS Image message and push it into the pipeline."""
        if not self.appsrc:
            self.logger.error(f"Cannot push image: appsrc not initialized for {self.stream_id}")
            return False
            
        # Configure caps if not already set
        if not self.caps_set:
            if not self._set_caps_from_image(msg):
                return False
        
        # Create buffer from ROS Image data
        try:
            buffer = Gst.Buffer.new_wrapped(bytes(msg.data))
            
            # Set timestamp
            buffer.pts = msg.header.stamp.sec * Gst.SECOND + msg.header.stamp.nanosec
            buffer.dts = Gst.CLOCK_TIME_NONE
            
            # Push buffer
            ret = self.appsrc.emit("push-buffer", buffer)
            if ret != Gst.FlowReturn.OK:
                flow_return_str = Gst.flow_return_get_name(ret)
                self.logger.error(f"push-buffer failed for {self.stream_id}: {flow_return_str} ({ret})")
                return False
                
            return True
        except Exception as e:
            self.logger.error(f"Error pushing image to pipeline: {e}")
            return False
    
    def _set_caps_from_image(self, msg):
        """Set GStreamer caps based on ROS Image message."""
        try:
            # Map ROS encoding to GStreamer format
            encoding_map = {
                'rgb8': 'RGB',
                'rgba8': 'RGBA',
                'bgr8': 'BGR',
                'bgra8': 'BGRA',
                'mono8': 'GRAY8',
                'mono16': 'GRAY16_LE',
                '16UC1': 'GRAY16_LE',
                '32FC1': 'GRAY32_FLOAT_LE'
            }
            
            gst_format = encoding_map.get(msg.encoding)
            if not gst_format:
                self.logger.error(f"Unsupported ROS image encoding: {msg.encoding}")
                return False
                
            framerate = "30/1"
            
            caps_str = (
                f"video/x-raw,"
                f"format={gst_format},"
                f"width={msg.width},"
                f"height={msg.height},"
                f"framerate={framerate}"
            )
            
            caps = Gst.Caps.from_string(caps_str)
            if not caps:
                self.logger.error(f"Failed to parse caps string: {caps_str}")
                return False
                
            self.appsrc.set_property("caps", caps)
            self.caps_set = True
            self.width = msg.width
            self.height = msg.height
            self.logger.info(f"Set caps for {self.stream_id}: {caps.to_string()}")
            return True
            
        except Exception as e:
            self.logger.error(f"Error setting caps: {e}")
            return False
    
    def cleanup(self):
        """Clean up resources used by this pipeline."""
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)


class OpenTeleopAvNode(Node):
    """ROS2 Node for handling A/V encoding and service calls."""

    def __init__(self):
        """Node initialization."""
        super().__init__('open_teleop_av_node')

        # Check environment *before* initializing Gst
        gst_debug_env = os.environ.get('GST_DEBUG')
        # Use standard print here initially, as logger might not be fully configured
        # and output might be redirected differently than logger
        print(f"[AV_NODE PRE-GST_INIT] GST_DEBUG environment variable: {gst_debug_env}")

        # Initialize GStreamer
        try:
            Gst.init(None)
        except Exception as e:
             # Use standard logger temporarily if custom one fails
             self.get_logger().error(f"Failed to initialize GStreamer: {e}")
             # Decide how to handle this - potentially raise an exception or shutdown
             raise RuntimeError("GStreamer initialization failed") from e

        # --- Logger Setup ---
        # Declare parameters for logger configuration
        self.declare_parameter('log.level', 'DEBUG')
        self.declare_parameter('log.path', '/tmp/open_teleop_logs') # Default path
        self.declare_parameter('log.to_file', False)
        self.declare_parameter('log.rotation_days', 7)

        self._configure_logging() # Configure logger based on parameters

        # Log it again using the configured logger for confirmation
        self.logger.debug(f"GST_DEBUG env var check via logger: {os.environ.get('GST_DEBUG')}")

        self.logger.info(f'{self.get_name()} starting up...')

        self.pipelines = {}

        # Create Publisher for EncodedFrame messages
        # TODO: Add QoS profile? Default is usually reliable, keep_last(10)
        self.encoded_frame_publisher = self.create_publisher(
            EncodedFrame, 
            '~/encoded_frames', 
            10 # QoS history depth
        )
        self.logger.info(f"Created publisher for {EncodedFrame.__name__} on '{self.encoded_frame_publisher.topic_name}'")

        # Setup Service Servers (ManageStream, GetStatus)
        self.manage_stream_service = self.create_service(ManageStream, '~/manage_stream', self.handle_manage_stream)
        self.get_status_service = self.create_service(GetStatus, '~/get_status', self.handle_get_status)

        # TODO: Setup Subscribers (to input sensor_msgs/Image topics - likely managed dynamically)
        # Moved subscriber creation into handle_manage_stream

        # TODO: Placeholder timer for testing - remove if no longer needed
        self.timer = self.create_timer(5.0, self.timer_callback) # Increase interval

        self.logger.info(f'{self.get_name()} initialized.')

    def timer_callback(self):
        # Placeholder action - maybe log pipeline count?
        self.logger.debug(f"Timer tick: {len(self.pipelines)} active pipelines.")
        pass

    # --- Service Callbacks ---
    def handle_manage_stream(self, request: ManageStream.Request, response: ManageStream.Response):
        """Handle stream management service requests."""
        self.logger.info("--- Entered handle_manage_stream ---")
        self.logger.info(f"ManageStream request received: Action={request.action}, StreamID='{request.stream_id}'")
        
        if request.action in [ManageStream.Request.ACTION_ADD, ManageStream.Request.ACTION_UPDATE]:
             self.logger.info(
                 f"  Input='{request.input_ros_topic}', Output='{request.output_ott_topic}', " 
                 f"Format='{request.encoding_format}', Params='{request.encoder_params}'"
             )

        self.logger.debug(f"Full request object representation: {request}")

        if request.action == ManageStream.Request.ACTION_ADD:
            return self._handle_add_stream(request, response)
        elif request.action == ManageStream.Request.ACTION_REMOVE:
            return self._handle_remove_stream(request, response)
        elif request.action == ManageStream.Request.ACTION_ENABLE:
            return self._handle_enable_stream(request, response)
        elif request.action == ManageStream.Request.ACTION_DISABLE:
            return self._handle_disable_stream(request, response)
        elif request.action == ManageStream.Request.ACTION_UPDATE:
            response.success = False
            response.message = "ACTION_UPDATE not implemented yet."
        else:
            response.success = False
            response.message = f"Unknown action code: {request.action}"
            self.logger.error(response.message)
            
        self.logger.info("--- Exiting handle_manage_stream ---")
        return response

    def handle_get_status(self, request: GetStatus.Request, response: GetStatus.Response):
        self.logger.info('GetStatus request received')
        # TODO: Implement logic to gather status from managed pipelines
        
        # Placeholder status
        response.node_status = "OK (placeholder)"
        response.active_streams = []
        # Populate from self.pipelines dictionary
        for stream_id, pipeline in self.pipelines.items():
            status_msg = StreamStatus(
                stream_id=stream_id,
                input_ros_topic=pipeline.input_topic,
                output_ott_topic=pipeline.output_topic,
                # TODO: Get actual encoding format used
                encoding_format="video/h264_placeholder", 
                is_enabled=(pipeline.status == 'PLAYING'), # Simplified status check
                status_message=pipeline.status,
                # TODO: Get actual runtime metrics if needed
                frame_rate_actual=0.0, 
                bitrate_actual=0
            )
            response.active_streams.append(status_msg)
        
        return response

    # --- GStreamer Pipeline Management (Example placeholders) ---
    # def add_pipeline(...): # Now handled within manage_stream
    #     pass

    # def remove_pipeline(...): # Will be handled within manage_stream
    #     pass
    
    # --- Appsink Callback ---
    def on_new_sample(self, appsink: Gst.Element, stream_id: str):
        """Callback executed when appsink receives a new sample."""
        sample = appsink.emit("pull-sample")
        if sample:
            buffer = sample.get_buffer()
            if buffer:
                # --- Extract data from buffer --- 
                pts = buffer.pts # In nanoseconds
                is_delta_unit = buffer.has_flags(Gst.BufferFlags.DELTA_UNIT)
                frame_type_enum = EncodedFrame.FRAME_TYPE_DELTA if is_delta_unit else EncodedFrame.FRAME_TYPE_KEY
                frame_type_str = "Delta" if is_delta_unit else "Key"

                # Map buffer to read data
                success, map_info = buffer.map(Gst.MapFlags.READ)
                if not success:
                    self.logger.error(f"Appsink ('{stream_id}'): Failed to map buffer")
                    return Gst.FlowReturn.ERROR
                    
                buffer_size = map_info.size
                # IMPORTANT: Copy the data immediately, map_info.data is a temporary view
                encoded_bytes = map_info.data[:] # Create a copy
                buffer.unmap(map_info) # Unmap ASAP
                # -------------------------------

                self.logger.debug(
                    f"Appsink ('{stream_id}'): Received sample. "
                    f"PTS={pts/Gst.SECOND:.3f}s, Size={buffer_size} bytes, Type={frame_type_str}"
                )
                
                # --- Get pipeline state info --- 
                if stream_id not in self.pipelines:
                    self.logger.error(f"Appsink ('{stream_id}'): Pipeline state not found! Cannot publish.")
                    # Sample cleanup should happen automatically?
                    return Gst.FlowReturn.OK # Or Error? Let's return OK to avoid pipeline issues
                
                pipeline = self.pipelines[stream_id]
                ott_topic = pipeline.output_topic
                width = pipeline.width
                height = pipeline.height
                # Use the encoding format requested by the user, assuming encoder matches
                encoding_format = pipeline.encoding_format 
                
                if not ott_topic:
                     self.logger.error(f"Appsink ('{stream_id}'): Output OTT topic not found in state! Cannot publish.")
                     return Gst.FlowReturn.OK
                # -------------------------------

                # --- Create and Publish EncodedFrame Message --- 
                try:
                    msg = EncodedFrame()
                    
                    # Convert GStreamer nanosecond PTS to ROS Time
                    ros_time = Time()
                    ros_time.sec = pts // Gst.SECOND # Integer division for seconds
                    ros_time.nanosec = pts % Gst.SECOND
                    msg.header.stamp = ros_time
                    # TODO: Set header.frame_id? Maybe from original ROS message?
                    
                    msg.ott_topic = ott_topic
                    msg.encoding_format = encoding_format
                    msg.frame_type = frame_type_enum
                    msg.width = width
                    msg.height = height
                    # Assign the copied data
                    # msg.data = encoded_bytes # This needs to be list/array of uint8, not bytes
                    msg.data = list(encoded_bytes)
                    
                    self.encoded_frame_publisher.publish(msg)
                    self.logger.debug(f"Published EncodedFrame for '{stream_id}' to '{ott_topic}'")

                except Exception as pub_e:
                    self.logger.error(f"Appsink ('{stream_id}'): Failed to create/publish EncodedFrame: {pub_e}")
                # ------------------------------------------------

            else:
                 self.logger.warning(f"Appsink ('{stream_id}'): Failed to get buffer from sample")
                 return Gst.FlowReturn.ERROR # Treat as error if buffer invalid

            return Gst.FlowReturn.OK
        else:
            self.logger.warning(f"Appsink ('{stream_id}'): pull_sample() returned None")
            return Gst.FlowReturn.ERROR

    # --- ROS Topic Callbacks ---
    def ros_image_callback(self, msg: Image, stream_id: str):
        """Callback executed when a ROS Image message arrives for a stream."""
        self.logger.debug(f"ROS Image received for stream '{stream_id}', seq={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, size={msg.width}x{msg.height}, encoding={msg.encoding}")

        # 1. Find the pipeline state for this stream
        if stream_id not in self.pipelines:
            self.logger.error(f"Received image for unknown stream_id '{stream_id}'. Ignoring.")
            return
            
        pipeline = self.pipelines[stream_id]

        if not pipeline.appsrc: # Check only for appsrc now
            self.logger.error(f"Appsrc not found for stream_id '{stream_id}'. Cannot process image.")
            return

        # 3. Configure appsrc caps if not already set
        if not pipeline.caps_set:
            self.logger.info(f"Attempting to set caps for stream '{stream_id}' based on first image.")
            # Map ROS encoding to GStreamer format
            # Based on http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html and GStreamer formats
            encoding_map = {
                'rgb8': 'RGB',
                'rgba8': 'RGBA',
                'bgr8': 'BGR',
                'bgra8': 'BGRA',
                'mono8': 'GRAY8',
                'mono16': 'GRAY16_LE', # Assuming little-endian
                # Add more mappings as needed (e.g., YUV formats, depth formats)
                '16UC1': 'GRAY16_LE', # Common for depth images
                '32FC1': 'GRAY32_FLOAT_LE' # Common for float depth images - need check if supported downstream
            }
            gst_format = encoding_map.get(msg.encoding)
            
            if not gst_format:
                 self.logger.error(f"Unsupported ROS image encoding '{msg.encoding}' for stream '{stream_id}'. Cannot set caps.")
                 # Consider stopping the pipeline or handling this error more robustly
                 return
                 
            # Assume a default framerate if not available? Or make configurable.
            # TODO: Make framerate configurable per stream?
            framerate = "30/1" 
            
            # Construct caps string
            caps_str = (
                f"video/x-raw,"
                f"format={gst_format},"
                f"width={msg.width},"
                f"height={msg.height},"
                f"framerate={framerate}"
            )
            self.logger.debug(f"Constructed caps string for '{stream_id}': {caps_str}")
            
            # Create Gst.Caps object
            caps = Gst.Caps.from_string(caps_str)
            if not caps:
                self.logger.error(f"Failed to parse caps string '{caps_str}' for stream '{stream_id}'")
                # Consider stopping the pipeline
                return
                
            # Set the caps property on appsrc
            pipeline.appsrc.set_property("caps", caps)
            pipeline.caps_set = True # Mark caps as set
            pipeline.width = msg.width # Store dimensions
            pipeline.height = msg.height
            self.logger.info(f"Successfully set appsrc caps for stream '{stream_id}': {caps.to_string()}")
        
        # 4. Convert ROS Image data to Gst.Buffer
        self.logger.debug(f"Attempting to create Gst.Buffer for '{stream_id}'")
        try:
            # Create a Gst.Buffer from the image data bytes
            # Note: msg.data is usually bytes or array.array in ROS2 Python
            buffer = Gst.Buffer.new_wrapped(bytes(msg.data))
        except Exception as e:
            self.logger.error(f"Failed to create Gst.Buffer from ROS Image data for '{stream_id}': {e}")
            return

        # 5. Set timestamp on Gst.Buffer (from msg.header.stamp)
        # GStreamer timestamps are in nanoseconds
        buffer.pts = msg.header.stamp.sec * Gst.SECOND + msg.header.stamp.nanosec
        # DTS (Decoding Time Stamp) is often set to PTS for simple cases
        buffer.dts = Gst.CLOCK_TIME_NONE # Let GStreamer handle DTS if possible, or set to PTS

        # 6. Push buffer into appsrc
        self.logger.debug(f"Attempting push_buffer signal emit for '{stream_id}'")
        # Correct method: emit the 'push-buffer' signal
        ret = pipeline.appsrc.emit("push-buffer", buffer)
        # Check the return value which should be a Gst.FlowReturn
        if ret == Gst.FlowReturn.OK:
            self.logger.debug(f"Successfully pushed buffer for stream '{stream_id}'")
        else:
            # Log the actual FlowReturn enum name for better debugging
            flow_return_str = Gst.flow_return_get_name(ret)
            self.logger.error(f"push-buffer signal failed for stream '{stream_id}' with return code: {flow_return_str} ({ret})")

    def destroy_node(self):
        self.logger.info(f'{self.get_name()} shutting down...')
        # Clean up GStreamer resources (pipelines, etc.)
        for stream_id, pipeline in self.pipelines.items():
            self.logger.info(f"Stopping pipeline for stream {stream_id}...")
            pipeline.cleanup()
            if pipeline.ros_subscriber:
                try:
                    self.destroy_subscription(pipeline.ros_subscriber)
                except Exception as e:
                    self.logger.error(f"Error destroying subscriber for {stream_id}: {e}")
        self.pipelines.clear()
        self.logger.info("All pipelines stopped and cleaned.")
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

    def _handle_add_stream(self, request, response):
        """Handle ACTION_ADD request."""
        # Validate inputs
        if not all([request.input_ros_topic, request.output_ott_topic, request.encoding_format]):
            response.success = False
            response.message = "ACTION_ADD requires input_ros_topic, output_ott_topic, and encoding_format."
            self.logger.error(response.message)
            return response

        # Generate/Assign Stream ID
        stream_id = request.stream_id
        if not stream_id:
            stream_id = str(uuid.uuid4())
            self.logger.info(f"No stream_id provided, generated: {stream_id}")
        
        response.assigned_stream_id = stream_id

        # Check for collision
        if stream_id in self.pipelines:
            response.success = False
            response.message = f"Stream ID '{stream_id}' already exists."
            self.logger.error(response.message)
            return response

        self.logger.info(f"Attempting to add pipeline for stream ID: {stream_id}")

        # Create GStreamer Pipeline
        try:
            # Create new pipeline object
            pipeline = GstPipeline(
                self.logger,
                stream_id,
                request.input_ros_topic,
                request.output_ott_topic,
                request.encoding_format,
                request.encoder_params
            )
            
            # Store pipeline in our dictionary FIRST before connecting signals and subscribers
            self.pipelines[stream_id] = pipeline
            self.logger.info(f"Pipeline {stream_id} added to active pipelines dictionary.")
            
            # Connect appsink signal
            pipeline.appsink.connect("new-sample", self.on_new_sample, stream_id)
            
            # Create ROS subscriber
            callback_with_id = functools.partial(self.ros_image_callback, stream_id=stream_id)
            ros_subscriber = self.create_subscription(
                Image,
                request.input_ros_topic,
                callback_with_id,
                10
            )
            pipeline.ros_subscriber = ros_subscriber
            
            # Set initial state to PAUSED
            ret = pipeline.set_state(Gst.State.PAUSED)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError(f"Failed to set pipeline {stream_id} to PAUSED state.")
            
            response.success = True
            response.message = f"Pipeline for stream '{stream_id}' created and paused successfully."
            
        except Exception as e:
            self.logger.error(f"Failed during pipeline creation/setup for stream '{stream_id}': {e}")
            # Clean up ROS subscriber if created
            if 'ros_subscriber' in locals() and ros_subscriber:
                try:
                    self.destroy_subscription(ros_subscriber)
                except Exception as destroy_e:
                    self.logger.error(f"Error destroying subscriber during cleanup: {destroy_e}")
            
            # Remove the pipeline from the dictionary if it was added
            if stream_id in self.pipelines:
                del self.pipelines[stream_id]
                
            response.success = False
            response.message = f"Failed to create/setup pipeline: {e}"
        
        return response

    def _handle_remove_stream(self, request, response):
        """Handle ACTION_REMOVE request."""
        stream_id = request.stream_id
        if not stream_id:
            response.success = False
            response.message = "ACTION_REMOVE requires a valid stream_id."
            self.logger.error(response.message)
            return response

        if stream_id not in self.pipelines:
            response.success = False
            response.message = f"Stream ID '{stream_id}' not found for ACTION_REMOVE."
            self.logger.error(response.message)
            return response
        
        pipeline = self.pipelines[stream_id]
        
        # Stop the pipeline
        self.logger.info(f"Setting pipeline state to NULL for {stream_id}")
        pipeline.set_state(Gst.State.NULL)
        
        # Destroy the subscriber
        if pipeline.ros_subscriber:
            self.logger.info(f"Destroying ROS subscriber for {stream_id}")
            try:
                self.destroy_subscription(pipeline.ros_subscriber)
            except Exception as destroy_e:
                self.logger.error(f"Error destroying subscriber during REMOVE for {stream_id}: {destroy_e}")
        
        # Remove from our state dictionary
        del self.pipelines[stream_id]
        self.logger.info(f"Removed pipeline state for {stream_id}")
        
        response.success = True
        response.message = f"Stream '{stream_id}' removed successfully."
        return response

    def _handle_enable_stream(self, request, response):
        """Handle ACTION_ENABLE request."""
        stream_id = request.stream_id
        if not stream_id:
            response.success = False
            response.message = "ACTION_ENABLE requires a valid stream_id."
            self.logger.error(response.message)
            return response

        if stream_id not in self.pipelines:
            response.success = False
            response.message = f"Stream ID '{stream_id}' not found for ACTION_ENABLE."
            self.logger.error(response.message)
            return response
        
        pipeline = self.pipelines[stream_id]
        
        # Set state to PLAYING
        ret = pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.logger.error(f"Failed to set pipeline {stream_id} to PLAYING state.")
            response.success = False
            response.message = f"Failed to set pipeline state to PLAYING."
        else:
            response.success = True
            if ret == Gst.StateChangeReturn.ASYNC:
                response.message = f"Pipeline {stream_id} state change to PLAYING initiated asynchronously."
            else:
                response.message = f"Pipeline {stream_id} state set to PLAYING."
        
        return response

    def _handle_disable_stream(self, request, response):
        """Handle ACTION_DISABLE request."""
        stream_id = request.stream_id
        if not stream_id:
            response.success = False
            response.message = "ACTION_DISABLE requires a valid stream_id."
            self.logger.error(response.message)
            return response

        if stream_id not in self.pipelines:
            response.success = False
            response.message = f"Stream ID '{stream_id}' not found for ACTION_DISABLE."
            self.logger.error(response.message)
            return response
        
        pipeline = self.pipelines[stream_id]
        
        # Set state to PAUSED
        ret = pipeline.set_state(Gst.State.PAUSED)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.logger.error(f"Failed to set pipeline {stream_id} to PAUSED state.")
            response.success = False
            response.message = f"Failed to set pipeline state to PAUSED."
        else:
            response.success = True
            if ret == Gst.StateChangeReturn.ASYNC:
                response.message = f"Pipeline {stream_id} state change to PAUSED initiated asynchronously."
            else:
                response.message = f"Pipeline {stream_id} state set to PAUSED."
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OpenTeleopAvNode()
        # Initialize GObject/GLib main loop integration with ROS event loop
        # This allows GStreamer signals (like bus messages, appsink signals) to be processed.
        # TODO: Add a proper GLib main loop integration if needed, or ensure rclpy spin handles it.
        # For now, relying on rclpy.spin which might be sufficient for basic callbacks.
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