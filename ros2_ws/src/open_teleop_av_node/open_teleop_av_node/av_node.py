#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import uuid
import functools
import json

# Import custom logger
import open_teleop_logger as log

# Import message types
from std_msgs.msg import String
from sensor_msgs.msg import Image
from open_teleop_msgs.msg import EncodedFrame, StreamStatus
from open_teleop_msgs.srv import ManageStream, GetStatus
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Import GStreamer and GLib
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class GstPipeline:
    """Manages a GStreamer pipeline for video encoding."""
    
    H264_ENCODER_CANDIDATES = [
        {
            "name": "nvh264enc", 
            "type": "hardware", 
            "fixed_properties": {
                "preset": 3, # low latency
                "rc-mode": 0, # CBR
                "zerolatency": True,
            },
            "param_mappings": {
                "gop_size": "gop-size", # nvh264enc uses gop-size
                "bitrate": "bitrate" # nvh264enc uses bitrate (kbps)
            }
        },
              {
            "name": "x264enc", 
            "type": "software", 
            "fixed_properties": {
                "tune": "zerolatency", 
            },
            "param_mappings": {
                "gop_size": "key-int-max", # x264enc uses key-int-max
                "bitrate": "bitrate" # x264enc uses bitrate (kbps)
            }
        }
    ]

    def __init__(self, logger, stream_id, input_topic, output_topic, encoding_format, encoder_params=None):
        self.logger = logger
        self.stream_id = stream_id
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.encoding_format = encoding_format
        
        self.parsed_encoder_params = {}
        if isinstance(encoder_params, str):
            try:
                self.parsed_encoder_params = json.loads(encoder_params)
                self.logger.debug(f"Parsed encoder_params for {stream_id}: {self.parsed_encoder_params}")
            except json.JSONDecodeError as e:
                self.logger.error(f"Failed to parse encoder_params JSON for {stream_id}: {e}. Raw params: '{encoder_params}'")
                raise ValueError(f"Invalid encoder_params JSON: {e}") from e
        elif isinstance(encoder_params, dict):
             self.logger.debug(f"encoder_params for {stream_id} was already a dict: {encoder_params}")
             self.parsed_encoder_params = encoder_params
        else:
            self.logger.warning(f"No valid encoder_params provided for {stream_id} or type is {type(encoder_params)}. Using defaults (empty dict).")

        self.pipeline = None
        self.appsrc = None
        self.appsink = None
        self.encoder = None
        self.ros_subscriber = None
        self.status = 'CREATED'
        self.caps_set = False
        self.width = 0
        self.height = 0
        self.h264parser = None # Initialize h264parser
        self.capsfilter = None # Initialize capsfilter
        
        self._create_pipeline()
    
    def _apply_encoder_properties(self, encoder_element, candidate_config, user_params):
        """Applies fixed and user-defined properties to an encoder element, relying on candidate_config.param_mappings."""
        encoder_name = candidate_config["name"]
        self.logger.debug(f"Applying properties to {encoder_name} for stream {self.stream_id}. Candidate: {candidate_config['name']}, User params: {user_params}")

        # 1. Apply fixed properties (defined in H264_ENCODER_CANDIDATES)
        fixed_props = candidate_config.get("fixed_properties", {})
        for prop_name, prop_value in fixed_props.items():
            if encoder_element.find_property(prop_name):
                try:
                    encoder_element.set_property(prop_name, prop_value)
                    self.logger.debug(f"Set {encoder_name} fixed property: {prop_name}={prop_value}")
                except Exception as e_prop:
                    self.logger.warning(f"Failed to set fixed property {prop_name}={prop_value} on {encoder_name}: {e_prop}")
            else:
                self.logger.warning(f"Fixed property '{prop_name}' not found on {encoder_name} (specified in candidate config for {encoder_name}).")

        # 2. Apply user-defined parameters (from encoder_params in service request) using param_mappings
        param_map = candidate_config.get("param_mappings", {})
        
        for user_param_key, user_param_value in user_params.items():
            gst_prop_name = param_map.get(user_param_key) 
            
            if gst_prop_name: 
                if encoder_element.find_property(gst_prop_name):
                    try:
                        actual_value = user_param_value
                        
                        # Convert to int if user_param_value is a digit string and the target GStreamer property is known to be an integer type
                        if isinstance(user_param_value, str) and user_param_value.isdigit():
                            if gst_prop_name in ["bitrate", "gop-size", "key-int-max"]: # Known integer GStreamer properties
                                actual_value = int(user_param_value)
                        # Warn if the type is not a basic one GStreamer usually handles, but still attempt to set.
                        elif not isinstance(user_param_value, (int, float, bool, str)):
                             self.logger.warning(f"User parameter '{user_param_key}' (value: {user_param_value}) has non-basic type {type(user_param_value)}. Using as is for {gst_prop_name}.")
                        
                        encoder_element.set_property(gst_prop_name, actual_value)
                        self.logger.debug(f"Set {encoder_name} user-defined property: {gst_prop_name}={actual_value} (from user param '{user_param_key}')")
                    except Exception as e_user_prop:
                        self.logger.warning(f"Failed to set user-defined property {gst_prop_name} (from '{user_param_key}'='{user_param_value}') on {encoder_name}: {e_user_prop}")
                else:
                    self.logger.warning(f"Mapped GStreamer property '{gst_prop_name}' (for user param '{user_param_key}') not found on {encoder_name}.")

    def _attempt_create_encoder(self, candidate_config, user_params):
        """Attempts to create and configure a single encoder candidate."""
        factory = Gst.ElementFactory.find(candidate_config["name"])
        if not factory:
            self.logger.debug(f"Encoder factory not found for {candidate_config['name']}. Skipping.")
            return None, None # Return None for encoder and config

        self.logger.info(f"Attempting to use encoder: {candidate_config['name']} ({candidate_config['type']}) for stream {self.stream_id}")
        encoder_element = Gst.ElementFactory.make(candidate_config["name"], f"enc_{self.stream_id}")

        if not encoder_element:
            self.logger.warning(f"Found factory for {candidate_config['name']}, but Gst.ElementFactory.make() failed for stream {self.stream_id}.")
            return None, None
        
        try:
            # _apply_encoder_properties logs its own warnings for individual property failures but does not raise exceptions itself.
            self._apply_encoder_properties(encoder_element, candidate_config, user_params)
            self.logger.info(f"Successfully created and configured encoder: {candidate_config['name']} for stream {self.stream_id}")
            return encoder_element, candidate_config # Return the element and its config
        except Exception as e: 
            # This catch block is more of a safeguard; _apply_encoder_properties should handle its errors gracefully.
            self.logger.error(f"Unexpected error occurred while applying properties to {candidate_config['name']} for stream {self.stream_id}: {e}. This specific encoder instance will not be used.")
            # Element is not part of a pipeline yet, so no need to set state to NULL.
            # It will be garbage collected if not returned.
            return None, None


    def _create_encoder_element(self):
        """Finds the best available H264 encoder and configures it.
        Returns the Gst.Element for the encoder and its candidate configuration dictionary.
        Raises RuntimeError if no suitable encoder can be created.
        """
        # This method is called when self.encoding_format is "video/h264"
        for candidate_config in self.H264_ENCODER_CANDIDATES:
            encoder, chosen_candidate_config = self._attempt_create_encoder(candidate_config, self.parsed_encoder_params)
            if encoder:
                self.logger.info(f"Selected encoder: {encoder.get_name()} with properties:")
                for prop in encoder.list_properties():
                    try:
                        value = encoder.get_property(prop.name)
                        self.logger.info(f"  {prop.name}: {value}")
                    except Exception as e:
                        self.logger.warning(f"  Could not get property {prop.name}: {e}")
                return encoder, chosen_candidate_config # Return both encoder and its config

        # If loop completes, no encoder was successfully created
        error_msg = f"Failed to create any suitable H.264 encoder for stream {self.stream_id} after trying all candidates ({[c['name'] for c in self.H264_ENCODER_CANDIDATES]})."
        self.logger.error(error_msg)
        raise RuntimeError(error_msg)
    
    def _create_pipeline(self):
        """Creates the GStreamer pipeline with all elements."""
        try:
            self.pipeline = Gst.Pipeline.new(f"pipeline_{self.stream_id}")
            
            self.appsrc = Gst.ElementFactory.make("appsrc", f"appsrc_{self.stream_id}")
            videoconvert = Gst.ElementFactory.make("videoconvert", f"vconv_{self.stream_id}")
            
            self.encoder = None
            # selected_encoder_candidate_config = None # Optionally store the chosen encoder's config

            if self.encoding_format == "video/h264":
                # _create_encoder_element will raise RuntimeError if no encoder is found
                self.encoder, _ = self._create_encoder_element() 
                # The second return value (selected_encoder_candidate_config) is currently not used further here.
                
                # Create and configure h264parse since H264 encoding is used
                self.h264parser = Gst.ElementFactory.make("h264parse", f"parse_{self.stream_id}")
                if not self.h264parser:
                    # This check is critical as Gst.ElementFactory.make can return None
                    raise RuntimeError(f"Failed to create h264parse element for {self.stream_id}")
                self.h264parser.set_property("config-interval", -1) # Send SPS/PPS with every IDR frame
                self.h264parser.set_property("disable-passthrough", True) # Force processing to ensure proper format
                
                # Create capsfilter to force byte-stream (Annex-B) format
                self.capsfilter = Gst.ElementFactory.make("capsfilter", f"caps_{self.stream_id}")
                if not self.capsfilter:
                    raise RuntimeError(f"Failed to create capsfilter element for {self.stream_id}")
                #caps = Gst.Caps.from_string("video/x-h264,stream-format=byte-stream,alignment=nal")
                caps = Gst.Caps.from_string("video/x-h264,stream-format=avc,alignment=au,profile=baseline")
                self.capsfilter.set_property("caps", caps)
                
                self.logger.debug(f"Created and configured h264parse and capsfilter for {self.stream_id}")
            else:
                # If other encoding formats were to be supported, their specific logic would go here.
                # For now, this node is focused on H264 via this structure.
                self.logger.error(f"Encoding format '{self.encoding_format}' is not 'video/h264' and is not supported by the current encoder creation logic.")
                raise NotImplementedError(f"Encoding format '{self.encoding_format}' is not supported by this pipeline configuration.")

            self.appsink = Gst.ElementFactory.make("appsink", f"appsink_{self.stream_id}")
            if self.appsink: # Standard check for appsink creation
                self.appsink.set_property("max-buffers", 5) 
                self.appsink.set_property("drop", False)
                self.appsink.set_property("sync", False) # Keep False for lower latency in teleop
                self.appsink.set_property("emit-signals", True)
            else:
                 raise RuntimeError(f"Failed to create appsink element for {self.stream_id}")


            # Consolidate checks for element creation failures.
            # self.encoder is guaranteed by _create_encoder_element raising an error on failure.
            # self.h264parser (if H264) is also checked.
            if not all([self.pipeline, self.appsrc, videoconvert, self.encoder, self.appsink]):
                # Construct a list of missing elements for a more informative error message
                missing_elements = []
                if not self.pipeline: missing_elements.append("pipeline")
                if not self.appsrc: missing_elements.append("appsrc")
                if not videoconvert: missing_elements.append("videoconvert")
                if not self.encoder: missing_elements.append("encoder") # Should be caught earlier by _create_encoder_element
                if not self.appsink: missing_elements.append("appsink")
                # self.h264parser is conditional, so not in this all() check directly for general pipeline
                raise RuntimeError(f"Failed to create one or more core GStreamer elements for {self.stream_id}: {', '.join(missing_elements)}")
            
            self.logger.debug(f"All core GStreamer elements created successfully for {self.stream_id}. Encoder being used: {self.encoder.get_name()}")

            # Add elements to the pipeline
            self.pipeline.add(self.appsrc)
            self.pipeline.add(videoconvert)
            self.pipeline.add(self.encoder) # self.encoder is guaranteed to be valid if we reached here for H264
            if self.h264parser: # Add h264parser if it was created (i.e., for H264)
                self.pipeline.add(self.h264parser)
                self.pipeline.add(self.capsfilter) # Add capsfilter after h264parse
            self.pipeline.add(self.appsink)

            # Link the elements sequentially
            if not self.appsrc.link(videoconvert):
                raise RuntimeError(f"Failed to link appsrc to videoconvert for {self.stream_id}")
            
            last_element_before_appsink = videoconvert # Start with videoconvert
            
            # Link videoconvert -> encoder
            if not last_element_before_appsink.link(self.encoder):
                    raise RuntimeError(f"Failed to link {last_element_before_appsink.get_name()} to encoder ({self.encoder.get_name()}) for {self.stream_id}")
            last_element_before_appsink = self.encoder # Update last successfully linked element

            # Link encoder -> h264parser (if H264 encoding is used)
            if self.h264parser:
                if not last_element_before_appsink.link(self.h264parser):
                    raise RuntimeError(f"Failed to link {last_element_before_appsink.get_name()} to h264parse ({self.h264parser.get_name()}) for {self.stream_id}")
                last_element_before_appsink = self.h264parser # Update last successfully linked element
                
                # Link h264parser -> capsfilter
                if not last_element_before_appsink.link(self.capsfilter):
                    raise RuntimeError(f"Failed to link {last_element_before_appsink.get_name()} to capsfilter for {self.stream_id}")
                last_element_before_appsink = self.capsfilter # Update last successfully linked element
            
            # Link the last element in the encoding chain to appsink
            if not last_element_before_appsink.link(self.appsink):
                raise RuntimeError(f"Failed to link {last_element_before_appsink.get_name()} to appsink for {self.stream_id}")
                
            self.logger.info(f"GStreamer elements linked successfully for {self.stream_id} using encoder {self.encoder.get_name()}")
            
        except Exception as e:
            self.logger.error(f"Failed to create GStreamer pipeline for stream {self.stream_id}: {e}")
            if self.pipeline: # Attempt to set the pipeline to NULL state on failure to aid cleanup
                current_state_result, current_state, pending_state = self.pipeline.get_state(Gst.CLOCK_TIME_NONE)
                if current_state != Gst.State.NULL:
                    self.pipeline.set_state(Gst.State.NULL)
            # Re-raise the exception so it can be handled by the caller (e.g., _handle_add_stream)
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
    
    def _set_caps_from_image(self, msg):
        """Set GStreamer caps based on ROS Image message."""
        try:
            encoding_map = {
                'rgb8': 'RGB',
                'rgba8': 'RGBA',
                'bgr8': 'BGR',
                'bgra8': 'BGRA',
                'mono8': 'GRAY8',
                'mono16': 'GRAY16_LE',
                '16UC1': 'GRAY16_LE', # Common OpenCV depth format
                '32FC1': 'GRAY32_FLOAT_LE' # Common OpenCV float format
            }
            
            gst_format = encoding_map.get(msg.encoding)
            if not gst_format:
                self.logger.error(f"Unsupported ROS image encoding: {msg.encoding} for stream {self.stream_id}")
                return False
                
            # Framerate can be part of caps, often important for encoders/muxers.
            # Using a common default, but could be derived from topic rate or config if needed.
            framerate_str = "30/1" # Defaulting to 30fps for caps
            
            caps_str = (
                f"video/x-raw,"
                f"format={gst_format},"
                f"width={msg.width},"
                f"height={msg.height},"
                f"framerate={framerate_str}"
            )
            
            caps = Gst.Caps.from_string(caps_str)
            if not caps: # Should not happen with well-formed string, but good check
                self.logger.error(f"Failed to parse caps string: {caps_str} for stream {self.stream_id}")
                return False
                
            self.appsrc.set_property("caps", caps)
            self.caps_set = True
            self.width = msg.width # Store actual dimensions used for caps
            self.height = msg.height # Store actual dimensions used for caps
            self.logger.info(f"Set caps for {self.stream_id}: {caps.to_string()}")
            return True
            
        except Exception as e:
            self.logger.error(f"Error setting caps for stream {self.stream_id}: {e}")
            return False
    
    def cleanup(self):
        """Clean up resources used by this pipeline."""
        self.logger.info(f"Cleaning up pipeline for stream {self.stream_id}...")
        if self.pipeline:
            # Get current state to avoid errors if already NULL or during async transitions
            state_change_ret, current_state, pending_state = self.pipeline.get_state(Gst.CLOCK_TIME_NONE) # 0 timeout
            if current_state != Gst.State.NULL:
                 self.logger.debug(f"Setting pipeline {self.stream_id} to NULL state for cleanup (current: {current_state.value_name}).")
                 self.pipeline.set_state(Gst.State.NULL)
            else:
                 self.logger.debug(f"Pipeline {self.stream_id} already in NULL state or state retrieval failed ({state_change_ret.value_name}).")
            self.pipeline = None # Allow garbage collection by removing reference

        # Elements within the pipeline are generally managed by the pipeline's lifecycle.
        # Resetting internal references for clarity and to release direct holds if any.
        self.appsrc = None
        self.encoder = None
        self.h264parser = None
        self.capsfilter = None
        self.appsink = None
        self.caps_set = False # Reset caps status
        self.status = 'STOPPED' # Update status to reflect cleanup
        self.logger.info(f"Pipeline cleanup for stream {self.stream_id} complete.")

    def ros_image_callback(self, msg):
        """Callback for ROS Image messages, pushes image into the pipeline."""
        if not self.appsrc or not self.pipeline: # Ensure pipeline and appsrc are valid
            self.logger.error(f"Cannot push image: appsrc or pipeline not initialized for {self.stream_id}")
            return

        # Optional: Check if pipeline is in PLAYING state before pushing.
        # state_ret, current_gst_state, pending_gst_state = self.pipeline.get_state(Gst.CLOCK_TIME_NONE)
        # if not (state_ret == Gst.StateChangeReturn.SUCCESS and current_gst_state == Gst.State.PLAYING):
        #     self.logger.warning(f"Stream '{self.stream_id}' is not in PLAYING state (current: {current_gst_state.value_name if state_ret == Gst.StateChangeReturn.SUCCESS else 'UNKNOWN'}). Dropping frame.")
        #     return

        self.logger.debug(f"ROS Image received for stream '{self.stream_id}', seq={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, size={msg.width}x{msg.height}, encoding={msg.encoding}")

        if not self.caps_set:
            if not self._set_caps_from_image(msg):
                self.logger.error(f"Failed to set caps from image for stream '{self.stream_id}'. Dropping frame.")
                return

        try:
            # Ensure data is in bytes format
            if isinstance(msg.data, list):
                image_bytes = bytes(msg.data)
            elif isinstance(msg.data, bytes):
                image_bytes = msg.data
            else: # Handles cases like array.array or numpy.ndarray (if tobytes() is available)
                 image_bytes = msg.data.tobytes()

            buffer = Gst.Buffer.new_wrapped(image_bytes)

            # Set PTS (Presentation Timestamp) for the buffer
            # DTS (Decode Timestamp) can be set to Gst.CLOCK_TIME_NONE; GStreamer will manage if not provided.
            buffer.pts = msg.header.stamp.sec * Gst.SECOND + msg.header.stamp.nanosec
            buffer.dts = Gst.CLOCK_TIME_NONE # Let pipeline elements manage DTS

            self.logger.debug(f"Attempting to push Gst.Buffer for '{self.stream_id}' (PTS: {buffer.pts / Gst.SECOND:.3f}s)")
            ret = self.appsrc.emit("push-buffer", buffer)
            if ret != Gst.FlowReturn.OK:
                flow_return_str = Gst.flow_return_get_name(ret) # Get human-readable name
                self.logger.error(f"push-buffer failed for {self.stream_id}: {flow_return_str} (Gst.FlowReturn code: {ret})")
            else:
                self.logger.debug(f"Successfully pushed buffer for stream '{self.stream_id}'")

        except Exception as e:
            self.logger.error(f"Error processing/pushing image to pipeline for {self.stream_id}: {e}")
            import traceback # Keep for detailed error logging in development
            self.logger.error(traceback.format_exc())


class OpenTeleopAvNode(Node):
    """ROS2 Node for handling A/V encoding and service calls."""

    def __init__(self):
        """Node initialization."""
        super().__init__('open_teleop_av_node')

        gst_debug_env = os.environ.get('GST_DEBUG')
        print(f"[AV_NODE PRE-GST_INIT] GST_DEBUG environment variable: {gst_debug_env}")

        try:
            Gst.init(None)
        except Exception as e:
             self.get_logger().error(f"Failed to initialize GStreamer: {e}")
             raise RuntimeError("GStreamer initialization failed") from e

        self.declare_parameter('log.level', 'DEBUG')
        self.declare_parameter('log.path', '/tmp/open_teleop_logs')
        self.declare_parameter('log.to_file', False)
        self.declare_parameter('log.rotation_days', 7)

        self._configure_logging()

        self.logger.debug(f"GST_DEBUG env var check via logger: {os.environ.get('GST_DEBUG')}")
        self.logger.info(f'{self.get_name()} starting up...')

        self.pipelines = {}

        self.encoded_frame_publisher = self.create_publisher(
            EncodedFrame, 
            '~/encoded_frames', 
            10
        )
        self.logger.info(f"Created publisher for {EncodedFrame.__name__} on '{self.encoded_frame_publisher.topic_name}'")

        self.manage_stream_service = self.create_service(ManageStream, '~/manage_stream', self.handle_manage_stream)
        self.get_status_service = self.create_service(GetStatus, '~/get_status', self.handle_get_status)

        self.timer = self.create_timer(5.0, self.timer_callback)

        self.logger.info(f'{self.get_name()} initialized.')

    def timer_callback(self):
        self.logger.debug(f"Timer tick: {len(self.pipelines)} active pipelines.")

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
        
        response.node_status = "OK"
        response.active_streams = []
        for stream_id, pipeline in self.pipelines.items():
            status_msg = StreamStatus(
                stream_id=stream_id,
                input_ros_topic=pipeline.input_topic,
                output_ott_topic=pipeline.output_topic,
                encoding_format=pipeline.encoding_format, 
                is_enabled=(pipeline.status == 'PLAYING'),
                status_message=pipeline.status,
                frame_rate_actual=0.0, 
                bitrate_actual=0
            )
            response.active_streams.append(status_msg)
        
        return response
    
    def on_new_sample(self, appsink: Gst.Element, stream_id: str):
        """Callback executed when appsink receives a new sample."""
        sample = appsink.emit("pull-sample")
        if sample:
            buffer = sample.get_buffer()
            if buffer:
                pts = buffer.pts
                is_delta_unit = buffer.has_flags(Gst.BufferFlags.DELTA_UNIT)
                frame_type_enum = EncodedFrame.FRAME_TYPE_DELTA if is_delta_unit else EncodedFrame.FRAME_TYPE_KEY
                frame_type_str = "Delta" if is_delta_unit else "Key"

                success, map_info = buffer.map(Gst.MapFlags.READ)
                if not success:
                    self.logger.error(f"Appsink ('{stream_id}'): Failed to map buffer")
                    return Gst.FlowReturn.ERROR
                    
                buffer_size = map_info.size
                encoded_bytes = map_info.data[:]
                buffer.unmap(map_info)

                # Log first 32 bytes of encoded frame with NAL unit analysis
                first_32_bytes = encoded_bytes[:32]
                hex_bytes = ' '.join([f'{b:02x}' for b in first_32_bytes])
                
                # Parse NAL unit type for NVIDIA encoder
                if len(encoded_bytes) >= 5:  # Need at least 5 bytes for NAL header
                    nal_length = int.from_bytes(encoded_bytes[0:4], byteorder='big')
                    nal_type = encoded_bytes[4] & 0x1F  # Last 5 bits are NAL type
                    nal_type_str = {
                        1: "Non-IDR",
                        5: "IDR",
                        7: "SPS",
                        8: "PPS",
                        9: "AUD",
                        12: "Filler"
                    }.get(nal_type, f"Unknown({nal_type})")
                    
                    self.logger.debug(
                        f"Appsink ('{stream_id}'): NAL Unit - Type: {nal_type_str}, Length: {nal_length}, "
                        f"First 32 bytes: {hex_bytes}"
                    )
                else:
                    self.logger.debug(
                        f"Appsink ('{stream_id}'): First 32 bytes of encoded frame: {hex_bytes}"
                    )

                self.logger.debug(
                    f"Appsink ('{stream_id}'): Received sample. "
                    f"PTS={pts/Gst.SECOND:.3f}s, Size={buffer_size} bytes, Type={frame_type_str}"
                )
                
                if stream_id not in self.pipelines:
                    self.logger.error(f"Appsink ('{stream_id}'): Pipeline state not found! Cannot publish.")
                    return Gst.FlowReturn.OK
                
                pipeline = self.pipelines[stream_id]
                ott_topic = pipeline.output_topic
                width = pipeline.width
                height = pipeline.height
                encoding_format = pipeline.encoding_format 
                
                if not ott_topic:
                     self.logger.error(f"Appsink ('{stream_id}'): Output OTT topic not found in state! Cannot publish.")
                     return Gst.FlowReturn.OK

                try:
                    msg = EncodedFrame()
                    
                    ros_time = Time()
                    ros_time.sec = pts // Gst.SECOND
                    ros_time.nanosec = pts % Gst.SECOND
                    msg.header.stamp = ros_time
                    
                    msg.ott_topic = ott_topic
                    msg.encoding_format = encoding_format
                    msg.frame_type = frame_type_enum
                    msg.width = width
                    msg.height = height
                    msg.data = list(encoded_bytes)
                    
                    self.encoded_frame_publisher.publish(msg)
                    self.logger.debug(f"Published EncodedFrame for '{stream_id}' to '{ott_topic}'")

                except Exception as pub_e:
                    self.logger.error(f"Appsink ('{stream_id}'): Failed to create/publish EncodedFrame: {pub_e}")

            else:
                 self.logger.warning(f"Appsink ('{stream_id}'): Failed to get buffer from sample")
                 return Gst.FlowReturn.ERROR

            return Gst.FlowReturn.OK
        else:
            self.logger.warning(f"Appsink ('{stream_id}'): pull_sample() returned None")
            return Gst.FlowReturn.ERROR

    def destroy_node(self):
        self.logger.info(f'{self.get_name()} shutting down...')
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

    def _configure_logging(self):
        """Configure the logger based on ROS parameters."""
        try:
            log_level_str = self.get_parameter('log.level').get_parameter_value().string_value
            log_path = self.get_parameter('log.path').get_parameter_value().string_value
            log_to_file = self.get_parameter('log.to_file').get_parameter_value().bool_value

            log_level = getattr(log, log_level_str.upper(), log.INFO)

            self.logger = log.get_logger(
                name=self.get_name(),
                log_dir=log_path if log_to_file else None,
                console_level=log_level,
                file_level=log_level if log_to_file else None
            )
            self.get_logger().info(f"Logger configured: Level={log_level_str}, FileOutput={log_to_file}, Path={log_path if log_to_file else 'N/A'}")

        except Exception as e:
            self.get_logger().error(f"Error configuring logger from parameters: {e}. Using basic configuration.")
            self.logger = log.get_logger(self.get_name())

    def _handle_add_stream(self, request, response):
        """Handle ACTION_ADD request."""
        stream_id = request.stream_id
        
        if not stream_id:
            response.success = False
            response.message = "ACTION_ADD request missing mandatory stream_id (must be topic_id)."
            self.logger.error(response.message)
            return response

        if stream_id in self.pipelines:
            response.success = False
            response.message = f"Stream ID '{stream_id}' already exists. Cannot add duplicate."
            self.logger.error(response.message)
            return response

        if not request.input_ros_topic or not request.output_ott_topic:
            response.success = False
            response.message = "Missing required fields: input_ros_topic and output_ott_topic"
            self.logger.error(response.message)
            return response
            
        self.logger.info(f"Attempting to add stream: {stream_id}")
        self.logger.debug(f"  Input: {request.input_ros_topic}")
        self.logger.debug(f"  Output: {request.output_ott_topic}")
        self.logger.debug(f"  Format: {request.encoding_format}")
        self.logger.debug(f"  Params: {request.encoder_params}")

        try:
            pipeline = GstPipeline(
                logger=self.logger,
                stream_id=stream_id,
                input_topic=request.input_ros_topic,
                output_topic=request.output_ott_topic,
                encoding_format=request.encoding_format,
                encoder_params=request.encoder_params
            )
            
            self.logger.info(f"Creating ROS subscriber for input topic: {pipeline.input_topic}")
            
            msg_type = Image 
            
            callback = functools.partial(pipeline.ros_image_callback) 
            
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
            
            pipeline.ros_subscriber = self.create_subscription(
                msg_type,
                pipeline.input_topic, 
                callback,
                qos
            )
            
            if pipeline.ros_subscriber is None:
                 raise RuntimeError(f"Failed to create ROS subscriber for topic '{pipeline.input_topic}'")
                 
            self.logger.info(f"Successfully created subscriber for '{pipeline.input_topic}'")
            
            if pipeline.appsink:
                handler_id = pipeline.appsink.connect("new-sample", self.on_new_sample, stream_id)
                if handler_id == 0:
                     self.logger.warning(f"Failed to connect new-sample signal for appsink '{stream_id}'")
                else:
                     self.logger.debug(f"Connected new-sample signal for appsink '{stream_id}' (handler: {handler_id})")
            
            self.pipelines[stream_id] = pipeline

            response.success = True
            response.assigned_stream_id = stream_id
            response.message = f"Stream {stream_id} added successfully."
            self.logger.info(response.message)

        except Exception as e:
            self.logger.error(f"Failed to add stream {stream_id}: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            response.success = False
            response.message = f"Failed to add stream: {e}"
            if 'pipeline' in locals() and pipeline:
                 pipeline.cleanup()
            if stream_id in self.pipelines:
                 del self.pipelines[stream_id]

        return response

    def _handle_remove_stream(self, request, response):
        """Handle ACTION_REMOVE request."""
        stream_id = request.stream_id
        self.logger.info(f"Attempting to remove stream: {stream_id}")

        if stream_id not in self.pipelines:
            response.success = False
            response.message = f"Stream ID {stream_id} not found."
            self.logger.error(response.message)
            return response

        try:
            pipeline_to_remove = self.pipelines[stream_id]

            if pipeline_to_remove.ros_subscriber:
                self.logger.info(f"Destroying ROS subscriber for '{pipeline_to_remove.input_topic}'")
                try:
                    self.destroy_subscription(pipeline_to_remove.ros_subscriber)
                    pipeline_to_remove.ros_subscriber = None
                    self.logger.info(f"Successfully destroyed subscriber for '{pipeline_to_remove.input_topic}'")
                except Exception as sub_e:
                     self.logger.error(f"Error destroying subscriber for '{pipeline_to_remove.input_topic}': {sub_e}")

            self.logger.info(f"Cleaning up GStreamer pipeline for {stream_id}")
            pipeline_to_remove.cleanup()
            self.logger.info(f"GStreamer pipeline cleanup complete for {stream_id}")

            del self.pipelines[stream_id]

            response.success = True
            response.message = f"Stream {stream_id} removed successfully."
            self.logger.info(response.message)

        except Exception as e:
            self.logger.error(f"Failed to remove stream {stream_id}: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            response.success = False
            response.message = f"Failed to remove stream: {e}"

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