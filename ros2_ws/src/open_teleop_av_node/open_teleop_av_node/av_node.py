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
        # For nvh264enc, profile: 0=Baseline, 1=Constrained Baseline, 2=Main, 3=High
        {"name": "nvh264enc", "type": "hardware", "properties": {"tune": "zerolatency", "profile": 0, "gop-size": "key-int-max"}},
        {"name": "x264enc", "type": "software", "properties": {"tune": "zerolatency", "profile": None, "key-int-max": "key-int-max"}} # x264enc profile is a string like 'baseline'
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
             self.logger.warning(f"encoder_params for {stream_id} was already a dict, not expected JSON string.")
             self.parsed_encoder_params = encoder_params
        else:
            self.logger.warning(f"No valid encoder_params provided for {stream_id}. Using defaults.")

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
        
        self._create_pipeline()
    
    def _create_pipeline(self):
        """Creates the GStreamer pipeline with all elements."""
        try:
            self.pipeline = Gst.Pipeline.new(f"pipeline_{self.stream_id}")
            
            self.appsrc = Gst.ElementFactory.make("appsrc", f"appsrc_{self.stream_id}")
            videoconvert = Gst.ElementFactory.make("videoconvert", f"vconv_{self.stream_id}")
            
            selected_encoder_info = None
            self.encoder = None

            if self.encoding_format == "video/h264":
                for candidate in self.H264_ENCODER_CANDIDATES:
                    factory = Gst.ElementFactory.find(candidate["name"])
                    if factory:
                        self.logger.info(f"Attempting to use encoder: {candidate['name']} ({candidate['type']}) for stream {self.stream_id}")
                        temp_encoder = Gst.ElementFactory.make(candidate["name"], f"enc_{self.stream_id}")
                        if temp_encoder:
                            try:
                                # Set common tune property if defined for the candidate
                                if "tune" in candidate["properties"] and candidate["properties"]["tune"]:
                                    tune_value = candidate["properties"]["tune"]
                                    if temp_encoder.find_property("tune"): # GObject.ParamSpec
                                        temp_encoder.set_property("tune", tune_value)
                                        self.logger.debug(f"Set {candidate['name']} tune={tune_value}")
                                    else:
                                        self.logger.warning(f"Property 'tune' not found on {candidate['name']}, but specified in candidate config.")

                                # Set H.264 profile if specified (especially for nvh264enc)
                                if "profile" in candidate["properties"] and candidate["properties"]["profile"] is not None:
                                    profile_value = candidate["properties"]["profile"]
                                    if temp_encoder.find_property("profile"):
                                        try:
                                            temp_encoder.set_property("profile", profile_value)
                                            self.logger.debug(f"Set {candidate['name']} profile={profile_value} (0=Baseline for nvh264enc)")
                                        except Exception as e_profile:
                                            self.logger.warning(f"Failed to set profile {profile_value} on {candidate['name']}: {e_profile}")
                                    elif candidate["name"] == "x264enc": # x264enc uses string for profile
                                        # For x264enc, we'd set something like 'baseline' if desired
                                        # but our candidate list currently has None for x264enc profile, so this won't trigger as is.
                                        # Example: temp_encoder.set_property("profile", "baseline")
                                        pass

                                if 'bitrate' in self.parsed_encoder_params:
                                    bitrate_kbps = int(self.parsed_encoder_params['bitrate'])
                                    # Common property name for bitrate is 'bitrate'.
                                    # Some encoders might use different names or require specific rate control modes to be set first.
                                    # For nvh264enc, 'bitrate' is used for CBR/VBR target. For x264enc, 'bitrate' is target in kbps.
                                    if temp_encoder.find_property("bitrate"):
                                        temp_encoder.set_property("bitrate", bitrate_kbps) 
                                        self.logger.debug(f"Set {candidate['name']} bitrate={bitrate_kbps} kbps for {self.stream_id}")
                                    else:
                                        self.logger.warning(f"Property 'bitrate' not found on {candidate['name']}. Cannot set bitrate.")
                                        
                                if 'gop_size' in self.parsed_encoder_params:
                                    gop_frames = int(self.parsed_encoder_params['gop_size'])
                                    gop_prop_name = candidate["properties"].get("gop-size") or candidate["properties"].get("key-int-max") # Prioritize candidate's specific prop name

                                    if gop_prop_name and temp_encoder.find_property(gop_prop_name):
                                        temp_encoder.set_property(gop_prop_name, gop_frames) 
                                        self.logger.debug(f"Set {candidate['name']} {gop_prop_name}={gop_frames} frames for {self.stream_id}")
                                    elif temp_encoder.find_property("key-int-max"): # Fallback to common name
                                        temp_encoder.set_property("key-int-max", gop_frames)
                                        self.logger.debug(f"Set {candidate['name']} key-int-max={gop_frames} (fallback property) for {self.stream_id}")
                                    else:
                                        self.logger.warning(f"Could not find a suitable GOP size property on {candidate['name']}.")
                                
                                self.encoder = temp_encoder
                                selected_encoder_info = candidate
                                self.logger.info(f"Successfully created and configured encoder: {candidate['name']} for stream {self.stream_id}")
                                break # Found and configured an encoder
                            except Exception as e:
                                self.logger.warning(f"Failed to configure {candidate['name']} for stream {self.stream_id}: {e}. Trying next.")
                                if temp_encoder:
                                    # temp_encoder.set_state(Gst.State.NULL) # Risky if not part of pipeline yet
                                    pass # Element will be garbage collected if not assigned to self.encoder
                                self.encoder = None 
                        else:
                            self.logger.warning(f"Found factory for {candidate['name']}, but Gst.ElementFactory.make() failed for stream {self.stream_id}.")
                    else:
                        self.logger.debug(f"Encoder factory not found for {candidate['name']}. Skipping.")

                if not self.encoder:
                    self.logger.error(f"Failed to create any suitable H.264 encoder for stream {self.stream_id} after trying all candidates ({[c['name'] for c in self.H264_ENCODER_CANDIDATES]}).")
                    raise RuntimeError(f"No H264 encoder could be initialized for {self.stream_id}")
                
                # Create and configure h264parse if H264 encoding is used
                self.h264parser = Gst.ElementFactory.make("h264parse", f"parse_{self.stream_id}")
                if not self.h264parser:
                    raise RuntimeError(f"Failed to create h264parse element for {self.stream_id}")
                self.h264parser.set_property("config-interval", -1) # Send SPS/PPS with every IDR frame
                self.logger.debug(f"Created and configured h264parse for {self.stream_id}")
            else:
                raise NotImplementedError(f"Encoding format '{self.encoding_format}' is not supported yet.")

            self.appsink = Gst.ElementFactory.make("appsink", f"appsink_{self.stream_id}")
            if self.appsink:
                self.appsink.set_property("max-buffers", 5) 
                self.appsink.set_property("drop", False)
                self.appsink.set_property("sync", False)
                self.appsink.set_property("emit-signals", True)

            if not all([self.pipeline, self.appsrc, videoconvert, self.encoder, self.appsink]):
                raise RuntimeError(f"Failed to create one or more GStreamer elements for {self.stream_id}")
            
            self.logger.debug(f"Elements created for {self.stream_id}")

            self.pipeline.add(self.appsrc)
            self.pipeline.add(videoconvert)
            self.pipeline.add(self.encoder)
            if self.h264parser: # Add h264parser if it was created (i.e. for H264)
                self.pipeline.add(self.h264parser)
            self.pipeline.add(self.appsink)

            if not self.appsrc.link(videoconvert):
                raise RuntimeError(f"Failed to link appsrc to videoconvert for {self.stream_id}")
            
            last_element_before_appsink = videoconvert
            if self.encoder:
                if not last_element_before_appsink.link(self.encoder):
                     raise RuntimeError(f"Failed to link {last_element_before_appsink.get_name()} to encoder for {self.stream_id}")
                last_element_before_appsink = self.encoder

            if self.h264parser:
                if not last_element_before_appsink.link(self.h264parser):
                    raise RuntimeError(f"Failed to link {last_element_before_appsink.get_name()} to h264parse for {self.stream_id}")
                last_element_before_appsink = self.h264parser
            
            if not last_element_before_appsink.link(self.appsink):
                raise RuntimeError(f"Failed to link {last_element_before_appsink.get_name()} to appsink for {self.stream_id}")
                
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

    def ros_image_callback(self, msg):
        """Callback for ROS Image messages, pushes image into the pipeline."""
        if not self.appsrc:
            self.logger.error(f"Cannot push image: appsrc not initialized for {self.stream_id}")
            return

        self.logger.debug(f"ROS Image received for stream '{self.stream_id}', seq={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, size={msg.width}x{msg.height}, encoding={msg.encoding}")

        if not self.caps_set:
            if not self._set_caps_from_image(msg):
                self.logger.error(f"Failed to set caps from image for stream '{self.stream_id}'. Dropping frame.")
                return

        try:
            if isinstance(msg.data, list):
                image_bytes = bytes(msg.data)
            elif isinstance(msg.data, bytes):
                image_bytes = msg.data
            else:
                 image_bytes = msg.data.tobytes()

            buffer = Gst.Buffer.new_wrapped(image_bytes)

            buffer.pts = msg.header.stamp.sec * Gst.SECOND + msg.header.stamp.nanosec
            buffer.dts = Gst.CLOCK_TIME_NONE

            self.logger.debug(f"Attempting to create Gst.Buffer for '{self.stream_id}'")
            ret = self.appsrc.emit("push-buffer", buffer)
            if ret != Gst.FlowReturn.OK:
                flow_return_str = Gst.flow_return_get_name(ret)
                self.logger.error(f"push-buffer failed for {self.stream_id}: {flow_return_str} ({ret})")
            else:
                self.logger.debug(f"Successfully pushed buffer for stream '{self.stream_id}'")

        except Exception as e:
            self.logger.error(f"Error processing/pushing image to pipeline for {self.stream_id}: {e}")
            import traceback
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