"""
Pipeline Manager for Media Gateway.

This module handles creation and management of GStreamer pipelines for hardware devices,
following the exact same pattern as the ros2 av_node implementation.
"""

import time
import json
import logging
import asyncio
import threading
import flatbuffers
from typing import Dict, List, Optional, Callable

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Import the generated FlatBuffer code (same as ros2 gateway)
from media_gateway.flatbuffers.open_teleop.message import ContentType
from media_gateway.flatbuffers.open_teleop.message.FrameType import FrameType
from media_gateway.flatbuffers.open_teleop.message.OttMessage import (
    OttMessageStart,
    OttMessageAddVersion,
    OttMessageAddPayload,
    OttMessageAddContentType,
    OttMessageAddOtt,
    OttMessageAddTimestampNs,
    OttMessageAddVideoMetadata,
    OttMessageEnd
)
from media_gateway.flatbuffers.open_teleop.message.VideoFrameMetadata import (
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


class GstMediaPipeline:
    """
    Manages a GStreamer pipeline for hardware media capture and encoding.
    
    Follows the exact same pattern as the ros2 av_node GstPipeline but uses
    hardware sources (V4L2, ALSA) instead of ROS image topics.
    """
    
    # Same H264 encoder candidates as ros2 av_node
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
                "gop_size": "gop-size",
                "bitrate": "bitrate" # kbps
            }
        },
        {
            "name": "x264enc", 
            "type": "software", 
            "fixed_properties": {
                "tune": "zerolatency", 
            },
            "param_mappings": {
                "gop_size": "key-int-max",
                "bitrate": "bitrate" # kbps
            }
        }
    ]
    
    def __init__(self, stream_id: str, device_path: str, ott_topic: str, 
                 encoding_format: str, encoder_params: dict, 
                 frame_callback: Callable, logger):
        """Initialize the GStreamer pipeline for hardware capture."""
        self.logger = logger
        self.stream_id = stream_id
        self.device_path = device_path
        self.ott_topic = ott_topic
        self.encoding_format = encoding_format
        self.encoder_params = encoder_params or {}
        self.frame_callback = frame_callback
        
        # Pipeline components
        self.pipeline = None
        self.source = None
        self.encoder = None
        self.appsink = None
        self.h264parser = None
        self.capsfilter = None
        self.queue = None
        
        # State tracking
        self.status = 'CREATED'
        self.width = encoder_params.get('width', 1920)
        self.height = encoder_params.get('height', 1080)
        self.framerate = encoder_params.get('framerate', 30)
        
        # Frame sequence tracking (same as ros2 gateway)
        self.sequence_number = 0
        
        self._create_pipeline()
        
    def _create_pipeline(self):
        """Creates the GStreamer pipeline with all elements."""
        try:
            self.pipeline = Gst.Pipeline.new(f"pipeline_{self.stream_id}")
            
            # Create source element based on device type
            if self.device_path.startswith('/dev/video'):
                # V4L2 video source
                self.source = Gst.ElementFactory.make("v4l2src", f"source_{self.stream_id}")
                self.source.set_property("device", self.device_path)
                
                # Create videoconvert for format conversion
                videoconvert = Gst.ElementFactory.make("videoconvert", f"vconv_{self.stream_id}")
                
                # Create caps filter for source format
                src_caps = Gst.ElementFactory.make("capsfilter", f"src_caps_{self.stream_id}")
                caps_str = (
                    f"video/x-raw,"
                    f"width={self.width},"
                    f"height={self.height},"
                    f"framerate={self.framerate}/1"
                )
                caps = Gst.Caps.from_string(caps_str)
                src_caps.set_property("caps", caps)
                
                # Add elements to pipeline
                self.pipeline.add(self.source)
                self.pipeline.add(src_caps)
                self.pipeline.add(videoconvert)
                
                # Link source chain
                if not self.source.link(src_caps):
                    raise RuntimeError(f"Failed to link source to caps for {self.stream_id}")
                if not src_caps.link(videoconvert):
                    raise RuntimeError(f"Failed to link caps to videoconvert for {self.stream_id}")
                
                last_element = videoconvert
                
            elif self.device_path.startswith('hw:'):
                # ALSA audio source (future implementation)
                raise NotImplementedError("Audio pipelines not yet implemented")
            else:
                raise ValueError(f"Unsupported device path: {self.device_path}")
            
            # Create encoder based on encoding format
            if self.encoding_format == "video/h264":
                self.encoder, _ = self._create_encoder_element()
                
                # Create h264parse for proper format
                self.h264parser = Gst.ElementFactory.make("h264parse", f"parse_{self.stream_id}")
                if not self.h264parser:
                    raise RuntimeError(f"Failed to create h264parse element for {self.stream_id}")
                self.h264parser.set_property("config-interval", 1)
                self.h264parser.set_property("disable-passthrough", True)
                
                # Create capsfilter for output format
                self.capsfilter = Gst.ElementFactory.make("capsfilter", f"caps_{self.stream_id}")
                if not self.capsfilter:
                    raise RuntimeError(f"Failed to create capsfilter element for {self.stream_id}")
                caps = Gst.Caps.from_string("video/x-h264,stream-format=avc,alignment=au,profile=baseline")
                self.capsfilter.set_property("caps", caps)
                
                # Create queue
                self.queue = Gst.ElementFactory.make("queue", f"queue_{self.stream_id}")
                self.queue.set_property("leaky", "downstream")
                self.queue.set_property("max-size-buffers", 2)
                self.queue.set_property("max-size-bytes", 0)
                self.queue.set_property("max-size-time", 0)
                
                # Create appsink
                self.appsink = Gst.ElementFactory.make("appsink", f"appsink_{self.stream_id}")
                self.appsink.set_property("max-buffers", 5)
                self.appsink.set_property("drop", False)
                self.appsink.set_property("sync", False)
                self.appsink.set_property("emit-signals", True)
                
                # Add encoding elements to pipeline
                self.pipeline.add(self.encoder)
                self.pipeline.add(self.h264parser)
                self.pipeline.add(self.capsfilter)
                self.pipeline.add(self.queue)
                self.pipeline.add(self.appsink)
                
                # Link encoding chain
                if not last_element.link(self.encoder):
                    raise RuntimeError(f"Failed to link {last_element.get_name()} to encoder for {self.stream_id}")
                if not self.encoder.link(self.h264parser):
                    raise RuntimeError(f"Failed to link encoder to h264parse for {self.stream_id}")
                if not self.h264parser.link(self.capsfilter):
                    raise RuntimeError(f"Failed to link h264parse to capsfilter for {self.stream_id}")
                if not self.capsfilter.link(self.queue):
                    raise RuntimeError(f"Failed to link capsfilter to queue for {self.stream_id}")
                if not self.queue.link(self.appsink):
                    raise RuntimeError(f"Failed to link queue to appsink for {self.stream_id}")
                
                # Connect appsink signal (same as ros2 av_node)
                self.appsink.connect("new-sample", self._on_new_sample)
                
            else:
                raise NotImplementedError(f"Encoding format '{self.encoding_format}' not supported")
                
            self.status = 'READY'
            self.logger.info(f"Pipeline created successfully for {self.stream_id}")
            
        except Exception as e:
            self.logger.error(f"Failed to create pipeline for {self.stream_id}: {e}")
            self.status = 'ERROR'
            raise
    
    def _create_encoder_element(self):
        """Create H264 encoder element (same logic as ros2 av_node)."""
        for candidate_config in self.H264_ENCODER_CANDIDATES:
            encoder, chosen_candidate_config = self._attempt_create_encoder(candidate_config, self.encoder_params)
            if encoder:
                self.logger.info(f"Selected encoder: {encoder.get_name()} for {self.stream_id}")
                return encoder, chosen_candidate_config

        error_msg = f"Failed to create any suitable H.264 encoder for stream {self.stream_id}"
        self.logger.error(error_msg)
        raise RuntimeError(error_msg)
    
    def _attempt_create_encoder(self, candidate_config, params):
        """Attempt to create an encoder with given configuration."""
        try:
            encoder = Gst.ElementFactory.make(candidate_config["name"])
            if not encoder:
                return None, None
                
            # Set fixed properties
            for prop, value in candidate_config["fixed_properties"].items():
                encoder.set_property(prop, value)
                
            # Set parameter mappings
            for param_key, gst_prop in candidate_config["param_mappings"].items():
                if param_key in params:
                    encoder.set_property(gst_prop, params[param_key])
                    
            return encoder, candidate_config
            
        except Exception as e:
            self.logger.debug(f"Failed to create encoder {candidate_config['name']}: {e}")
            return None, None
    
    def _on_new_sample(self, appsink):
        """Handle new sample from appsink (same logic as ros2 av_node)."""
        sample = appsink.emit("pull-sample")
        if sample:
            buffer = sample.get_buffer()
            if buffer:
                self._process_encoded_frame(buffer)
                return Gst.FlowReturn.OK
        return Gst.FlowReturn.ERROR
    
    def _process_encoded_frame(self, buffer):
        """Process encoded frame and send via callback."""
        try:
            pts = buffer.pts
            is_delta_unit = buffer.has_flags(Gst.BufferFlags.DELTA_UNIT)
            frame_type = 2 if is_delta_unit else 1  # DELTA=2, KEY=1 (same as ros2)
            
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                self.logger.error(f"Failed to map buffer for {self.stream_id}")
                return
                
            encoded_bytes = map_info.data[:]
            buffer.unmap(map_info)
            
            # Create frame data structure (same as ros2 EncodedFrame)
            frame_data = {
                'ott_topic': self.ott_topic,
                'encoding_format': self.encoding_format,
                'frame_type': frame_type,
                'width': self.width,
                'height': self.height,
                'pts': pts,
                'data': encoded_bytes,
                'sequence_number': self._get_next_sequence_number()
            }
            
            self.logger.debug(f"Processed frame for {self.stream_id}: {len(encoded_bytes)} bytes, "
                            f"type={'KEY' if frame_type == 1 else 'DELTA'}")
            
            # Send via callback
            self.frame_callback(frame_data)
            
        except Exception as e:
            self.logger.error(f"Error processing encoded frame for {self.stream_id}: {e}")
    
    def _get_next_sequence_number(self):
        """Get next sequence number for this stream."""
        self.sequence_number += 1
        return self.sequence_number
    
    def start(self):
        """Start the pipeline."""
        if self.pipeline:
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                self.status = 'ERROR'
                raise RuntimeError(f"Failed to start pipeline for {self.stream_id}")
            else:
                self.status = 'RUNNING'
                self.logger.info(f"Pipeline started for {self.stream_id}")
    
    def stop(self):
        """Stop the pipeline."""
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.status = 'STOPPED'
            self.logger.info(f"Pipeline stopped for {self.stream_id}")
    
    def cleanup(self):
        """Clean up resources."""
        self.stop()
        self.pipeline = None


class PipelineManager:
    """
    Manages multiple GStreamer pipelines for the Media Gateway.
    
    Follows the same pattern as the ros2 av_node service management but
    for hardware devices instead of ROS topics.
    """
    
    def __init__(self, device_manager, zmq_client, logger):
        """Initialize the pipeline manager."""
        self.device_manager = device_manager
        self.zmq_client = zmq_client
        self.logger = logger
        
        # Pipeline tracking
        self.pipelines: Dict[str, GstMediaPipeline] = {}
        self.sequence_numbers: Dict[str, int] = {}
        
        # Initialize GStreamer
        if not Gst.is_initialized():
            Gst.init(None)
            
        # GLib main loop for GStreamer callbacks
        self.main_loop = GLib.MainLoop()
        self.loop_thread = None
        
    async def start_manager(self):
        """Start the pipeline manager and GLib main loop."""
        self.logger.info("Starting Pipeline Manager...")
        
        # Start GLib main loop in separate thread
        self.loop_thread = threading.Thread(target=self.main_loop.run)
        self.loop_thread.daemon = True
        self.loop_thread.start()
        
        self.logger.info("Pipeline Manager started")
    
    async def stop_manager(self):
        """Stop the pipeline manager."""
        self.logger.info("Stopping Pipeline Manager...")
        
        # Stop all pipelines
        await self.stop_all_streams()
        
        # Stop main loop
        if self.main_loop and self.main_loop.is_running():
            self.main_loop.quit()
            
        if self.loop_thread and self.loop_thread.is_alive():
            self.loop_thread.join(timeout=5.0)
            
        self.logger.info("Pipeline Manager stopped")
        
    async def create_streams_from_config(self, media_mappings: dict):
        """Create streams based on media_mappings configuration."""
        self.logger.info("Creating streams from media mappings...")
        
        # Process video streams
        video_streams = media_mappings.get('video', [])
        for stream_config in video_streams:
            try:
                await self.create_stream(stream_config)
            except Exception as e:
                self.logger.error(f"Failed to create video stream: {e}")
                
        # Audio streams would be processed here in the future
        audio_streams = media_mappings.get('audio', [])
        if audio_streams:
            self.logger.info(f"Skipping {len(audio_streams)} audio streams (not implemented yet)")
            
        self.logger.info(f"Created {len(self.pipelines)} video streams")
        
    async def create_stream(self, stream_config: dict):
        """Create a new stream pipeline."""
        device_id = stream_config['device_id']
        ott_topic = stream_config['ott']
        encoding_format = stream_config['encoding_format']
        encoder_params = stream_config.get('encoder_params', {})
        
        stream_id = f"{device_id}_{ott_topic.replace('.', '_')}"
        
        self.logger.info(f"Creating stream: {stream_id} for device {device_id}")
        
        # Get device information
        try:
            if encoding_format.startswith('video/'):
                device = self.device_manager.get_video_device(device_id)
                device_path = device.device_path
            elif encoding_format.startswith('audio/'):
                device = self.device_manager.get_audio_device(device_id)
                device_path = device.device_path
            else:
                raise ValueError(f"Unsupported encoding format: {encoding_format}")
                
        except Exception as e:
            self.logger.error(f"Device {device_id} not found: {e}")
            raise
            
        # Create pipeline
        pipeline = GstMediaPipeline(
            stream_id=stream_id,
            device_path=device_path,
            ott_topic=ott_topic,
            encoding_format=encoding_format,
            encoder_params=encoder_params,
            frame_callback=self._on_frame_ready,
            logger=self.logger
        )
        
        # Start pipeline
        pipeline.start()
        
        # Track pipeline
        self.pipelines[stream_id] = pipeline
        
        self.logger.info(f"Stream created and started: {stream_id}")
        
    def _on_frame_ready(self, frame_data: dict):
        """Handle encoded frame from pipeline (same format as ros2 gateway)."""
        try:
            # Get current timestamp in nanoseconds
            gateway_timestamp_ns = time.time_ns()
            
            # Extract frame data
            ott_topic = frame_data['ott_topic']
            encoding_format = frame_data['encoding_format']
            frame_type = frame_data['frame_type']
            width = frame_data['width']
            height = frame_data['height']
            pts = frame_data['pts']
            encoded_data = frame_data['data']
            sequence_number = frame_data['sequence_number']
            
            # Convert PTS to nanoseconds (original timestamp)
            original_timestamp_ns = pts
            
            # Map frame type to FlatBuffer frame type
            if frame_type == 1:
                fb_frame_type = FrameType.FRAME_TYPE_KEY
            elif frame_type == 2:
                fb_frame_type = FrameType.FRAME_TYPE_DELTA
            else:
                fb_frame_type = FrameType.FRAME_TYPE_UNKNOWN
            
            # Create OTT FlatBuffer message (same as ros2 gateway)
            builder = flatbuffers.Builder(1024 + len(encoded_data))
            
            # Create string fields
            ott_fb = builder.CreateString(ott_topic)
            encoding_format_fb = builder.CreateString(encoding_format)
            frame_id_fb = builder.CreateString("")  # No frame_id for hardware sources
            
            # Create VideoFrameMetadata
            VideoFrameMetadataStart(builder)
            VideoFrameMetadataAddSequenceNumber(builder, sequence_number)
            VideoFrameMetadataAddOriginalTimestampNs(builder, original_timestamp_ns)
            VideoFrameMetadataAddFrameType(builder, fb_frame_type)
            VideoFrameMetadataAddEncodingFormat(builder, encoding_format_fb)
            VideoFrameMetadataAddWidth(builder, width)
            VideoFrameMetadataAddHeight(builder, height)
            VideoFrameMetadataAddFrameId(builder, frame_id_fb)
            video_metadata = VideoFrameMetadataEnd(builder)
            
            # Create the payload byte vector
            payload = builder.CreateByteVector(encoded_data)
            
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
            
            # Send to controller (same as ros2 gateway)
            self.logger.debug(f"Sending encoded frame ({len(buf)} bytes) for {ott_topic} "
                            f"[seq={sequence_number}, type={'KEY' if fb_frame_type == FrameType.FRAME_TYPE_KEY else 'DELTA'}, "
                            f"{width}x{height}, {encoding_format}]")
            
            reply_str = self.zmq_client.send_request_binary(buf)
            
            if reply_str:
                self.logger.debug(f"Reply from ZeroMQ: {reply_str}")
                
        except Exception as e:
            self.logger.error(f"Error sending frame for {frame_data.get('ott_topic', 'unknown')}: {e}")
        
    async def stop_all_streams(self):
        """Stop all active streams."""
        self.logger.info("Stopping all streams...")
        
        for stream_id, pipeline in self.pipelines.items():
            try:
                pipeline.cleanup()
                self.logger.info(f"Stopped stream: {stream_id}")
            except Exception as e:
                self.logger.error(f"Error stopping stream {stream_id}: {e}")
                
        self.pipelines.clear()
        self.logger.info("All streams stopped")
        
    async def health_check(self):
        """Perform health check on all pipelines."""
        healthy_count = 0
        
        for stream_id, pipeline in self.pipelines.items():
            if pipeline.status == 'RUNNING':
                healthy_count += 1
            else:
                self.logger.warning(f"Pipeline {stream_id} is not running: {pipeline.status}")
                
        self.logger.debug(f"Health check: {healthy_count}/{len(self.pipelines)} pipelines healthy")
        return healthy_count == len(self.pipelines)
        
    async def update_streams(self, new_config):
        """Update streams with new configuration."""
        self.logger.info("Updating streams with new configuration")
        
        # For now, restart all streams
        await self.stop_all_streams()
        
        media_mappings = new_config.get('media_mappings', {})
        await self.create_streams_from_config(media_mappings) 