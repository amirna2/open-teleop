#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import gi

# Import message type
from open_teleop_msgs.msg import EncodedFrame

# --- Restore GStreamer Imports ---
# Import GStreamer and GLib
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
# --- END Restore ---

class StreamViewer(Node):
    """ROS2 Node to subscribe to EncodedFrame messages and display the video using GStreamer."""

    def __init__(self):
        """Node initialization."""
        super().__init__('stream_viewer') # Restore original name

        # --- Restore GStreamer Initialization and Pipeline ---
        # Initialize GStreamer
        try:
            Gst.init(None)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GStreamer: {e}")
            raise RuntimeError("GStreamer initialization failed") from e
        
        self.pipeline = None
        self.appsrc = None
        
        # Build the GStreamer pipeline
        try:
            # Original pipeline description using gst-parse-launch syntax
            pipeline_desc = (
                "appsrc name=ros_src format=time ! "
                "h264parse ! "
                "avdec_h264 ! "
                "videoconvert ! "
                "autovideosink sync=false"
            )
            self.get_logger().info(f"Creating GStreamer pipeline: {pipeline_desc}")
            self.pipeline = Gst.parse_launch(pipeline_desc)

            # Get the appsrc element
            self.appsrc = self.pipeline.get_by_name("ros_src")
            if not self.appsrc:
                raise RuntimeError("Failed to get appsrc element from pipeline.")

            # Set pipeline to playing state
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("Failed to set GStreamer pipeline to PLAYING state.")
            elif ret == Gst.StateChangeReturn.ASYNC:
                 self.get_logger().info("Pipeline state change to PLAYING is ASYNC.")
            else:
                self.get_logger().info("Pipeline state set to PLAYING.")

        except Exception as e:
            self.get_logger().error(f"Failed to create or start GStreamer pipeline: {e}")
            if self.pipeline:
                 self.pipeline.set_state(Gst.State.NULL)
            raise RuntimeError("GStreamer pipeline setup failed") from e
        # --- END Restore ---

        # Create ROS Subscriber (Keep this)
        self.subscription = self.create_subscription(
            EncodedFrame,
            '/open_teleop_av_node/encoded_frames',
            self.encoded_frame_callback,
            10 # QoS depth
        )
        self.get_logger().info(f"Subscribed to {self.subscription.topic_name}")
        # --- Add a Timer for Liveness Check ---
        self.timer_period = 2.0 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info(f"Created liveness timer running every {self.timer_period} seconds.")
        # --- End Add Timer ---

    def encoded_frame_callback(self, msg: EncodedFrame):
        """Callback executed when an EncodedFrame message arrives."""
        # Restore simpler logging for callback entry
        self.get_logger().debug(
            f"Received EncodedFrame: PTS={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, "
            f"Size={len(msg.data)}, Type={msg.frame_type}"
        )

        # --- Restore GStreamer Pushing Logic ---
        if not self.appsrc:
            self.get_logger().error("Appsrc not initialized, cannot push buffer.")
            return
        # --- END Restore ---

        try:
            # Convert list of uint8 back to bytes
            encoded_bytes = bytes(msg.data)
            
            # --- Restore GStreamer Buffer Creation/Pushing ---
            # Create a Gst.Buffer from the encoded bytes
            buffer = Gst.Buffer.new_wrapped(encoded_bytes)
            
            # Set timestamp on Gst.Buffer (from msg.header.stamp)
            buffer.pts = msg.header.stamp.sec * Gst.SECOND + msg.header.stamp.nanosec
            buffer.dts = Gst.CLOCK_TIME_NONE # Let GStreamer handle DTS

            # Push buffer into appsrc
            ret = self.appsrc.emit("push-buffer", buffer)
            
            if ret != Gst.FlowReturn.OK:
                 flow_return_str = Gst.flow_return_get_name(ret)
                 # Use warning level for flow errors, might not be fatal
                 self.get_logger().warning(f"push-buffer signal failed with return code: {flow_return_str} ({ret})")
            # --- END Restore ---

        except Exception as e:
            self.get_logger().error(f"Error processing encoded frame or pushing to GStreamer: {e}")
            # rclpy.shutdown() # Shutdown ROS node on error

    def timer_callback(self):
        """Callback for the liveness timer."""
        self.get_logger().debug("Liveness timer ticked.")

    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info("Shutting down StreamViewer node...")
        # --- Restore GStreamer Cleanup ---
        if self.pipeline:
            self.get_logger().info("Setting GStreamer pipeline to NULL state.")
            self.pipeline.set_state(Gst.State.NULL)
        # --- END Restore ---
        super().destroy_node()

def main(args=None):
    # --- Remove VERY early logging ---
    # print("DEBUG: Script execution started.", file=sys.stderr)
    # try:
    #     print("DEBUG: Attempting rclpy.init()", file=sys.stderr)
    rclpy.init(args=args)
    #     print("DEBUG: rclpy.init() successful.", file=sys.stderr)
    # except Exception as init_e:
    #     print(f"FATAL: rclpy.init() failed: {init_e}", file=sys.stderr)
    #     sys.exit(1)
    # --- End Remove ---
    node = None
    try:
        # print("DEBUG: Attempting StreamViewer() creation", file=sys.stderr)
        node = StreamViewer()
        # print("DEBUG: StreamViewer() creation successful.", file=sys.stderr)
        # --- Use standard rclpy.spin() --- 
        node.get_logger().info("Running ROS subscriber, writing data to stdout...")
        # --- Remove wrap spin() --- 
        # try:
        #     print("DEBUG: Entering rclpy.spin()...", file=sys.stderr)
        rclpy.spin(node)
        #     print("DEBUG: rclpy.spin() exited normally.", file=sys.stderr)
        # except Exception as spin_e:
        #     # Log any error during spin itself
        #     if node:
        #          node.get_logger().error(f"Exception during rclpy.spin(): {spin_e}")
        #     else:
        #         print(f"Exception during rclpy.spin() (node unavailable): {spin_e}", file=sys.stderr)
        #     import traceback
        #     traceback.print_exc(file=sys.stderr)
        # --- End Remove ---
    except KeyboardInterrupt:
        # print("DEBUG: KeyboardInterrupt received.", file=sys.stderr)
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"Unhandled exception: {e}")
            import traceback
            node.get_logger().error(traceback.format_exc())
        else:
            print(f"Failed to initialize node: {e}", file=sys.stderr)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 