#!/usr/bin/env python3

"""
ROS2 Webcam Publisher Node

This node captures video from the laptop webcam and publishes it as raw Image messages.
It provides real-time video streaming capabilities for robotics applications.

Features:
- Webcam capture using OpenCV
- Configurable resolution and frame rate
- Raw Image message publishing
- Proper ROS2 lifecycle management
- Error handling and recovery
"""

import argparse
import sys
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class WebcamPublisher(Node):
    """
    ROS2 node that captures video from webcam and publishes as Image messages.
    
    This node provides a bridge between standard webcam hardware and ROS2
    image topics, enabling real-time video streaming for robotics applications.
    """
    
    def __init__(self, 
                 camera_id: int = 0,
                 width: int = 640,
                 height: int = 480,
                 fps: float = 30.0,
                 topic_name: str = "camera/image_raw"):
        """
        Initialize the webcam publisher.
        
        Args:
            camera_id: Camera device ID (usually 0 for built-in webcam)
            width: Image width in pixels
            height: Image height in pixels
            fps: Target frames per second
            topic_name: ROS topic name for publishing images
        """
        super().__init__('webcam_publisher')
        
        # Store configuration
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.fps = fps
        self.topic_name = topic_name
        
        # Initialize camera capture
        self.cap: Optional[cv2.VideoCapture] = None
        self.frame_count = 0
        self.start_time = time.time()
        
        # Processing time statistics
        self.total_processing_time = 0.0
        self.min_processing_time = float('inf')
        self.max_processing_time = 0.0
        
        # Detailed timing statistics
        self.total_capture_time = 0.0
        self.total_conversion_time = 0.0
        self.total_message_creation_time = 0.0
        self.total_publish_time = 0.0
        
        # Threading control
        self.running = False
        self.capture_thread: Optional[threading.Thread] = None
        
        # Setup ROS2 publisher with appropriate QoS
        self._setup_publisher()
        
        # Initialize camera
        self._initialize_camera()
        
        self.get_logger().info(
            f"Webcam publisher initialized - Camera: {self.camera_id}, "
            f"Resolution: {self.width}x{self.height}, FPS: {self.fps}, "
            f"Topic: {self.topic_name}"
        )

    def _setup_publisher(self) -> None:
        """Setup ROS2 publisher with appropriate QoS settings for video streaming."""
        # QoS profile optimized for real-time video streaming
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Prioritize low latency over reliability
            history=HistoryPolicy.KEEP_LAST,
            depth=10,  # Only keep latest frame
            durability=DurabilityPolicy.VOLATILE  # Don't persist old frames
        )
        
        self.image_publisher = self.create_publisher(
            Image, 
            self.topic_name, 
            qos
        )

    def _initialize_camera(self) -> bool:
        """
        Initialize camera capture with error handling.
        
        Returns:
            True if camera initialized successfully, False otherwise
        """
        try:
            # Attempt to use V4L2 backend directly
            self.get_logger().info("Attempting to open camera using cv2.CAP_V4L2 backend...")
            self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
            
            if not self.cap.isOpened():
                self.get_logger().warn("Failed to open camera with cv2.CAP_V4L2 backend. Falling back to default backend.")
                self.cap = cv2.VideoCapture(self.camera_id) # Fallback to default

            if not self.cap.isOpened():
                self.get_logger().error(f"Failed to open camera {self.camera_id} with any backend.")
                return False
            
            # Store requested dimensions for clarity in logging
            requested_width = self.width
            requested_height = self.height
            requested_fps = self.fps

            self.get_logger().info(f"Attempting to set camera resolution to {requested_width}x{requested_height} @ {requested_fps:.1f} FPS")

            # Set buffer size to minimize latency
            buffer_success = self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if buffer_success:
                self.get_logger().info("Successfully set buffer size to 1")
            else:
                self.get_logger().warn("Failed to set buffer size to 1")

            # Try YUYV format first (uncompressed, potentially faster)
            fourcc_yuyv = cv2.VideoWriter_fourcc(*'YUYV')
            success_yuyv = self.cap.set(cv2.CAP_PROP_FOURCC, fourcc_yuyv)
            if success_yuyv:
                self.get_logger().info("Successfully set FOURCC: YUYV")
            else:
                self.get_logger().warn("Failed to set FOURCC YUYV, trying MJPG...")
                # Fallback to MJPG
                fourcc_mjpg = cv2.VideoWriter_fourcc(*'MJPG')
                success_mjpg = self.cap.set(cv2.CAP_PROP_FOURCC, fourcc_mjpg)
                if success_mjpg:
                    self.get_logger().info("Successfully set FOURCC: MJPG")
                else:
                    self.get_logger().warn("Failed to set any specific FOURCC format")

            # Set camera properties and check success
            success_width = self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(requested_width))
            if not success_width:
                self.get_logger().warn(f"Failed to set frame width to {requested_width}. OpenCV returned failure.")
            
            success_height = self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(requested_height))
            if not success_height:
                self.get_logger().warn(f"Failed to set frame height to {requested_height}. OpenCV returned failure.")
            
            success_fps = self.cap.set(cv2.CAP_PROP_FPS, float(requested_fps))
            if not success_fps:
                self.get_logger().warn(f"Failed to set FPS to {requested_fps:.1f}. OpenCV returned failure.")
            
            # Verify actual settings
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            actual_fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
            actual_buffer_size = int(self.cap.get(cv2.CAP_PROP_BUFFERSIZE))
            
            # Convert FOURCC to readable format
            fourcc_str = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])
            
            self.get_logger().info(
                f"Camera successfully initialized. "
                f"Requested: {requested_width}x{requested_height} @ {requested_fps:.1f} FPS. "
                f"Actual: {actual_width}x{actual_height} @ {actual_fps:.1f} FPS, "
                f"Format: {fourcc_str}, Buffer size: {actual_buffer_size}"
            )
            
            # Update dimensions to actual values for the rest of the node's operations
            self.width = actual_width
            self.height = actual_height
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Camera initialization failed: {str(e)}")
            return False

    def _create_image_message(self, cv_image: np.ndarray) -> tuple[Image, float, float]:
        """
        Convert OpenCV image to ROS Image message.
        
        Args:
            cv_image: OpenCV image in BGR format
            
        Returns:
            Tuple of (ROS Image message, conversion_time, message_creation_time)
        """
        # Time BGR to RGB conversion
        conversion_start = time.time()
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        conversion_time = time.time() - conversion_start
        
        # Time ROS message creation
        message_start = time.time()
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_frame"
        
        msg.height = rgb_image.shape[0]
        msg.width = rgb_image.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = rgb_image.shape[1] * 3  # 3 bytes per pixel for RGB
        
        # Flatten image data
        msg.data = np.array(rgb_image).tobytes()
        message_creation_time = time.time() - message_start
        
        return msg, conversion_time, message_creation_time

    def _capture_and_publish(self) -> None:
        """Main capture loop running in separate thread."""
        target_interval = 1.0 / self.fps
        
        while self.running and rclpy.ok():
            loop_start = time.time()
            
            try:
                if self.cap is None or not self.cap.isOpened():
                    self.get_logger().warn("Camera not available, attempting to reconnect...")
                    if not self._initialize_camera():
                        time.sleep(1.0)  # Wait before retry
                        continue
                
                # Time the camera capture separately
                capture_start = time.time()
                ret, frame = self.cap.read()
                capture_time = time.time() - capture_start
                
                if not ret:
                    self.get_logger().warn("Failed to capture frame")
                    continue
                
                # Create and publish ROS message with detailed timing
                image_msg, conversion_time, message_creation_time = self._create_image_message(frame)
                
                # Time the publishing step
                publish_start = time.time()
                self.image_publisher.publish(image_msg)
                publish_time = time.time() - publish_start
                
                # Calculate total processing time (from capture start to publish end)
                processing_time = time.time() - capture_start
                
                # Update processing time statistics
                self.total_processing_time += processing_time
                self.min_processing_time = min(self.min_processing_time, processing_time)
                self.max_processing_time = max(self.max_processing_time, processing_time)
                
                # Update detailed timing statistics
                self.total_capture_time += capture_time
                self.total_conversion_time += conversion_time
                self.total_message_creation_time += message_creation_time
                self.total_publish_time += publish_time
                
                # Update frame statistics
                self.frame_count += 1
                
                # Log periodic statistics
                if self.frame_count % (self.fps * 15) == 0:  # Every 15 seconds
                    elapsed = time.time() - self.start_time
                    actual_fps = self.frame_count / elapsed if elapsed > 0 else 0
                    subscribers = self.image_publisher.get_subscription_count()
                    
                    # Calculate average processing times
                    avg_processing_time = self.total_processing_time / self.frame_count if self.frame_count > 0 else 0
                    avg_capture_time = self.total_capture_time / self.frame_count if self.frame_count > 0 else 0
                    avg_conversion_time = self.total_conversion_time / self.frame_count if self.frame_count > 0 else 0
                    avg_message_time = self.total_message_creation_time / self.frame_count if self.frame_count > 0 else 0
                    avg_publish_time = self.total_publish_time / self.frame_count if self.frame_count > 0 else 0
                    
                    # Calculate frame size in MB
                    frame_size_mb = (self.width * self.height * 3) / (1024 * 1024)
                    
                    self.get_logger().info(
                        f"Published {self.frame_count} frames - "
                        f"Actual FPS: {actual_fps:.1f}, "
                        f"Subscribers: {subscribers}, "
                        f"Frame size: {frame_size_mb:.2f}MB, "
                        f"Processing time - Total: {avg_processing_time*1000:.2f}ms "
                        f"(Capture: {avg_capture_time*1000:.2f}ms, "
                        f"BGR→RGB: {avg_conversion_time*1000:.2f}ms, "
                        f"Message: {avg_message_time*1000:.2f}ms, "
                        f"Publish: {avg_publish_time*1000:.2f}ms), "
                        f"Min: {self.min_processing_time*1000:.2f}ms, "
                        f"Max: {self.max_processing_time*1000:.2f}ms"
                    )
                
                # Maintain target frame rate
                loop_time = time.time() - loop_start
                sleep_time = max(0, target_interval - loop_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except Exception as e:
                self.get_logger().error(f"Error in capture loop: {str(e)}")
                time.sleep(0.1)  # Brief pause before retry

    def start(self) -> None:
        """Start the webcam capture and publishing."""
        if self.cap is None:
            self.get_logger().error("Camera not initialized")
            return
        
        self.get_logger().info("Starting webcam capture...")
        self.running = True
        
        # Start capture thread
        self.capture_thread = threading.Thread(target=self._capture_and_publish)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        try:
            # Keep the main thread alive for ROS2 spinning
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info("Shutdown requested...")
        finally:
            self.stop()

    def stop(self) -> None:
        """Stop webcam capture and cleanup resources."""
        self.get_logger().info("Stopping webcam publisher...")
        
        # Stop capture thread
        self.running = False
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        
        # Release camera
        if self.cap:
            self.cap.release()
            self.cap = None
        
        # Final statistics
        if self.frame_count > 0:
            elapsed = time.time() - self.start_time
            avg_fps = self.frame_count / elapsed if elapsed > 0 else 0
            avg_processing_time = self.total_processing_time / self.frame_count
            avg_capture_time = self.total_capture_time / self.frame_count
            avg_conversion_time = self.total_conversion_time / self.frame_count
            avg_message_time = self.total_message_creation_time / self.frame_count
            avg_publish_time = self.total_publish_time / self.frame_count
            
            self.get_logger().info(
                f"Session complete - Total frames: {self.frame_count}, "
                f"Average FPS: {avg_fps:.1f}, Duration: {elapsed:.1f}s, "
                f"Processing time - Total: {avg_processing_time*1000:.2f}ms "
                f"(Capture: {avg_capture_time*1000:.2f}ms, "
                f"BGR→RGB: {avg_conversion_time*1000:.2f}ms, "
                f"Message: {avg_message_time*1000:.2f}ms, "
                f"Publish: {avg_publish_time*1000:.2f}ms), "
                f"Min: {self.min_processing_time*1000:.2f}ms, "
                f"Max: {self.max_processing_time*1000:.2f}ms"
            )
        
        self.destroy_node()


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="ROS2 Webcam Publisher - Stream webcam video as ROS Image messages",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        "--camera-id", "-c",
        type=int,
        default=0,
        help="Camera device ID (default: 0 for built-in webcam)"
    )
    
    parser.add_argument(
        "--width", "-w",
        type=int,
        default=640,
        help="Image width in pixels (default: 640)"
    )
    
    parser.add_argument(
        "--height",
        type=int,
        default=480,
        help="Image height in pixels (default: 480)"
    )
    
    parser.add_argument(
        "--fps", "-f",
        type=float,
        default=30.0,
        help="Target frames per second (default: 30.0)"
    )
    
    parser.add_argument(
        "--topic", "-t",
        type=str,
        default="camera/image_raw",
        help="ROS topic name for publishing images (default: camera/image_raw)"
    )
    
    parser.epilog = """
Examples:
  # Basic usage with default settings
  python3 webcam_pub.py
  
  # High resolution at 60 FPS
  python3 webcam_pub.py --width 1920 --height 1080 --fps 60
  
  # Use external USB camera
  python3 webcam_pub.py --camera-id 1
  
  # Custom topic name
  python3 webcam_pub.py --topic robot/front_camera/image_raw

Note: Actual resolution and FPS may be limited by camera capabilities.
    """
    
    return parser.parse_args()


def main(args=None):
    """Main entry point."""
    # Parse arguments
    parsed_args = parse_arguments()
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create and start webcam publisher
        webcam_publisher = WebcamPublisher(
            camera_id=parsed_args.camera_id,
            width=parsed_args.width,
            height=parsed_args.height,
            fps=parsed_args.fps,
            topic_name=parsed_args.topic
        )
        
        webcam_publisher.start()
        
    except Exception as e:
        print(f"Failed to start webcam publisher: {e}")
        sys.exit(1)
    finally:
        # Cleanup ROS2
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main() 
