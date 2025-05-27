#!/usr/bin/env python3

"""
ROS2 Image Viewer Node

This node subscribes to ROS Image messages and displays them using OpenCV.
Useful for testing and debugging image publishers like the webcam publisher.

Features:
- Subscribe to raw Image messages
- Real-time image display using OpenCV
- Frame rate monitoring
- Keyboard controls for interaction
"""

import argparse
import sys
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image


class ImageViewer(Node):
    """
    ROS2 node that subscribes to Image messages and displays them using OpenCV.
    """
    
    def __init__(self, topic_name: str = "camera/image_raw", window_name: str = "ROS2 Image Viewer"):
        """
        Initialize the image viewer.
        
        Args:
            topic_name: ROS topic name to subscribe to
            window_name: OpenCV window name
        """
        super().__init__('image_viewer')
        
        self.topic_name = topic_name
        self.window_name = window_name
        
        # Statistics tracking
        self.frame_count = 0
        self.start_time = time.time()
        self.last_fps_update = time.time()
        self.fps_frame_count = 0
        self.current_fps = 0.0
        
        # Setup OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        
        # Setup ROS2 subscriber
        self._setup_subscriber()
        
        self.get_logger().info(f"Image viewer initialized - Topic: {topic_name}")
        self.get_logger().info("Press 'q' in the image window to quit")

    def _setup_subscriber(self) -> None:
        """Setup ROS2 subscriber with appropriate QoS settings."""
        # QoS profile matching typical image publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.image_subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            qos
        )

    def image_callback(self, msg: Image) -> None:
        """
        Callback function for received Image messages.
        
        Args:
            msg: ROS Image message
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self._ros_to_cv_image(msg)
            
            if cv_image is not None:
                # Update statistics
                self._update_statistics()
                
                # Add FPS overlay
                self._add_info_overlay(cv_image, msg)
                
                # Display image
                cv2.imshow(self.window_name, cv_image)
                
                # Handle keyboard input (non-blocking)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info("Quit requested by user")
                    rclpy.shutdown()
                    
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def _ros_to_cv_image(self, msg: Image) -> Optional[np.ndarray]:
        """
        Convert ROS Image message to OpenCV image.
        
        Args:
            msg: ROS Image message
            
        Returns:
            OpenCV image or None if conversion failed
        """
        try:
            # Convert data to numpy array
            if msg.encoding == "rgb8":
                # RGB format
                image_array = np.array(msg.data, dtype=np.uint8)
                cv_image = image_array.reshape((msg.height, msg.width, 3))
                # Convert RGB to BGR for OpenCV
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                
            elif msg.encoding == "bgr8":
                # BGR format (already OpenCV compatible)
                image_array = np.array(msg.data, dtype=np.uint8)
                cv_image = image_array.reshape((msg.height, msg.width, 3))
                
            elif msg.encoding == "mono8":
                # Grayscale format
                image_array = np.array(msg.data, dtype=np.uint8)
                cv_image = image_array.reshape((msg.height, msg.width))
                
            else:
                self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
                return None
                
            return cv_image
            
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {str(e)}")
            return None

    def _update_statistics(self) -> None:
        """Update frame rate statistics."""
        self.frame_count += 1
        self.fps_frame_count += 1
        
        current_time = time.time()
        
        # Update FPS every second
        if current_time - self.last_fps_update >= 1.0:
            elapsed = current_time - self.last_fps_update
            self.current_fps = self.fps_frame_count / elapsed
            self.fps_frame_count = 0
            self.last_fps_update = current_time

    def _add_info_overlay(self, cv_image: np.ndarray, msg: Image) -> None:
        """
        Add information overlay to the image.
        
        Args:
            cv_image: OpenCV image to add overlay to
            msg: Original ROS Image message
        """
        # Prepare text information
        info_lines = [
            f"Topic: {self.topic_name}",
            f"Resolution: {msg.width}x{msg.height}",
            f"Encoding: {msg.encoding}",
            f"FPS: {self.current_fps:.1f}",
            f"Frame: {self.frame_count}",
            f"Frame ID: {msg.header.frame_id}",
            "Press 'q' to quit"
        ]
        
        # Text properties
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        color = (0, 255, 0)  # Green
        thickness = 1
        line_height = 20
        
        # Add background rectangle for better readability
        overlay_height = len(info_lines) * line_height + 10
        overlay = cv_image.copy()
        cv2.rectangle(overlay, (5, 5), (300, overlay_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, cv_image, 0.3, 0, cv_image)
        
        # Add text lines
        for i, line in enumerate(info_lines):
            y_position = 20 + i * line_height
            cv2.putText(cv_image, line, (10, y_position), font, font_scale, color, thickness)

    def run(self) -> None:
        """Run the image viewer."""
        try:
            self.get_logger().info("Starting image viewer...")
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info("Shutdown requested...")
        finally:
            self.cleanup()

    def cleanup(self) -> None:
        """Cleanup resources."""
        self.get_logger().info("Cleaning up image viewer...")
        
        # Close OpenCV windows
        cv2.destroyAllWindows()
        
        # Print final statistics
        if self.frame_count > 0:
            elapsed = time.time() - self.start_time
            avg_fps = self.frame_count / elapsed if elapsed > 0 else 0
            self.get_logger().info(
                f"Session complete - Total frames: {self.frame_count}, "
                f"Average FPS: {avg_fps:.1f}, Duration: {elapsed:.1f}s"
            )
        
        self.destroy_node()


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="ROS2 Image Viewer - Display ROS Image messages using OpenCV",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        "--topic", "-t",
        type=str,
        default="camera/image_raw",
        help="ROS topic name to subscribe to (default: camera/image_raw)"
    )
    
    parser.add_argument(
        "--window-name", "-w",
        type=str,
        default="ROS2 Image Viewer",
        help="OpenCV window name (default: ROS2 Image Viewer)"
    )
    
    parser.epilog = """
Examples:
  # Basic usage with default settings
  python3 image_viewer.py
  
  # Subscribe to custom topic
  python3 image_viewer.py --topic robot/front_camera/image_raw
  
  # Custom window name
  python3 image_viewer.py --window-name "Robot Camera Feed"

Controls:
  q - Quit the viewer
    """
    
    return parser.parse_args()


def main(args=None):
    """Main entry point."""
    # Parse arguments
    parsed_args = parse_arguments()
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create and run image viewer
        image_viewer = ImageViewer(
            topic_name=parsed_args.topic,
            window_name=parsed_args.window_name
        )
        
        image_viewer.run()
        
    except Exception as e:
        print(f"Failed to start image viewer: {e}")
        sys.exit(1)
    finally:
        # Cleanup ROS2
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main() 