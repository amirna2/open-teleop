# ROS2 Scripts Collection

This directory contains a collection of ROS2 scripts for testing, development, and demonstration purposes. These scripts are designed to support the Open-Teleop platform development and provide examples of ROS2 best practices.

## Scripts Overview

### 1. Publisher Scripts

#### `pub.py` - Multi-Topic Data Publisher
A comprehensive ROS2 publisher that generates synthetic data for various message types.

**Features:**
- Publishes to 20+ different ROS2 message types
- Coordinated localization simulation with dynamic obstacles
- Realistic sensor data generation (LiDAR, cameras, IMU, etc.)
- Configurable publishing rate
- Selective topic publishing

**Usage:**
```bash
# Publish all topics at 0.2 Hz
python3 pub.py

# Publish specific topics at 1 Hz
python3 pub.py --topics image pointcloud2 odometry --rate 1.0

# List available topics
python3 pub.py --help
```

#### `webcam_pub.py` - Webcam Video Publisher
Captures video from laptop webcam and publishes as raw ROS2 Image messages.

**Features:**
- Real-time webcam capture using OpenCV
- Configurable resolution and frame rate
- Proper ROS2 QoS settings for video streaming
- Error handling and camera reconnection
- Performance monitoring and statistics

**Usage:**
```bash
# Basic usage with default settings (640x480 @ 30 FPS)
python3 webcam_pub.py

# High resolution capture
python3 webcam_pub.py --width 1920 --height 1080 --fps 60

# Use external USB camera
python3 webcam_pub.py --camera-id 1

# Custom topic name
python3 webcam_pub.py --topic robot/front_camera/image_raw

# Low resolution for testing
python3 webcam_pub.py --width 320 --height 240 --fps 15
```

**Command Line Options:**
- `--camera-id, -c`: Camera device ID (default: 0)
- `--width, -w`: Image width in pixels (default: 640)
- `--height`: Image height in pixels (default: 480)
- `--fps, -f`: Target frames per second (default: 30.0)
- `--topic, -t`: ROS topic name (default: camera/image_raw)

### 2. Subscriber Scripts

#### `sub.py` - Universal Subscriber
A flexible ROS2 subscriber that can listen to multiple topic types with formatted output.

**Features:**
- Subscribe to any supported message type
- Colored terminal output with statistics
- Binary message inspection
- Real-time message rate monitoring
- Selective topic subscription

**Usage:**
```bash
# Subscribe to specific topics
python3 sub.py imagepub pointcloud2pub

# Subscribe to all available topics
python3 sub.py --all

# Show binary message content
python3 sub.py imagepub --binary

# List available topics
python3 sub.py --list
```

#### `image_viewer.py` - Image Message Viewer
Subscribes to ROS2 Image messages and displays them using OpenCV.

**Features:**
- Real-time image display with OpenCV
- Frame rate monitoring and overlay
- Support for multiple image encodings (rgb8, bgr8, mono8)
- Interactive controls (press 'q' to quit)
- Performance statistics

**Usage:**
```bash
# View default camera topic
python3 image_viewer.py

# View custom topic
python3 image_viewer.py --topic robot/front_camera/image_raw

# Custom window name
python3 image_viewer.py --window-name "Robot Camera Feed"
```

## Testing Workflows

### 1. Webcam Video Streaming Test

Test the complete video pipeline from webcam capture to display:

**Terminal 1 - Start webcam publisher:**
```bash
cd tests/ros2-scripts
python3 webcam_pub.py --width 640 --height 480 --fps 30
```

**Terminal 2 - View the video stream:**
```bash
cd tests/ros2-scripts
python3 image_viewer.py --topic camera/image_raw
```

**Terminal 3 - Monitor with universal subscriber:**
```bash
cd tests/ros2-scripts
python3 sub.py imagepub
```

### 2. Multi-Sensor Simulation Test

Test comprehensive sensor data publishing and monitoring:

**Terminal 1 - Start multi-topic publisher:**
```bash
cd tests/ros2-scripts
python3 pub.py --topics image pointcloud2 odometry laserscan occupancygrid --rate 2.0
```

**Terminal 2 - Monitor all topics:**
```bash
cd tests/ros2-scripts
python3 sub.py --all
```

**Terminal 3 - View generated images:**
```bash
cd tests/ros2-scripts
python3 image_viewer.py --topic test/image_prog
```

### 3. Performance Testing

Test system performance under different loads:

**High-frequency publishing:**
```bash
python3 webcam_pub.py --width 1920 --height 1080 --fps 60
```

**Multiple publishers:**
```bash
# Terminal 1
python3 webcam_pub.py --topic camera1/image_raw --camera-id 0

# Terminal 2 (if you have multiple cameras)
python3 webcam_pub.py --topic camera2/image_raw --camera-id 1

# Terminal 3
python3 pub.py --rate 10.0
```

## Dependencies

### Required Python Packages
- `rclpy` - ROS2 Python client library
- `opencv-python` - Computer vision library for camera access
- `numpy` - Numerical computing
- `Pillow` - Image processing library
- `colorama` - Colored terminal output

### ROS2 Message Packages
- `sensor_msgs` - Sensor data messages (Image, PointCloud2, LaserScan, etc.)
- `geometry_msgs` - Geometric data messages (Pose, Twist, etc.)
- `nav_msgs` - Navigation messages (OccupancyGrid, Odometry, Path)
- `std_msgs` - Standard messages (String, Int32, Float64, etc.)
- `visualization_msgs` - Visualization messages (Marker, MarkerArray)
- `rcl_interfaces` - ROS2 core interface messages

### System Requirements
- ROS2 (Humble, Iron, or Jazzy)
- Linux with camera access permissions
- OpenCV-compatible camera (built-in webcam or USB camera)

## Installation

1. **Install ROS2** (if not already installed):
   ```bash
   # Follow official ROS2 installation guide for your distribution
   ```

2. **Install Python dependencies:**
   ```bash
   pip3 install opencv-python numpy Pillow colorama
   ```

3. **Make scripts executable:**
   ```bash
   chmod +x tests/ros2-scripts/*.py
   ```

4. **Test camera access:**
   ```bash
   python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera available:', cap.isOpened()); cap.release()"
   ```

## Troubleshooting

### Camera Issues

**Camera not found:**
- Check if camera is connected and not used by another application
- Try different camera IDs: `--camera-id 1`, `--camera-id 2`
- Check camera permissions: `ls -l /dev/video*`

**Poor performance:**
- Reduce resolution: `--width 320 --height 240`
- Lower frame rate: `--fps 15`
- Check system resources with `htop`

### ROS2 Issues

**No messages received:**
- Check if publisher and subscriber are running
- Verify topic names match: `ros2 topic list`
- Check QoS compatibility: `ros2 topic info <topic_name> --verbose`

**High latency:**
- Use BEST_EFFORT reliability for real-time data
- Reduce queue depth in QoS settings
- Check network configuration for distributed systems

### Display Issues

**Image viewer not showing:**
- Ensure X11 forwarding is enabled for SSH sessions
- Check OpenCV installation: `python3 -c "import cv2; print(cv2.__version__)"`
- Try different image encodings

## Development Notes

### Code Quality Standards
- All scripts follow PEP 8 style guidelines
- Comprehensive error handling and logging
- Type hints for better code documentation
- Modular design with clear separation of concerns

### ROS2 Best Practices
- Appropriate QoS profiles for different data types
- Proper node lifecycle management
- Thread-safe operations for real-time processing
- Resource cleanup on shutdown

### Performance Considerations
- Efficient image conversion between OpenCV and ROS formats
- Non-blocking operations in callback functions
- Memory management for high-frequency data streams
- Configurable parameters for different use cases

## Contributing

When adding new scripts or modifying existing ones:

1. Follow the established code patterns and documentation style
2. Include comprehensive error handling
3. Add command-line argument parsing with help text
4. Update this README with usage examples
5. Test with different ROS2 distributions when possible

## License

These scripts are part of the Open-Teleop project and follow the project's licensing terms. 