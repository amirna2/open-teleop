# ROS Gateway

## Overview

The ROS Gateway is a bridge between ROS2 and the Open-Teleop Controller, enabling seamless communication between robots and remote operators. It handles:

- Dynamic topic subscriptions based on configuration
- Message conversion between ROS and OttMessage format
- Priority-based message handling
- Bidirectional communication with the controller

## Architecture

The ROS Gateway consists of these main components:

1. **Config Processor**: Loads and validates the configuration
2. **Topic Subscriber**: Subscribes to configured ROS topics
3. **Message Converter**: Converts ROS messages to OttMessages
4. **ZeroMQ Client**: Communicates with the controller
5. **Topic Publisher**: Publishes commands to ROS topics

## Usage

### Prerequisites

- ROS2 Humble/Iron
- Python 3.8+
- ZeroMQ
- FlatBuffers

### Installation

1. Clone this repository into your ROS2 workspace
2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ros_gateway
   source install/setup.bash
   ```

### Configuration

Edit the `config/gateway_config.yaml` file to configure:

- Topic mappings between ROS and Open-Teleop
- ZeroMQ connection settings
- Logging parameters

Example configuration:

```yaml
gateway:
  zmq:
    controller_address: "tcp://localhost:5555"
    publish_address: "tcp://*:5556"
  
  topic_mappings:
    - ros_topic: "/camera/image_raw"
      ott: "teleop.video.main_camera"
      priority: "HIGH"
      message_type: "sensor_msgs/msg/Image"
    
    - ros_topic: "/cmd_vel"
      ott: "teleop.control.velocity"
      direction: "INBOUND"
      message_type: "geometry_msgs/msg/Twist"
```

### Running

Launch the ROS Gateway using:

```bash
ros2 launch ros_gateway gateway.launch.py
```

Or with a custom config:

```bash
ros2 launch ros_gateway gateway.launch.py config_path:=/path/to/custom_config.yaml
```

## Development

### Adding New Topic Types

The ROS Gateway automatically handles any ROS message type through dynamic imports. To add a new topic:

1. Add the mapping to the configuration file
2. Ensure any custom message packages are installed
3. Restart the gateway

### Testing

Run the tests with:

```bash
colcon test --packages-select ros_gateway
```

## License

MIT License 