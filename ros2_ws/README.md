# ROS Gateway (`ros_gateway`)

## Overview

The ROS Gateway node acts as a bridge between the ROS 2 ecosystem and the Open-Teleop Go Controller, enabling seamless communication between ROS 2-based robots and remote operators. It handles:

- Dynamic topic subscriptions/publications based on configuration received from the controller.
- Message conversion between ROS native types and the OttMessage FlatBuffers format.
- Priority-based message handling (delegated by the controller).
- Bidirectional communication with the controller via ZeroMQ.

## Architecture

The ROS Gateway consists of these main components:

1.  **Node Core (`gateway_node.py`)**: Initializes ROS node, manages components.
2.  **ZeroMQ Client**: Communicates with the controller (requests config, sends ROS data, receives commands).
3.  **Config Processor**: Handles configuration received from the controller.
4.  **Topic Manager**: Dynamically creates ROS publishers and subscribers based on the received configuration.
5.  **Message Converter**: Converts messages between ROS types and OttMessages (using FlatBuffers).

*(Note: Includes dependency on `open_teleop_logger` package for structured logging)*

## Usage

### Prerequisites

- ROS 2 Jazzy
- All other prerequisites as defined in the main project `README.md` (Go, Python libs, build tools, etc.).

### Building

This package is built as part of the main project build process. From the project root:

```bash
# Clean (optional)
./scripts/build.sh clean

# Build all (includes ros_gateway)
./scripts/build.sh all
```

### Configuration

The ROS Gateway receives its operational configuration (including topic mappings, ZeroMQ details etc.) directly from the Go Controller via a ZeroMQ request during startup.

The *source* of this configuration is typically the `config/open_teleop_config.yaml` file loaded by the controller. Refer to that file for defining topic mappings like this example:

```yaml
# Example within config/open_teleop_config.yaml (under topic_mappings section)

topic_mappings:
  - ros_topic: "/camera/image_raw"       # ROS topic name
    ott: "teleop.video.main_camera"     # Corresponding Open-Teleop topic name
    message_type: "sensor_msgs/msg/Image" # ROS message type
    priority: "HIGH"                    # Priority hint for controller
    direction: "OUTBOUND"               # Gateway sends ROS -> Controller
    source_type: "ROS2_CDR"             # Data source type

  - ros_topic: "/cmd_vel"
    ott: "teleop.control.velocity"
    message_type: "geometry_msgs/msg/Twist"
    priority: "HIGH"
    direction: "INBOUND"                # Gateway receives Controller -> ROS
    source_type: "ROS2_CDR"
```

### Running

Ensure the Go controller is running first. Then, launch the ROS Gateway from the workspace root after sourcing the environment:

```bash
# In your ROS 2 terminal
cd /path/to/open-teleop/ros2_ws
source install/setup.bash
ros2 launch ros_gateway gateway.launch.py
```

## Development

### Adding New Topic Types

The ROS Gateway uses dynamic message type imports. To support a new topic type:

1.  Ensure the corresponding ROS 2 message package (e.g., `my_custom_msgs`) is installed in the environment (either host or Docker).
2.  Add the topic mapping to the central `config/open_teleop_config.yaml` file used by the controller.
3.  Restart the Go controller (to load the new config) and then restart the ROS Gateway (which will request the updated config).

## License

MIT License 