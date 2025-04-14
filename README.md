# Open-Teleop: Robotics Teleoperation Platform

A platform for remote operation of robots using WebRTC for low-latency video streaming and command transmission, bridging web clients with ROS 2 systems.

## Project Overview

This project implements a robotics teleoperation system with two main components:

1.  **ROS 2 Workspace (`ros2_ws`)**: Contains the `ros_gateway` node, which acts as a bridge between the ROS 2 ecosystem (topics, services, actions) and the controller. It handles message serialization/deserialization and communication. Includes supporting packages like `open_teleop_logger`.
2.  **Go Controller (`controller`)**: Manages communication with web clients (likely via WebSockets or WebRTC data channels, TBD), handles signaling, processes commands, routes data via ZeroMQ, and orchestrates the teleoperation session.

## Project Structure

```
open-teleop/
├── ros2_ws/                # Container A: ROS2 workspace
│   ├── src/
│   │   ├── video_streaming_bridge/
│   │   ├── audio_streaming_bridge/
│   │   ├── sensor_bridge/
│   │   ├── navigation_bridge/
│   │   └── teleop_command_bridge/
│   ├── launch/
│   │   └── all_bridges.launch.py
│   └── Dockerfile
├── controller/            # Container B: Go Controller
│   ├── cmd/
│   │   └── controller/
│   │       └── main.go
│   ├── domain/
│   │   ├── video/
│   │   ├── audio/
│   │   ├── sensor/
│   │   ├── navigation/
│   │   └── teleop/
│   ├── pkg/
│   │   ├── webrtc/
│   │   └── zero_mq/
│   └── Dockerfile
├── infra/                 
│   └── docker-compose.yml
├── docs/                  
├── tests/                 
└── config/                
```

## Getting Started

### Prerequisites

Ensure you have the following installed:

*   ROS 2 Jazzy (including `colcon`)
*   Go (version 1.21 or later recommended)
*   FlatBuffers Compiler (`flatc`) - usually via `sudo apt install flatbuffers-compiler`
*   Docker & Docker Compose (for containerized deployment)
*   ZeroMQ Development Libraries (`libzmq3-dev`) - usually via `sudo apt install libzmq3-dev`
*   Python 3 ZMQ library (`python3-zmq`) - usually via `sudo apt install python3-zmq`
*   Python 3 Flatbuffers library (`python3-flatbuffers`) - usually via `sudo apt install python3-flatbuffers`

*(Note: Check `scripts/dev_setup.sh` if available for a more automated dependency check/install process)*

### Building the Platform

From the project root directory:

```bash
# Clean previous build artifacts (optional but recommended)
./scripts/build.sh clean

# Build all components (ROS gateway, Go controller, generate interfaces)
./scripts/build.sh all
```

### Running on Host

1.  **Terminal 1 (ROS Gateway):**
    ```bash
    cd ros2_ws
    source install/setup.bash
    # Adjust launch file name if needed
    ros2 launch ros_gateway gateway.launch.py 
    ```

2.  **Terminal 2 (Go Controller):**
    ```bash
    cd controller
    # Run the controller (use flags for config/logging if needed)
    ./bin/controller 
    ```

### Running with Docker

1.  **Build the Image:**
    ```bash
    # This assumes the Dockerfile is configured for runtime
    docker build -t open-teleop-runtime . 
    ```
    *(Note: This image will build and contain the application ready to run)*

2.  **Run the Container:** 
    *(Requires updating the Dockerfile CMD/ENTRYPOINT to run the gateway & controller - see future steps)*
    ```bash
    # Placeholder - final command depends on chosen startup method
    docker run --rm -p 8080:8080 open-teleop-runtime 
    ```
    *(Alternatively, use `docker-compose up` if the `infra/docker-compose.yml` is configured)*

## Development

See `docs/development_guide.md` for potential development workflows and guidelines (may require updates).

## Architecture

See `docs/architecture.md` for system architecture details (may require updates).
