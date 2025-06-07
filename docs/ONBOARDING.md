# Open-Teleop Developer Onboarding Guide

This document provides all the information you need to set up your development environment and start contributing to the Open-Teleop platform.

## Project Overview

Open-Teleop is a distributed robotics teleoperation platform that enables remote operation of robots by providing a standardized way to stream sensor data, video, and command information between robots and operators. The architecture consists of five main components:

1. **ROS Gateway** (`ros2_ws/src/ros_gateway/`) - Python ROS2 node that bridges ROS topics to/from the controller via ZeroMQ using FlatBuffers serialization
2. **Go Controller** (`controller/`) - Main backend that processes messages, serves web UI, and handles client connections via WebSocket/WebRTC/REST
3. **Media Gateway** (`media-gateway/`) - Python service for hardware A/V capture and encoding using GStreamer
4. **A/V Node** (`ros2_ws/src/open_teleop_av_node/`) - ROS2 node that handles encoded video frame processing and streaming
5. **Web UI** (`controller/web/static/`) - Frontend interface for teleoperation control and video streaming

Communication flow: ROS Topics ↔ ROS Gateway ↔ Controller ↔ Web Clients

For a more detailed overview, see [architecture.md](architecture.md).

## Development Prerequisites

You'll need the following installed on your development machine:

- **ROS2 Jazzy or newer** - For running and developing bridge nodes
- **Go 1.24 or newer** - For controller development
- **ZeroMQ** - For communication between components
- **Docker and Docker Compose** - For containerized development and testing
- **FlatBuffers compiler (flatc)** - For interface code generation
- **Git** - For version control
- **Python 3.8+** - For ROS2 development and Media Gateway
- **colcon** - For building ROS2 packages
- **GStreamer** - For media capture and encoding in Media Gateway

### Required ROS2 Packages

The following ROS2 packages are required:
- ros-jazzy-cv-bridge
- ros-jazzy-image-transport
- ros-jazzy-audio-common
- ros-jazzy-navigation2
- ros-jazzy-nav2-bringup

### Required Go Packages

All required Go packages are specified in the `go.mod` file. They will be automatically installed when building.

### Required Python Packages

The following Python packages are needed for the ROS2 bridges and Media Gateway:
- pyzmq (Python ZeroMQ bindings)
- gstreamer-python (for Media Gateway)
- opencv-python (for image processing)

## Setting Up Your Development Environment

### Step 1: Clone the Repository

```bash
git clone <repository-url> open-teleop
cd open-teleop
```

### Step 2: Install Dependencies

#### Ubuntu/Debian:

```bash
# Install ROS2 Jazzy (if not already installed)
# See: https://docs.ros.org/en/jazzy/Installation.html

# Install additional ROS2 packages
sudo apt-get update
sudo apt-get install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-audio-common \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup

# Install ZeroMQ and GStreamer
sudo apt-get install -y libzmq3-dev python3-pip \
    gstreamer1.0-tools gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly python3-gi python3-gi-cairo \
    gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0

# Install Python packages
pip3 install pyzmq opencv-python

# Install Go (if not already installed)
# See: https://golang.org/doc/install

# Install FlatBuffers compiler
sudo apt-get install -y flatbuffers-compiler

# Install Docker (if not already installed)
# See: https://docs.docker.com/engine/install/ubuntu/

# Install Docker Compose (if not already installed)
# See: https://docs.docker.com/compose/install/
```

### Step 3: Set Up Environment Variables

Add the following to your `~/.bashrc` or equivalent:

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source Open-Teleop workspace (after building)
source ~/path/to/open-teleop/ros2_ws/install/setup.bash

# Set ROS_DOMAIN_ID for isolation
export ROS_DOMAIN_ID=42
```

## Development Workflow

### Building the Project

Use the main build script for streamlined building:

```bash
# Build everything (recommended)
./scripts/build.sh all

# Build individual components
./scripts/build.sh gateway      # ROS Gateway only
./scripts/build.sh controller   # Go controller only  
./scripts/build.sh fbs          # Generate FlatBuffers interfaces only
./scripts/build.sh av_node      # A/V node only

# Clean all build artifacts
./scripts/build.sh clean

# Install to directory
./scripts/build.sh install [directory]
```

#### Alternative: Manual Building Steps

If you need to build components manually:

**1. Generate Interface Code**:
```bash
./scripts/generate_interfaces.sh
```

**2. Build ROS2 Components**:
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**3. Build Go Controller**:
```bash
cd controller
go build -o bin/controller ./cmd/controller
```

**4. Build Media Gateway**:
```bash
cd media-gateway
pip install -e .
```

### Running the Project

Use the provided run scripts for easy startup:

```bash
# Run individual components (auto-builds if needed)
./scripts/run_controller.sh     # Go controller
./scripts/run_gateway.sh        # ROS gateway
./scripts/run_media_gateway.sh  # Media gateway
./scripts/run_av_node.sh        # A/V node
```

#### Typical Development Workflow

**Terminal 1 (ROS Gateway)**:
```bash
./scripts/run_gateway.sh
```

**Terminal 2 (Go Controller)**:
```bash
./scripts/run_controller.sh
```

**Terminal 3 (Media Gateway - if using video)**:
```bash
./scripts/run_media_gateway.sh
```

**Terminal 4 (A/V Node - if using video)**:
```bash
./scripts/run_av_node.sh
```

#### Docker Deployment

```bash
# Multi-container deployment
docker-compose up

# Single runtime container  
docker build -f Dockerfile.runtime -t open-teleop-runtime .
docker run --rm -p 8080:8080 open-teleop-runtime
```

### Accessing the System

Once running, you can access:
- Web Interface: http://localhost:8080
- API Endpoints: http://localhost:8080/api/v1/...

## Testing the Project

### Running Unit Tests

**ROS2 Bridge Tests**:
```bash
# From the project root directory
cd ros2_ws
colcon test
```

**Go Controller Tests**:
```bash
# From the project root directory
cd controller
go test ./...
```

### Running Integration Tests

```bash
# From the project root directory
cd tests
# Use test scripts in tests/ros2-scripts/ for development and debugging
python tests/ros2-scripts/pub.py  # Test publisher
python tests/ros2-scripts/sub.py  # Test subscriber
```

## Development Best Practices

### Code Organization

- Follow standard ROS2 package structure in `ros2_ws/src` directory
- Keep Go code in the `controller` directory
- Media Gateway Python code lives in `media-gateway/` directory
- Web UI static files are in `controller/web/static/`
- Configuration files are in `config/` directory

### Contribution Workflow

1. Create a feature branch from `main`
2. Make your changes
3. Run tests to ensure everything works
4. Submit a pull request

### Debugging Tips

- Use `ros2 topic echo` to view ROS2 topics
- Check logs in ROS2 gateway, Go controller, and Media Gateway
- Web UI accessible at `http://localhost:8080` when controller is running
- ZeroMQ ports: 5555 (request/reply), 5556 (publish/subscribe)
- Use test scripts in `tests/ros2-scripts/` for development and debugging

## Project Roadmap

See [PROJECT_PLAN.md](PROJECT_PLAN.md) for the current development roadmap.

## Getting Help

- Check the documentation in the `docs` directory
- Submit issues on GitHub
- Reach out to the core development team 