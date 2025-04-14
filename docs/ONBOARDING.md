# Open-Teleop Developer Onboarding Guide

This document provides all the information you need to set up your development environment and start contributing to the Open-Teleop platform.

## Project Overview

Open-Teleop is a distributed system that enables remote operation of robots by providing a standardized way to stream sensor data, video, and command information between robots and operators. The architecture consists of:

1. **Robot ROS Nodes** - The native ROS2 ecosystem running on the robot
2. **Bridge Nodes Container** - Python/C++ bridges that connect to ROS topics and forward data
3. **Controller Container** - Go-based services that process data and provide APIs

For a more detailed overview, see [architecture.md](architecture.md).

## Development Prerequisites

You'll need the following installed on your development machine:

- **ROS2 Jazzy or newer** - For running and developing bridge nodes
- **Go 1.24 or newer** - For controller development
- **ZeroMQ** - For communication between components
- **Docker and Docker Compose** - For containerized development and testing
- **FlatBuffers compiler (flatc)** - For interface code generation
- **Git** - For version control
- **Python 3.8+** - For ROS2 development
- **colcon** - For building ROS2 packages

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

The following Python packages are needed for the ROS2 bridges:
- pyzmq (Python ZeroMQ bindings)

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

# Install ZeroMQ
sudo apt-get install -y libzmq3-dev python3-pip
pip3 install pyzmq

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

#### Option 1: Building with Docker (Recommended for Full System Testing)

This builds both components in containers:

```bash
# From the project root directory
cd infra
docker-compose build
```

#### Option 2: Building Locally (Recommended for Development)

**1. Generate Interface Code**:

```bash
# From the project root directory
./scripts/generate_interfaces.sh
```

**2. Build ROS2 Bridges**:

```bash
# From the project root directory
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**3. Build Go Controller**:

```bash
# From the project root directory
cd controller
go build -o bin/controller ./cmd/controller
```

### Running the Project

#### Option 1: Running with Docker

```bash
# From the project root directory
cd infra
docker-compose up
```

#### Option 2: Running Locally

**Terminal 1 (ROS2 Bridges)**:
```bash
# From the project root directory
cd ros2_ws
source install/setup.bash
ros2 launch ros_gateway gateway.launch.py
```

**Terminal 2 (Go Controller)**:
```bash
# From the project root directory
cd controller
./bin/controller
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
./run_integration_tests.sh
```

## Development Best Practices

### Code Organization

- Follow standard ROS2 package structure in `ros2_ws/src` directory
- Keep Go code in the `controller` directory

### Contribution Workflow

1. Create a feature branch from `main`
2. Make your changes
3. Run tests to ensure everything works
4. Submit a pull request

### Debugging Tips

- Use `ros2 topic echo` to view ROS2 topics
- Check logs in both the ROS2 bridge and Go controller
- Use Docker logs for containerized deployment

## Project Roadmap

See [PROJECT_PLAN.md](PROJECT_PLAN.md) for the current development roadmap.

## Getting Help

- Check the documentation in the `docs` directory
- Submit issues on GitHub
- Reach out to the core development team 