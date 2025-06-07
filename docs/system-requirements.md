# System Requirements

## Prerequisites
- **Ubuntu 24.04 Noble** (required)
- **ROS2 Jazzy** (required)
- **Python 3.12** (required)
- **Go 1.24+** (required)

## Installation

### System Dependencies
```bash
sudo apt-get update && sudo apt-get install -y \
    wget \
    libzmq3-dev \
    flatbuffers-compiler \
    python3-pip \
    python3-colcon-common-extensions \
    python3-zmq \
    python3-flatbuffers \
    python3-gi \
    libgirepository1.0-dev \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-audio-common \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rmw-cyclonedds-cpp
```

### Python Dependencies
```bash
pip3 install \
    pyzmq>=25.0.0 \
    PyYAML>=6.0 \
    flatbuffers>=23.5.26 \
    numpy \
    setuptools>=61.0 \
    pytest>=7.0.0 \
    pytest-asyncio>=0.21.0 \
    pytest-mock>=3.10.0 \
    structlog>=23.1.0 \
    psutil>=5.9.0 \
    pyudev>=0.24.0 \
    v4l2py>=0.8.0 \
    uv
```

## Environment Setup
```bash
# ROS2 environment
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Go environment  
export GOPATH=/go
export PATH=$GOPATH/bin:/usr/local/go/bin:$PATH
```

## Verification
```bash
./scripts/build.sh all
```