#!/bin/bash

# Open-Teleop Development Setup Script
# This script checks for all required dependencies and sets up the development environment

echo "=== Open-Teleop Development Setup ==="
echo "Checking dependencies..."

# Check ROS2 installation
if [ -f /opt/ros/jazzy/setup.bash ]; then
    echo "✅ ROS2 Jazzy found"
else
    echo "❌ ROS2 Jazzy not found. Please install it first."
    echo "   See: https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi

# Check Go installation
if command -v go &> /dev/null; then
    GO_VERSION=$(go version | awk '{print $3}' | sed 's/go//')
    echo "✅ Go found: $GO_VERSION"
    
    # Compare version
    if [ "$(printf '%s\n' "1.18" "$GO_VERSION" | sort -V | head -n1)" != "1.18" ]; then
        echo "⚠️ Warning: Go version should be 1.18 or higher for optimal compatibility"
    fi
else
    echo "❌ Go not found. Please install Go."
    echo "   See: https://golang.org/doc/install"
    exit 1
fi

# Check ZeroMQ installation
if [ -f /usr/include/zmq.h ] || [ -f /usr/local/include/zmq.h ]; then
    echo "✅ ZeroMQ development files found"
else
    echo "❌ ZeroMQ development files not found."
    echo "   Install with: sudo apt-get install -y libzmq3-dev"
    exit 1
fi

# Check Python ZeroMQ bindings
if python3 -c "import zmq" &> /dev/null; then
    echo "✅ Python ZeroMQ bindings found"
else
    echo "❌ Python ZeroMQ bindings not found."
    echo "   Install with: pip3 install pyzmq"
    exit 1
fi

# Check Docker installation
if command -v docker &> /dev/null; then
    DOCKER_VERSION=$(docker --version | awk '{print $3}' | sed 's/,//')
    echo "✅ Docker found: $DOCKER_VERSION"
else
    echo "❌ Docker not found. Please install Docker."
    echo "   See: https://docs.docker.com/engine/install/"
    exit 1
fi

# Check Docker Compose installation
if command -v docker-compose &> /dev/null; then
    COMPOSE_VERSION=$(docker-compose --version | awk '{print $3}' | sed 's/,//')
    echo "✅ Docker Compose found: $COMPOSE_VERSION"
else
    echo "❌ Docker Compose not found. Please install Docker Compose."
    echo "   See: https://docs.docker.com/compose/install/"
    exit 1
fi

# Check FlatBuffers compiler
if command -v flatc &> /dev/null; then
    FLATC_VERSION=$(flatc --version)
    echo "✅ FlatBuffers compiler found: $FLATC_VERSION"
else
    echo "❌ FlatBuffers compiler not found. Please install it."
    echo "   Run: sudo apt-get install flatbuffers-compiler"
    exit 1
fi

# Check required ROS2 packages
MISSING_PACKAGES=()
ROS_PACKAGES=(
    "ros-jazzy-cv-bridge"
    "ros-jazzy-image-transport"
    "ros-jazzy-audio-common"
    "ros-jazzy-navigation2"
    "ros-jazzy-nav2-bringup"
)

echo "Checking ROS2 packages..."
for PKG in "${ROS_PACKAGES[@]}"; do
    if dpkg -l | grep -q "$PKG"; then
        echo "✅ $PKG found"
    else
        echo "❌ $PKG not found"
        MISSING_PACKAGES+=("$PKG")
    fi
done

if [ ${#MISSING_PACKAGES[@]} -ne 0 ]; then
    echo -e "\n⚠️ The following ROS2 packages are missing:"
    printf "   %s\n" "${MISSING_PACKAGES[@]}"
    echo -e "\nYou can install them with:"
    echo "   sudo apt-get install" "${MISSING_PACKAGES[@]}"
    echo
fi

# Check project directory structure
echo "Checking project structure..."
MISSING_DIRS=()

if [ ! -d "ros2_ws" ]; then
    MISSING_DIRS+=("ros2_ws")
fi

if [ ! -d "controller" ]; then
    MISSING_DIRS+=("controller")
fi

if [ ! -d "schemas" ]; then
    MISSING_DIRS+=("schemas")
fi

if [ ${#MISSING_DIRS[@]} -ne 0 ]; then
    echo -e "\n⚠️ The following project directories are missing:"
    printf "   %s\n" "${MISSING_DIRS[@]}"
    echo "Please make sure you're running this script from the project root directory."
    echo
fi

echo -e "\n=== Setup Summary ==="
if [ ${#MISSING_PACKAGES[@]} -eq 0 ] && [ ${#MISSING_DIRS[@]} -eq 0 ]; then
    echo "🎉 All dependencies are installed and project structure looks good!"
    echo "You're ready to develop Open-Teleop."
else
    echo "⚠️ Some dependencies or project directories are missing."
    echo "Please address the issues above before continuing."
fi

echo -e "\nTo set up your environment variables, add the following to your ~/.bashrc:"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   source $(pwd)/ros2_ws/install/setup.bash  # After building"
echo "   export ROS_DOMAIN_ID=42"

# Source ROS2 setup for the current session
source /opt/ros/jazzy/setup.bash 