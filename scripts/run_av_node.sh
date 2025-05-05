#!/bin/bash

# Script to run the Open Teleop A/V Node

set -e

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${YELLOW}$1${NC}"
}

print_error() {
    echo -e "${RED}$1${NC}"
}

# Find project root (assuming script is in project_root/scripts)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." &> /dev/null && pwd )"

print_status "Setting environment for Open Teleop A/V Node..."

# Check if ros2_ws exists
if [ ! -d "$PROJECT_ROOT/ros2_ws" ]; then
    print_error "Error: ros2_ws directory not found in $PROJECT_ROOT."
    print_error "Please ensure you are in the project root and have built the workspace."
    exit 1
fi

# Source ROS2 environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    print_status "Sourced ROS2 Jazzy environment."
else
    print_error "Warning: ROS2 Jazzy setup.bash not found!"
fi

# Source local workspace overlay
WS_SETUP="$PROJECT_ROOT/ros2_ws/install/setup.bash"
if [ -f "$WS_SETUP" ]; then
    source "$WS_SETUP"
    print_status "Sourced local workspace: $WS_SETUP"
    print_status "PYTHONPATH=$PYTHONPATH" 
else
    print_error "Error: Local workspace setup file not found: $WS_SETUP"
    print_error "Please run the build script (./scripts/build.sh) first."
    exit 1
fi

# Set RMW Implementation (optional, but good for consistency)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Configure logging with microsecond precision
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time:yyyy-MM-dd HH:mm:ss.uuuuuu}] [{name}]: {message}"


print_status "Launching Open Teleop A/V Node..."

# Execute the launch file
ros2 launch open_teleop_av_node av_node.launch.py "$@"

EXIT_CODE=$?
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}A/V Node finished gracefully.${NC}"
else
    echo -e "${RED}A/V Node exited with code $EXIT_CODE.${NC}"
fi

exit $EXIT_CODE 
