#!/bin/bash

# Open-Teleop Development Environment Setup
# This script sets up the development environment for Open-Teleop

# Detect the absolute path to the project root
PROJECT_ROOT="$(cd "$(dirname "$(dirname "${BASH_SOURCE[0]}")")" && pwd)"

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print banner
echo -e "${BLUE}"
echo "  ____                   _____    _                    "
echo " / __ \                 |_   _|  | |                   "
echo "| |  | |_ __   ___ _ __   | | ___| | ___  ___  _ __    "
echo "| |  | | '_ \ / _ \ '_ \  | |/ _ \ |/ _ \/ _ \| '_ \   "
echo "| |__| | |_) |  __/ | | | | |  __/ |  __/ (_) | |_) |  "
echo " \____/| .__/ \___|_| |_| \_/\___|_|\___|\___/| .__/   "
echo "       | |                                     | |      "
echo "       |_|                                     |_|      "
echo -e "${NC}"
echo "Development Environment Setup"

# Source ROS2
echo -e "${YELLOW}Sourcing ROS2 Jazzy...${NC}"
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo -e "${GREEN}✓ ROS2 Jazzy sourced${NC}"
else
    echo -e "${YELLOW}⚠️ ROS2 Jazzy setup.bash not found at /opt/ros/jazzy/setup.bash${NC}"
    echo "Please make sure ROS2 Jazzy is installed."
fi

# Source ROS2 workspace if built
if [ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]; then
    echo -e "${YELLOW}Sourcing Open-Teleop ROS2 workspace...${NC}"
    source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
    echo -e "${GREEN}✓ Open-Teleop ROS2 workspace sourced${NC}"
else
    echo -e "${YELLOW}⚠️ Open-Teleop ROS2 workspace not built yet${NC}"
    echo "Run 'scripts/build.sh --ros2-only' to build it"
fi

# Set ROS_DOMAIN_ID for isolation
export ROS_DOMAIN_ID=42
echo -e "${GREEN}✓ Set ROS_DOMAIN_ID=42${NC}"

# Set up environment variables for the controller
export OPEN_TELEOP_ROOT="$PROJECT_ROOT"
export OPEN_TELEOP_ROS2_BRIDGE="$PROJECT_ROOT/ros2_ws"
export OPEN_TELEOP_CONTROLLER="$PROJECT_ROOT/controller"
export OPEN_TELEOP_SCHEMAS="$PROJECT_ROOT/schemas"

# Add controller's bin directory to PATH if it exists
if [ -d "$PROJECT_ROOT/controller/bin" ]; then
    export PATH="$PATH:$PROJECT_ROOT/controller/bin"
    echo -e "${GREEN}✓ Added controller/bin to PATH${NC}"
fi

# Set environment variables for development
export OPEN_TELEOP_DEV_MODE=true
export PORT=8080
export WEBRTC_PORT=8888
export ZMQ_ADDRESS="tcp://*:5555"

echo -e "${GREEN}✓ Environment variables set for development${NC}"

# Display environment information
echo -e "\n${BLUE}=== Open-Teleop Development Environment ===${NC}"
echo -e "Project root: ${YELLOW}$PROJECT_ROOT${NC}"
echo -e "ROS_DOMAIN_ID: ${YELLOW}$ROS_DOMAIN_ID${NC}"
echo -e "Dev mode: ${YELLOW}enabled${NC}"
echo -e "Controller port: ${YELLOW}$PORT${NC}"
echo -e "WebRTC port: ${YELLOW}$WEBRTC_PORT${NC}"
echo -e "ZeroMQ address: ${YELLOW}$ZMQ_ADDRESS${NC}"

echo -e "\n${GREEN}Development environment is ready!${NC}"
echo "Run the following commands in separate terminals to start development:"
echo -e "  ${YELLOW}cd $PROJECT_ROOT/ros2_ws && source ../scripts/dev_env.sh && ros2 launch launch/all_bridges.launch.py${NC}"
echo -e "  ${YELLOW}cd $PROJECT_ROOT/controller && source ../scripts/dev_env.sh && ./bin/controller${NC}"

# This script should be sourced, not executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo -e "\n${YELLOW}⚠️  This script should be sourced, not executed:${NC}"
    echo -e "  ${YELLOW}source scripts/dev_env.sh${NC}"
fi 