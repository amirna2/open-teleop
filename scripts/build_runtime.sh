#!/bin/bash
# Script to build the Open-Teleop runtime container

set -e  # Exit on error

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Building Open-Teleop runtime container${NC}"

# Ensure we're in the project root
if [ ! -d "scripts" ] || [ ! -d "controller" ] || [ ! -d "ros2_ws" ]; then
    echo -e "${RED}Error: Not in the project root directory.${NC}"
    echo -e "${RED}Please run this script from the root of the Open-Teleop project.${NC}"
    exit 1
fi

# Clean any previous builds
echo -e "${YELLOW}Cleaning previous builds...${NC}"
./scripts/build.sh clean

# Build and install all components
echo -e "${YELLOW}Building and installing components...${NC}"
./scripts/build.sh install

# Check if the install directory was created and has contents
if [ ! -d "install" ] || [ -z "$(ls -A install 2>/dev/null)" ]; then
    echo -e "${RED}Error: install directory is missing or empty!${NC}"
    echo -e "${RED}The installation step did not complete successfully.${NC}"
    exit 1
fi

# Build runtime container from installed components
echo -e "${YELLOW}Building runtime container...${NC}"
docker build -t open-teleop-runtime:latest -f Dockerfile.runtime .

echo -e "${GREEN}Runtime container built successfully!${NC}"
echo ""
echo "To run the container:"
echo "  docker run --network host --name open-teleop open-teleop-runtime:latest"
echo ""
echo "To run with custom configuration:"
echo "  docker run --network host -v \$(pwd)/config:/opt/open-teleop/config open-teleop-runtime:latest" 