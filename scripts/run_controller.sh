#!/bin/bash

# Open-Teleop Controller Run Script
# This script runs the controller, finding the binary and config directory.

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Get the absolute path to the project root
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_DIR="${PROJECT_ROOT}/config"
CONTROLLER_BIN="${PROJECT_ROOT}/controller/bin/controller"

# Check if controller binary exists
if [ ! -f "$CONTROLLER_BIN" ]; then
  echo -e "${YELLOW}Controller binary not found. Building now...${NC}"
  cd "$PROJECT_ROOT" && ./scripts/build.sh controller
  
  # Check if build succeeded
  if [ ! -f "$CONTROLLER_BIN" ]; then
    echo -e "${RED}Failed to build controller${NC}"
    exit 1
  fi
fi

# Run the controller
echo -e "${GREEN}Running controller with config from: ${CONFIG_DIR}${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
echo ""

# Run the controller, pointing it to the config directory
"$CONTROLLER_BIN" -config-dir "$CONFIG_DIR" 