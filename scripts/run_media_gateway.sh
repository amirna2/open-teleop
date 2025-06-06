#!/bin/bash

# Run script for the Open Teleop Media Gateway

set -e

# --- Configuration ---
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m'

# --- Helper Functions ---
print_status() {
    echo -e "${YELLOW}$1${NC}"
}

print_error() {
    echo -e "${RED}$1${NC}"
}

# --- Script Body ---
# Find project root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." &> /dev/null && pwd )"
MEDIA_GATEWAY_DIR="$PROJECT_ROOT/media-gateway"
VENV_DIR="$MEDIA_GATEWAY_DIR/.venv"

# 1. Check if virtual environment exists
if [ ! -d "$VENV_DIR" ]; then
    print_error "Error: Virtual environment not found at $VENV_DIR"
    print_error "Please run the build script first: ./scripts/build_media_gateway.sh"
    exit 1
fi

# 2. Activate virtual environment
source "$VENV_DIR/bin/activate"

# 3. Check if the entry point executable exists
if ! command -v open-teleop-media-gateway &> /dev/null; then
    print_error "Error: 'open-teleop-media-gateway' command not found."
    print_error "This indicates a problem with the installation."
    print_error "Please try running the build script again: ./scripts/build_media_gateway.sh"
    exit 1
fi

# 4. Launch the Media Gateway
print_status "Launching Open Teleop Media Gateway..."

# The 'open-teleop-media-gateway' command was created by the build script.
# Any arguments passed to this script will be forwarded to the gateway.
cd "$MEDIA_GATEWAY_DIR"
if [ $# -eq 0 ]; then
    print_status "No arguments provided, using default config: config/media-gateway-config.yaml"
    open-teleop-media-gateway --config-path "config/media-gateway-config.yaml"
else
    open-teleop-media-gateway "$@"
fi

EXIT_CODE=$?
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}Media Gateway finished gracefully.${NC}"
else
    echo -e "${RED}Media Gateway exited with code $EXIT_CODE.${NC}"
fi

exit $EXIT_CODE 