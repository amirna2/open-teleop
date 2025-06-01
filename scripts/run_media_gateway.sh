#!/bin/bash

# Script to run the Open Teleop Media Gateway

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
MEDIA_GATEWAY_DIR="$PROJECT_ROOT/media-gateway"

print_status "Setting environment for Open Teleop Media Gateway..."

# Check if media-gateway directory exists
if [ ! -d "$MEDIA_GATEWAY_DIR" ]; then
    print_error "Error: media-gateway directory not found in $PROJECT_ROOT."
    print_error "Please run ./scripts/build_media_gateway.sh first."
    exit 1
fi

# Change to media gateway directory
cd "$MEDIA_GATEWAY_DIR"

# Check if virtual environment exists
if [ ! -d ".venv" ]; then
    print_error "Error: Virtual environment not found in $MEDIA_GATEWAY_DIR/.venv"
    print_error "Please run ./scripts/build_media_gateway.sh first."
    exit 1
fi

# Activate virtual environment
source .venv/bin/activate
print_status "Activated virtual environment: $MEDIA_GATEWAY_DIR/.venv"

# Verify installation
if ! python -c "import media_gateway" 2>/dev/null; then
    print_error "Error: Media Gateway package not installed"
    print_error "Please run ./scripts/build_media_gateway.sh first."
    exit 1
fi

# Set environment variables for better debugging
export GST_DEBUG_DUMP_DOT_DIR="$PROJECT_ROOT/logs/gstreamer-dots"
export PYTHONPATH="$MEDIA_GATEWAY_DIR:$PYTHONPATH"

# Create log directories
mkdir -p "$PROJECT_ROOT/logs/gstreamer-dots"

print_status "Launching Open Teleop Media Gateway..."

# Execute the media gateway with all passed arguments
python -m media_gateway.main "$@"

EXIT_CODE=$?
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}Media Gateway finished gracefully.${NC}"
else
    echo -e "${RED}Media Gateway exited with code $EXIT_CODE.${NC}"
fi

exit $EXIT_CODE 