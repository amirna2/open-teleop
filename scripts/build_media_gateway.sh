#!/bin/bash

# Build script for the Open Teleop Media Gateway using uv

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

print_success() {
    echo -e "${GREEN}$1${NC}"
}

print_error() {
    echo -e "${RED}$1${NC}"
}

print_warning() {
    echo -e "${YELLOW}$1${NC}"
}

# --- Script Body ---
# Find project root (assuming script is in project_root/scripts)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." &> /dev/null && pwd )"
MEDIA_GATEWAY_DIR="$PROJECT_ROOT/media-gateway"
VENV_DIR="$MEDIA_GATEWAY_DIR/.venv"

# 1. Prerequisite Check: Verify uv is installed
print_status "Step 1: Checking for uv..."
if ! command -v uv &> /dev/null; then
    print_error "Error: 'uv' command not found."
    print_error "Please install uv first. See: https://github.com/astral-sh/uv"
    exit 1
fi
print_success "uv is installed."

# 2. Create Virtual Environment
print_status "Step 2: Creating virtual environment with uv..."
cd "$MEDIA_GATEWAY_DIR"
uv venv -p python3.12 "$VENV_DIR" --seed --system-site-packages
print_success "Virtual environment created at $VENV_DIR"

# 3. Install Package in Editable Mode
print_status "Step 3: Installing media-gateway package in editable mode..."
# Activate the virtual environment for this script's context
source "$VENV_DIR/bin/activate"

# Make the build resilient by finding and setting the pkg-config path
print_status "--> Searching for system libraries to ensure a resilient build..."
GI_PC_FILE=$(find /usr -name "girepository-2.0.pc" 2>/dev/null | head -n 1)

if [ -n "$GI_PC_FILE" ]; then
    GI_PC_DIR=$(dirname "$GI_PC_FILE")
    print_status "--> Found GObject Introspection at $GI_PC_DIR. Temporarily setting PKG_CONFIG_PATH."
    export PKG_CONFIG_PATH=$GI_PC_DIR${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}
else
    print_warning "--> WARNING: Could not automatically find 'girepository-2.0.pc'."
    print_warning "--> The build may fail if pkg-config cannot find it in the default paths."
    print_warning "--> This can happen in non-standard environments. Ensure 'libgirepository1.0-dev' is installed."
fi

uv pip install -e .

print_success "\nBuild complete!"
print_success "The 'open-teleop-media-gateway' command is now available in the virtual environment."
print_status "\nTo run the gateway, use the dedicated run script:"
print_status "  ./scripts/run_media_gateway.sh" 