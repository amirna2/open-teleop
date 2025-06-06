#!/bin/bash

set -e

# The root of the repository
PROJECT_ROOT=$(git rev-parse --show-toplevel)
MEDIA_GATEWAY_DIR="$PROJECT_ROOT/media-gateway"

# Function to handle cleanup
clean_media_gateway() {
    echo "Cleaning Media Gateway build artifacts..."
    cd "$MEDIA_GATEWAY_DIR"
    
    rm -rf .venv
    find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
    rm -rf build/ dist/ *.egg-info/ .pytest_cache/ 2>/dev/null || true
    rm -rf logs/ 2>/dev/null || true
    rm -rf media_gateway/flatbuffers 2>/dev/null || true
    
    echo "Media Gateway cleanup complete."
    cd "$PROJECT_ROOT"
}

# Handle 'clean' argument
if [ "$1" = "clean" ]; then
    clean_media_gateway
    exit 0
fi

echo "Building Media Gateway..."

# System package dependencies
PACKAGES=(
    "python3-gi"
    "python3-gi-cairo"
    "gir1.2-gstreamer-1.0"
    "gir1.2-gst-plugins-base-1.0"
    "gstreamer1.0-plugins-good"
    "gstreamer1.0-plugins-bad"
    "gstreamer1.0-plugins-ugly"
    "gstreamer1.0-libav"
    "libzmq3-dev"
    "python3-dev"
    "python3-pip"
    "python3-venv"
    "v4l-utils"
    "alsa-utils"
    "libudev-dev"
    "flatbuffers-compiler"
)

# Check and install system packages
# This is a simplified check. A more robust solution might be desired for a real project.
if ! dpkg -l "${PACKAGES[@]}" >/dev/null 2>&1; then
    echo "Installing missing system packages..."
    sudo apt-get update
    sudo apt-get install -y "${PACKAGES[@]}"
else
    echo "All system packages are already installed."
fi

# Navigate to the component directory
cd "$MEDIA_GATEWAY_DIR"

# Create python virtual environment if it doesn't exist
if [ ! -d ".venv" ]; then
    echo "Creating Python virtual environment..."
    python3 -m venv .venv --system-site-packages
fi

# Activate virtual environment
source .venv/bin/activate

# Generate flatbuffers code
echo "Generating flatbuffers code..."
FLATBUFFERS_DIR="media_gateway/flatbuffers"
mkdir -p "$FLATBUFFERS_DIR"
flatc --python -o "$FLATBUFFERS_DIR" "$PROJECT_ROOT/schemas/ott_message.fbs"

# Create __init__.py files to ensure the generated code is a package
touch "$FLATBUFFERS_DIR/__init__.py"
find "$FLATBUFFERS_DIR" -type d -exec touch {}/__init__.py \;

# Install/update Python dependencies
echo "Installing Python packages..."
pip install --upgrade pip
pip install -r requirements.txt
pip install -e .

echo "Media Gateway build complete!" 