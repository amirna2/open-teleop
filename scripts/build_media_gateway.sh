#!/bin/bash

set -e

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Cleanup function
clean_media_gateway() {
    echo -e "${YELLOW}Cleaning Media Gateway build artifacts...${NC}"
    
    cd media-gateway
    
    # Remove virtual environment
    if [ -d ".venv" ]; then
        echo "Removing virtual environment..."
        rm -rf .venv
    fi
    
    # Remove Python cache files
    echo "Removing Python cache files..."
    find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
    find . -type f -name "*.pyc" -delete 2>/dev/null || true
    find . -type f -name "*.pyo" -delete 2>/dev/null || true
    
    # Remove build artifacts
    echo "Removing build artifacts..."
    rm -rf build/ dist/ *.egg-info/ .pytest_cache/ 2>/dev/null || true
    
    # Remove log files
    echo "Removing log files..."
    rm -rf logs/ 2>/dev/null || true
    
    echo -e "${GREEN}✓ Media Gateway cleanup complete${NC}"
    cd ..
}

# Check for clean argument
if [ "$1" = "clean" ]; then
    clean_media_gateway
    exit 0
fi

echo "Building Media Gateway..."

# Install system packages
echo "Installing system packages..."
sudo apt update
sudo apt install -y \
    python3-gi python3-gi-cairo gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0 gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav libzmq3-dev python3-dev \
    python3-pip python3-venv \
    v4l-utils alsa-utils libudev-dev

# Create venv
echo "Setting up virtual environment..."
cd media-gateway
python3 -m venv .venv
source .venv/bin/activate

# Install Python packages  
echo "Installing Python packages..."
pip install --upgrade pip
pip install -r requirements.txt
pip install -e .

echo "Media Gateway build complete!" 