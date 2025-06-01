#!/bin/bash

set -e

echo "Building Media Gateway..."

# Install system packages
echo "Installing system packages..."
sudo apt update
sudo apt install -y \
    python3-gi python3-gi-cairo gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0 gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav libzmq3-dev python3-dev \
    python3-pip python3-venv

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

echo "Done!" 