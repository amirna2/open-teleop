# Open-Teleop Media Gateway

The Media Gateway is a dedicated component for high-performance audio/video capture and streaming in the Open-Teleop system. It accesses hardware devices like cameras and microphones directly, encodes their output into standard formats (H.264, Opus), and streams the data to the central controller.

This component is designed to run on the robot and provides the low-latency, high-quality media streams required for production teleoperation.

## Setup and Installation

### 1. System Prerequisites

Before building, you must install the required system-level development libraries. These are necessary to build the Python packages that interface with GStreamer and system hardware.

On Debian-based systems (like Ubuntu), you can install them with the following command:

```bash
sudo apt update && sudo apt install -y \
    build-essential \
    python3-dev \
    libgirepository1.0-dev \
    gir1.2-gtk-3.0 \
    python3-gi \
    python3-gi-cairo \
    libcairo2-dev \
    pkg-config \
    libglib2.0-dev \
    libudev-dev \
    libzmq3-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    v4l-utils \
    alsa-utils
```

### 2. Build the Gateway

Once the system prerequisites are installed, you can build the gateway using the provided script. This will set up a Python virtual environment using `uv` and install all necessary Python packages.

```bash
./scripts/build_media_gateway.sh
```

The build script will:
1. Check for `uv` package manager
2. Create a Python virtual environment with system site packages enabled
3. Install the media gateway package in editable mode

### 3. Run the Gateway

After a successful build, you can run the media gateway with this command:

```bash
./scripts/run_media_gateway.sh
```

The script will automatically use a default configuration file. To provide a custom configuration, pass it as an argument:

```bash
./scripts/run_media_gateway.sh --config-path /path/to/your/config.yaml
```

## Overview

The Media Gateway bridges hardware devices (cameras, microphones) to the Open Teleop protocol, bypassing ROS2 limitations for real-time video streaming. It uses GStreamer for hardware acceleration and supports multiple encoding formats.

## Architecture

```
Hardware Devices → Media Gateway → ZeroMQ → Controller → Web UI
```

### Core Components

- **Config Manager**: Requests and parses configuration from Controller
- **Device Manager**: Discovers and manages hardware devices (V4L2, ALSA)
- **Pipeline Manager**: Creates and manages GStreamer pipelines
- **Frame Router**: Routes encoded frames to appropriate outputs
- **Connection Manager**: Handles ZeroMQ transport to Controller

## Requirements

- Python 3.8+
- GStreamer 1.0+ with development headers
- PyGObject (GI bindings for GStreamer)
- ZeroMQ
- Hardware devices (USB cameras, microphones)

### System Dependencies

```bash
# Ubuntu/Debian
sudo apt install python3-gi python3-gi-cairo gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0 gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav libzmq3-dev

# Fedora/RHEL
sudo dnf install python3-gobject gstreamer1-devel \
    gstreamer1-plugins-good gstreamer1-plugins-bad-free \
    gstreamer1-plugins-ugly-free gstreamer1-libav zeromq-devel
```

## Configuration

The Media Gateway uses the same configuration pattern as other Open Teleop components, extending the YAML configuration with a `media_mappings` section:

```yaml
media_mappings:
  video:
    - device_id: "video0"
      ott: "teleop.video.main_camera"
      encoding_format: "video/h264"
      encoder_params:
        width: 1920
        height: 1080
        framerate: 30
        bitrate: 4000
        gop_size: 15
      priority: "HIGH"
```

## Usage

```bash
# Build and install
./scripts/build_media_gateway.sh

# Run with default configuration
./scripts/run_media_gateway.sh

# Run with custom config URL
./scripts/run_media_gateway.sh --controller-url http://localhost:8080
```

## Development

### Project Structure

```
media-gateway/
├── media_gateway/           # Main package
│   ├── __init__.py
│   ├── main.py             # Entry point
│   ├── config/             # Configuration management
│   ├── devices/            # Hardware device management
│   ├── pipelines/          # GStreamer pipeline management
│   ├── transport/          # ZeroMQ transport
│   └── routing/            # Frame routing
├── tests/                  # Unit tests
├── requirements.txt        # Python dependencies
├── setup.py               # Package setup
└── README.md              # This file
```

### Performance Considerations

The Media Gateway is designed for easy migration to C++ for performance-critical components:

- **Interface-driven design**: Clean abstractions for C++ migration
- **Minimal Python overhead**: Critical paths use minimal Python processing
- **Modular architecture**: Individual components can be replaced with C++ implementations
- **Performance monitoring**: Built-in profiling to identify bottlenecks

## Testing

```bash
# Run unit tests
python -m pytest tests/

# Test device discovery
python -m media_gateway.devices.discovery --list

# Test pipeline creation
python -m media_gateway.pipelines.test --device /dev/video0
```

## Troubleshooting

### Common Issues

1. **No devices found**: Check USB permissions and V4L2 device accessibility
2. **GStreamer errors**: Verify GStreamer plugins are installed
3. **Permission denied**: Add user to `video` and `audio` groups
4. **Pipeline fails**: Check device capabilities match configuration

### Debug Mode

```bash
# Enable debug logging
export GST_DEBUG=3
export PYTHONPATH=/path/to/media-gateway:$PYTHONPATH
python -m media_gateway.main --debug
```

## License

This project is part of the Open Teleop platform and follows the same licensing terms. 