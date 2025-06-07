# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Architecture Overview

Open-Teleop is a distributed robotics teleoperation platform with three main components:

1. **ROS Gateway** (`ros2_ws/src/ros_gateway/`) - Python ROS2 node that bridges ROS topics to/from the controller via ZeroMQ using FlatBuffers serialization
2. **Go Controller** (`controller/`) - Main backend that processes messages, serves web UI, and handles client connections via WebSocket/WebRTC/REST
3. **Media Gateway** (`media-gateway/`) - Python service for hardware A/V capture and encoding using GStreamer

Communication flow: ROS Topics ↔ ROS Gateway ↔ Controller ↔ Web Clients

## Build Commands

**Main build script**: `./scripts/build.sh`

```bash
# Build everything (recommended)
./scripts/build.sh all

# Build individual components
./scripts/build.sh gateway      # ROS Gateway only
./scripts/build.sh controller   # Go controller only  
./scripts/build.sh fbs          # Generate FlatBuffers interfaces only
./scripts/build.sh av_node      # A/V node only

# Clean all build artifacts
./scripts/build.sh clean

# Install to directory
./scripts/build.sh install [directory]
```

**Run scripts** (auto-build if needed):
```bash
./scripts/run_controller.sh     # Go controller
./scripts/run_gateway.sh        # ROS gateway
./scripts/run_media_gateway.sh  # Media gateway
./scripts/run_av_node.sh        # A/V node
```

## Development Setup

**Prerequisites**: ROS2 Jazzy, Go 1.24+, FlatBuffers compiler, ZeroMQ dev libraries, Python 3.8+

```bash
# Check/install dependencies
./scripts/dev_setup.sh

# Typical development workflow
./scripts/build.sh all
./scripts/run_gateway.sh     # Terminal 1
./scripts/run_controller.sh  # Terminal 2
```

## Key Configuration

**Main config**: `config/open_teleop_config.yaml`
- Topic mappings between ROS topics and OTT identifiers
- Message priorities (HIGH/STANDARD/LOW) and directions (INBOUND/OUTBOUND)
- Throttle rates per priority level

**Controller config**: `config/controller_config.yaml`
- ZeroMQ addresses (5555 for requests, 5556 for publishing)
- Processing pool worker counts
- HTTP server port (8080)

## Important Details

- **FlatBuffers**: Schema at `schemas/ott_message.fbs` generates interface code for both Go and Python
- **Message Processing**: Priority-based pools in controller handle HIGH/STANDARD/LOW priority messages
- **ROS Integration**: Uses CGO/C++ library (`controller/pkg/rosparser/cpp/`) for ROS message parsing
- **Web UI**: Static files in `controller/web/static/` served by Fiber framework
- **ZeroMQ Ports**: 5555 (request/reply), 5556 (publish/subscribe)
- **Build Dependencies**: ROS packages built in order: `open_teleop_logger` → `open_teleop_msgs` → `ros_gateway` → `open_teleop_av_node`

## Testing

- Test scripts in `tests/ros2-scripts/` for development and debugging
- Use `ros2 topic pub/echo` commands for testing topic mappings
- Web UI accessible at `http://localhost:8080` when controller is running

## Docker Deployment

```bash
# Multi-container deployment
docker-compose up

# Single runtime container  
docker build -f Dockerfile.runtime -t open-teleop-runtime .
docker run --rm -p 8080:8080 open-teleop-runtime
```