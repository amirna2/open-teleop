# Open-Teleop: Robotics Teleoperation Platform

A platform for remote operation of robots using WebRTC for low-latency video streaming and command transmission.

## Project Overview

This project implements a robotics teleoperation system with two main components:

1. **ROS2 Workspace (Container A)**: Connects to ROS2-based robots and bridges video streams, audio streams, sensor data, navigation commands, and teleop commands
2. **Go Controller (Container B)**: Provides WebRTC services and command handling for web-based clients

## Project Structure

```
open-teleop/
├── ros2_ws/                # Container A: ROS2 workspace
│   ├── src/
│   │   ├── video_streaming_bridge/
│   │   ├── audio_streaming_bridge/
│   │   ├── sensor_bridge/
│   │   ├── navigation_bridge/
│   │   └── teleop_command_bridge/
│   ├── launch/
│   │   └── all_bridges.launch.py
│   └── Dockerfile
├── controller/            # Container B: Go Controller
│   ├── cmd/
│   │   └── controller/
│   │       └── main.go
│   ├── domain/
│   │   ├── video/
│   │   ├── audio/
│   │   ├── sensor/
│   │   ├── navigation/
│   │   └── teleop/
│   ├── pkg/
│   │   ├── webrtc/
│   │   └── zero_mq/
│   └── Dockerfile
├── infra/                 
│   └── docker-compose.yml
├── docs/                  
├── tests/                 
└── config/                
```

## Getting Started

1. Run the setup script to create the project structure:
   ```
   ./setup_project_structure_corrected.sh
   ```

2. Follow development steps as outlined in the project plan:
   - Initialize the Go module in controller
   - Set up the ROS2 workspace in ros2_ws
   - Implement minimal bridges and controller components
   - Create Docker containers and orchestration

## Development Plan

See `docs/development_guide.md` for the detailed development plan and next steps.

## Architecture

See `docs/architecture.md` for the system architecture and component interaction details.
