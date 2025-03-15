# Open-Teleop Architecture

## System Architecture

The Open-Teleop platform consists of two primary containers that work together to enable remote operation of robots.

```
┌─────────────────┐        ┌─────────────────┐        ┌─────────────┐
│                 │        │                 │        │             │
│  Web Browser    │◄─────► │  Go Controller  │◄─────► │  ROS2       │
│  (Operator)     │WebRTC  │  Container B    │ROS Msg │  Bridge     │◄─────► Robot
│                 │        │                 │        │  Container A│
└─────────────────┘        └─────────────────┘        └─────────────┘
```

### Container A: ROS2 Bridge

The ROS2 Bridge container interfaces directly with ROS2-based robots. Its primary responsibilities include:

1. **Video Streaming Bridge**: Subscribes to camera topics on the robot and prepares video frames for streaming
2. **Teleop Command Bridge**: Forwards operator commands to the robot's control system
3. **Launch System**: Orchestrates the bridge components and manages the ROS2 connections

### Container B: Go Controller

The Go Controller container manages the operator interface and WebRTC connections. Its components include:

1. **HTTP Server**: Serves the web interface to operators
2. **WebRTC Service**: Handles low-latency video streaming to operators
3. **Teleop Service**: Processes and validates operator commands
4. **Configuration Manager**: Manages system settings and robot profiles

## Data Flow

1. The robot publishes video frames and sensor data to ROS2 topics
2. The ROS2 Bridge (Container A) subscribes to these topics
3. Video frames are processed and forwarded to the Go Controller (Container B)
4. The Go Controller streams video to the operator via WebRTC
5. The operator sends control commands through the web interface
6. Commands are validated by the Go Controller and sent to the ROS2 Bridge
7. The ROS2 Bridge publishes the commands to the appropriate ROS2 topics
8. The robot acts on the received commands

## Security Considerations

- WebRTC connections are secured with DTLS
- Operator authentication will be implemented in future versions
- Command validation prevents unsafe operations
- Network isolation between containers

## Performance Considerations

- WebRTC provides low-latency video streaming essential for teleoperation
- Minimal processing in the command path to reduce latency
- Video compression settings balance quality and bandwidth
- Containerization enables scaling for multiple robot connections 