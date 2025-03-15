# Open-Teleop Development Guide

This guide outlines the revised development plan and implementation phases for the Open-Teleop platform.

## Development Phases

### Phase 1: Project Setup (Complete)

- [x] Create project structure
- [x] Initialize Go module for Controller with Fiber framework
- [ ] Set up ROS2 workspace for Bridge
- [ ] Configure basic Docker containers

### Phase 2: Diagnostic Service Implementation (Current Focus)

#### Diagnostic Bridge Node
- [ ] Create ROS2 node to collect system metrics (CPU, memory, etc.)
- [ ] Implement periodic publishing of diagnostic data
- [ ] Create interface for transmitting metrics to Go Controller

#### Diagnostic Service in Go Controller
- [ ] Set up basic service structure
- [ ] Create endpoints for retrieving diagnostic data
- [ ] Implement simple dashboard for visualization
- [ ] Establish communication with diagnostic bridge

### Phase 3: Teleop Service Implementation

#### Teleop Command Bridge
- [ ] Create ROS2 node to publish command messages
- [ ] Implement command validation
- [ ] Create interface for receiving commands from Go Controller

#### Teleop Service in Go Controller
- [ ] Implement command handling
- [ ] Create command validation logic
- [ ] Set up communication with ROS2 Bridge
- [ ] Create simple UI for sending commands

### Phase 4: Container and Orchestration Setup

- [ ] Refine Dockerfiles for both containers
- [ ] Configure Docker Compose for local development
- [ ] Set up networking between containers
- [ ] Implement health checks and restart policies

### Phase 5: Testing and Verification

- [ ] Implement integration tests
- [ ] Create test harness for diagnostics
- [ ] Create test harness for teleop commands
- [ ] Test with robot simulator

### Future Phases (Post-MVP)

#### Video Streaming Implementation
- [ ] Create ROS2 node to subscribe to camera topics
- [ ] Implement frame processing (compression, encoding)
- [ ] Implement WebRTC signaling server in Go Controller
- [ ] Create video streaming pipeline
- [ ] Set up STUN/TURN configuration

#### Additional Features
- [ ] Add authentication system
- [ ] Implement multi-robot support
- [ ] Add sensor data visualization
- [ ] Implement audio communication
- [ ] Create recording and playback functionality

## Development Guidelines

### Go Development
- Use Go modules for dependency management
- Follow standard Go project layout
- Implement unit tests for all packages
- Use context for cancellation
- Follow idiomatic Go practices
- Use Fiber for HTTP server and API

### ROS2 Development
- Use ROS2 Humble or later
- Implement nodes as composable components
- Use parameter system for configuration
- Follow ROS2 naming conventions
- Create launch files for all components

### Docker Development
- Use multi-stage builds to minimize container size
- Create separate development and production configurations
- Use volumes for persistent data
- Implement proper networking between containers
- Follow Docker best practices

## Next Steps (Immediate Focus)

1. **Create Diagnostic Bridge Node**:
   ```bash
   cd ros2_ws/src
   ros2 pkg create --build-type ament_cmake diagnostic_bridge --dependencies rclcpp std_msgs diagnostic_msgs
   ```

2. **Implement Diagnostic Service**:
   ```bash
   cd controller/domain
   mkdir -p diagnostic
   touch diagnostic/diagnostic_service.go
   ```

3. **Testing the Integration**:
   - Test diagnostic data flow from ROS2 to Go Controller
   - Verify metrics are accurately reported
   - Test simple dashboard visualization 