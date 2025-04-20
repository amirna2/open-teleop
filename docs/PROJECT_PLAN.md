# Revised Project Plan for Open-Teleop

For a minimal viable implementation of this robotics teleoperation platform, I will focus on these core components in a more incremental approach:

## Core Components (In Priority Order)

### 1. Container A: Universal Bridge (ROS Gateway)
- Implements the `ros_gateway` package.
- **Functionality:** Handles communication between the ROS environment and the Go controller via ZeroMQ. Includes topic subscription/publication, message conversion, ZeroMQ client logic, and configuration processing. Aligns with the "UNIVERSAL BRIDGE" layer in the architecture diagram.
- Simplified launch file for the gateway node.
- Dockerfile located at `ros2_ws/Dockerfile` to build this container.

### 2. Container B: Go Controller
- **Functionality:** Main backend application handling external communication (WebRTC, WebSocket, REST, gRPC - although initial implementation might be simpler) and internal message processing. Includes session/connection management, priority-based processing pools, ZeroMQ message receiving/directing, and ROS message parsing, aligning with the "CONTROLLER" layer in the architecture diagram.
- Configuration manager: Simple configuration loading.
- Dockerfile located at `controller/Dockerfile` to build this container.

### 3. Infrastructure
- **Current Deployment:** A single runtime container is built using `Dockerfile.runtime`. This container packages pre-built artifacts for both the Universal Bridge (ROS Gateway) and the Go Controller, likely launched together via an entrypoint script (e.g., `scripts/start.sh`).
- **Target/Alternative Deployment:** The `docker-compose.yml` file in the `infra` directory provides the configuration to build and orchestrate the Universal Bridge and Go Controller as separate services using their respective Dockerfiles (`ros2_ws/Dockerfile` and `controller/Dockerfile`).
- **Networking:** Appropriate networking (e.g., Docker bridge network) is configured either within the single runtime container's environment or via `docker-compose` for the multi-container setup.

### 4. Testing
- Basic test for diagnostics reporting
- Basic test for teleop commands
- Integration test to verify end-to-end communication

## Revised Development Plan

### Phase 1: Project Setup (Complete)
- Create directory structure
- Initialize Go module
- Set up ROS2 workspace

### Phase 2: Implement Core Communication Pipeline (ROS Gateway <-> Controller) (Mostly Complete - Dashboard Pending)
- Implement initial `ros_gateway` node functionality (basic topic handling, ZeroMQ connection) - (Complete)
- Implement basic message receiving/handling logic in Go controller - (Complete)
- Establish ZeroMQ communication pattern between gateway and controller - (Complete)
- Create simple dashboard for metrics visualization - (Pending)

### Phase 3: Implement Teleop Command Path (Controller -> Gateway -> Robot) (Pending)
- Implement teleop command generation/handling logic within the Go controller service - (Pending)
- Utilize the existing `ros_gateway`'s publishing capabilities to send commands to ROS topics - (Gateway capability exists, Controller integration Pending)
- Create simple interface/API endpoint in the controller for sending teleop commands - (Pending)
- Test end-to-end teleop command path (Interface -> Controller -> Gateway -> ROS) - (Pending)

### Phase 4: Containerization and Orchestration (Mostly Complete)
- Refine `Dockerfile.runtime` for building the single-container deployment packaging both gateway and controller. (Complete)
- Develop/Refine `docker-compose.yml` and associated Dockerfiles (`ros2_ws/Dockerfile`, `controller/Dockerfile`) for the alternative multi-container deployment. (Complete)
- Ensure proper communication and networking configurations for both deployment strategies. (Ongoing/Verification needed)

### Phase 5: Basic Testing and Verification (Partially Pending)
- Verify core communication pipeline (Gateway -> Controller) for general message flow. (Verified)
- Test end-to-end teleop command path (Controller -> Gateway -> Robot) once implemented (Phase 3). (Pending)
- Test basic system functionality using both deployment strategies (single container via `Dockerfile.runtime`, multi-container via `docker-compose`). (Pending)
- Develop Minimal Web UI for Testing: Implement a basic static HTML/JavaScript frontend served directly by the Go controller (Fiber). This UI will use REST APIs and WebSockets provided by the controller to facilitate end-to-end testing (e.g., visualizing data flow, sending basic commands) and serve as a foundation for potential future admin/diagnostic panels. (Pending)
- Utilize a simple robot simulator for testing if a physical robot is unavailable. (Ongoing Approach)

### Future Phases (Post-MVP)
- Video streaming with WebRTC (more complex)
- Audio communication
- Sensor data visualization
- Navigation planning
- Multi-robot support

## Rationale for Revised Approach

This revised approach provides several benefits:
1. **Lower Initial Complexity**: Starting with diagnostics establishes the foundation without the complexity of video streaming
2. **Faster Time to Value**: Get a working system more quickly with basic functionality
3. **Risk Mitigation**: Identify architecture issues early with simpler components
4. **Clear Progression**: Each phase builds directly on the previous one

Video streaming is deferred to a future phase as it involves more complexity with WebRTC, signaling servers, and media handling. By building the system incrementally, we'll have a more solid foundation when we add these advanced features.
