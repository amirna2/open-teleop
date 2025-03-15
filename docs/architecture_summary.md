# Robotics Teleoperation Platform: Architecture Summary

## Architectural Overview

The platform consists of two main container components with a new parser integration:

1. **Container A: ROS2 Bridge**
   - Interacts with robot ROS topics/services
   - Forwards raw ROS messages via ZeroMQ/flatbuffers to Controller
   - No message parsing or transformation needed at this layer

2. **Container B: Go Controller**
   - Receives raw ROS messages from bridges
   - Leverages C++ Parser (via CGO) to parse any ROS message type
   - Manages visualization, cloud ingestion, and web services

3. **C++ Parser Component** (New Addition)
   - Integrated with Go Controller via CGO
   - Uses rclcpp for message introspection without being a full ROS node
   - Capable of parsing any ROS message type (standard or custom)

## Message Flow

1. **Discovery Phase**
   - System discovers available ROS topics when connecting to a robot
   - User selects which topics to monitor via UI

2. **Ingestion Phase**
   - ROS Bridge subscribes to selected topics
   - When messages arrive, Bridge forwards raw data to Controller
   - Data includes metadata: topic name, message type, robot ID

3. **Processing Phase**
   - Controller receives message data and metadata 
   - Controller passes raw message to C++ Parser
   - Parser uses rclcpp to parse message into structured data
   - Controller converts ROS message type to OTT (Open-Teleop-Topic)
   - Data stored and prepared for visualization/cloud ingestion

## OTT (Open-Teleop-Topic) Specification

The OTT provides a standardized naming convention for all message types:

- **Standard ROS Messages**: 
  - `sensor_msgs/BatteryState` → `teleop.sensor.battery_state`
  - `sensor_msgs/Image` → `teleop.sensor.image`

- **Custom Messages**:
  - `acme_msgs/RobotStatus` → `teleop.custom.acme_msgs.robot_status`
  - `foo/Bar` → `teleop.custom.foo.bar`

This algorithmic conversion requires no manual mapping and scales to any message type.

## Advantages of This Approach

1. **Scalability**:
   - No manual mapping required for new message types
   - Works with any robot or custom message without reconfiguration

2. **Simplicity**:
   - User only needs to select topics - system handles the rest
   - Consistent naming convention for internal processing

3. **Performance**:
   - Efficient parsing of ROS messages in C++
   - Clean data pipeline for cloud ingestion in Go

4. **Maintainability**:
   - Clear separation of concerns between components
   - Standardized terminology (OTT) for documentation and development

This architecture provides a flexible foundation that can scale with your teleoperation platform while keeping the implementation straightforward.