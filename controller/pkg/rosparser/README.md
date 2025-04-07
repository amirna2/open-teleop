# ROS Parser for Open-Teleop

This package provides a Go wrapper around a C++ ROS2 message parsing library. It allows Go code to parse raw ROS2 messages using the ROS2 type introspection system.

## Components

- `cpp/ros_parser.h` - C interface for the ROS Parser
- `cpp/ros_parser.cpp` - C++ implementation using ROS2 libraries
- `cpp/CMakeLists.txt` - CMake build file for the C++ library
- `rosparser.go` - Go wrapper using CGO to call the C++ library

## Building

To build the C++ library:

```bash
cd cpp
mkdir -p build
cd build
cmake ..
make
```

## Usage

```go
import "github.com/open-teleop/controller/pkg/rosparser"

// Initialize the parser
if err := rosparser.Initialize(); err != nil {
    log.Fatalf("Failed to initialize ROS parser: %v", err)
}
defer rosparser.Shutdown()

// Parse a ROS2 message
result, err := rosparser.ParseToJSON("sensor_msgs/msg/Image", messageData)
if err != nil {
    log.Errorf("Failed to parse message: %v", err)
    return
}

// Use the parsed message
fmt.Printf("Parsed message: %+v\n", result)
```

## Dependencies

- ROS2 (rclcpp)
- rosidl_typesupport_introspection_cpp
- nlohmann_json for C++

## Implementation Details

The parser uses the ROS2 type introspection system to dynamically parse any ROS2 message type. It works by:

1. Loading the appropriate type support library for the message type
2. Using the type introspection API to examine the message structure
3. Converting the message into a JSON representation
4. Returning the JSON to Go code

This allows parsing messages without having to generate Go code for each ROS2 message type. 