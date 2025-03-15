open-teleop/
├── docs/                    
├── config/                  
├── ros2_ws/                 # ROS 2 workspace (Container A)
│   ├── src/
│   │   ├── video_streaming_bridge/
│   │   │   ├── package.xml
│   │   │   ├── CMakeLists.txt
│   │   │   └── src/         
│   │   ├── audio_streaming_bridge/
│   │   │   ├── package.xml
│   │   │   ├── CMakeLists.txt
│   │   │   └── src/
│   │   ├── sensor_bridge/
│   │   │   ├── package.xml
│   │   │   ├── CMakeLists.txt
│   │   │   └── src/
│   │   ├── navigation_bridge/
│   │   │   ├── package.xml
│   │   │   ├── CMakeLists.txt
│   │   │   └── src/
│   │   ├── teleop_command_bridge/
│   │   │   ├── package.xml
│   │   │   ├── CMakeLists.txt
│   │   │   └── src/
│   │   └── ...
│   ├── launch/
│   │   ├── all_bridges.launch.py  # Single launch file for everything
│   │   └── ...
│   ├── Dockerfile                 # Builds the container for all bridging nodes
│   ├── README.md
│   └── colcon.default.yaml        # optional colcon config
├── controller/            # The Go monolith controller (Container B)
│   ├── cmd/
│   │   └── controller/
│   │       └── main.go    # Single entrypoint
│   ├── domain/
│   │   ├── video/
│   │   │   └── video_service.go
│   │   ├── sensor/
│   │   │   └── sensor_service.go
│   │   ├── navigation/
│   │   │   └── navigation_service.go
│   │   ├── teleop/
│   │   │   └── teleop_service.go
│   │   └── audio/
│   │       └── audio_service.go
│   ├── pkg/
│   │   ├── webrtc/
│   │   ├── zero_mq/
│   │   └── ...
│   ├── go.mod
│   ├── go.sum
│   └── Dockerfile           # Builds the monolith controller container
├── infra/                 
│   ├── docker-compose.yml   # Or other orchestration
│   ├── scripts/
│   └── ...
├── tests/                 
├── LICENSE
└── README.md