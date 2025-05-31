# Media Server Design Document

## Overview

This document captures the design decisions and architecture for the Open-Teleop Media Server, a dedicated component for high-performance audio/video capture and streaming in teleoperation systems.

## Background & Problem Statement

### ROS2 Image Transport Limitations

Through performance analysis of the existing A/V Node, we discovered significant limitations with ROS2 image transport:

#### Performance Analysis Results
- **480p (640x480)**: 0.92MB frames, ~20 FPS actual (vs 30 FPS target)
- **720p (1280x720)**: 2.76MB frames, ~10 FPS with intermittent frame drops
- **Processing breakdown** (480p):
  - Camera capture: 49.5ms (96.5% of total time)
  - BGR→RGB conversion: 0.23ms
  - ROS message creation: 0.67ms
  - Publishing: 0.92ms

#### Key Findings
1. **Camera hardware** is the primary bottleneck for frame rate
2. **ROS2 transport** struggles with large frame sizes (>1MB)
3. **Frame drops** occur at 720p due to serialization/transport overhead
4. **Processing pipeline** is highly efficient (~2ms total overhead)

### Industry Reality
- Most robotics companies avoid ROS2 for real-time video streaming
- Production teleoperation systems use dedicated streaming solutions (GStreamer, WebRTC, RTSP)
- ROS2 A/V topics are typically used for low-bandwidth status/diagnostic imagery

## Architecture Decision

### Component Separation

We decided to maintain **two distinct components** rather than combining them:

#### A/V Node (Existing)
- **Purpose**: ROS2 A/V topics ↔ Teleop application bridge
- **Use Cases**: 
  - Development and testing
  - ROS2 ecosystem compatibility
  - Low-bandwidth status/diagnostic video
  - Compressed image transport
- **Limitations**: Frame drops at higher resolutions, not suitable for production teleop

#### Media Server (New)
- **Purpose**: High-performance video/audio capture and streaming
- **Use Cases**:
  - Production teleoperation
  - Low-latency navigation video
  - High-quality audio streaming
- **Capabilities**: Direct hardware access, multiple transport backends

### Rationale for Separation

1. **Separation of Concerns**
   - A/V Node: ROS2 ↔ ZeroMQ translation
   - Media Server: Hardware ↔ Multiple transports

2. **Performance Isolation**
   - Real-time media capture isolated from ROS message processing
   - Independent failure domains

3. **Deployment Flexibility**
   - ROS2 robots: Both components
   - Non-ROS robots: Media Server only
   - Multiple cameras: Multiple Media Server instances

4. **Technology Stack Optimization**
   - Media Server: GStreamer, V4L2, ALSA (C/C++)
   - A/V Node: ROS2, message serialization

## Media Server Architecture

### Core Components

```
Media Server
├── Device Discovery (V4L2, ALSA enumeration)
├── Pipeline Manager (Dynamic GStreamer pipelines)
├── Stream Encoder (H.264, Opus, etc.)
├── Transport Layer (ZeroMQ, WebRTC, etc.)
└── Configuration Manager (YAML config handling)
```

### Technology Stack

- **Core Framework**: GStreamer
- **Video Capture**: V4L2, USB cameras, RTSP sources
- **Audio Capture**: ALSA, PulseAudio
- **Encoding**: H.264 (hardware accelerated), MJPEG, Opus
- **Transport**: ZeroMQ, WebRTC, direct streaming
- **Configuration**: YAML-based

### Device Discovery & Capabilities

#### Discovery Message (OUTBOUND)
The Media Server periodically sends available device information:

```yaml
topic_mappings:
  - ott: "teleop.media.available_sources"
    topic_id: "f0000001-aaaa-bbbb-cccc-d701748f0010"
    direction: "OUTBOUND"
    priority: "LOW"
```

#### Message Format
```json
{
  "timestamp": "2025-01-27T10:30:00Z",
  "video_sources": [
    {
      "device_id": "video0",
      "device_path": "/dev/video0", 
      "name": "Logitech C920",
      "resolutions": [
        {"width": 1920, "height": 1080, "max_fps": 30},
        {"width": 1280, "height": 720, "max_fps": 30},
        {"width": 640, "height": 480, "max_fps": 30}
      ]
    }
  ],
  "audio_sources": [
    {
      "device_id": "audio0",
      "device_path": "hw:0,0",
      "name": "Logitech C920 Microphone"
    }
  ]
}
```

## Configuration Design

### Media Mappings (New Section)

Extending the existing teleop configuration with a new `media_mappings` section:

```yaml
# Existing ROS2 mappings remain unchanged
topic_mappings:
  - ros_topic: "/camera/image_raw"
    ott: "teleop.video.status_camera"
    # ... existing ROS2 config

# New media server mappings
media_mappings:
  video:
    - device_id: "video0"  # References discovered device
      ott: "teleop.video.navigation_camera"
      topic_id: "f0000001-aaaa-bbbb-cccc-d701748f0007"
      resolution: "1280x720"  # Selected from available resolutions
      framerate: 30
      encoding: "h264"
      bitrate: 2000
      priority: "HIGH"
    
    - device_id: "video2"
      ott: "teleop.video.rear_camera"
      topic_id: "f0000001-aaaa-bbbb-cccc-d701748f0009"
      resolution: "640x480"
      framerate: 30
      encoding: "mjpeg"
      quality: 80
      priority: "STANDARD"

  audio:
    - device_id: "audio0"
      ott: "teleop.audio.main_microphone"
      topic_id: "f0000001-aaaa-bbbb-cccc-d701748f0008"
      encoding: "opus"
      bitrate: 64
      priority: "STANDARD"
```

### User Experience Flow

1. **Discovery**: Media Server automatically detects available devices
2. **Selection**: User selects devices from discovered list
3. **Configuration**: User chooses resolution/framerate from device capabilities
4. **Validation**: System ensures selected parameters are supported
5. **Deployment**: Media Server creates optimized GStreamer pipelines

## GStreamer Implementation

### Pipeline Examples

#### Video Capture Pipeline
```bash
# V4L2 → H.264 → ZeroMQ
gst-launch-1.0 \
  v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  videoconvert ! \
  x264enc bitrate=2000 tune=zerolatency ! \
  h264parse ! \
  appsink name=videosink
```

#### Audio Capture Pipeline
```bash
# ALSA → Opus → ZeroMQ
gst-launch-1.0 \
  alsasrc device=hw:0,0 ! \
  audio/x-raw,rate=48000,channels=1 ! \
  opusenc bitrate=64000 ! \
  appsink name=audiosink
```

### Benefits of GStreamer

1. **Hardware Acceleration**: Automatic GPU encoding when available
2. **Format Support**: All major codecs and containers
3. **Device Abstraction**: V4L2, ALSA, PulseAudio built-in
4. **Pipeline Flexibility**: Runtime parameter modification
5. **Production Ready**: Used in commercial streaming systems

## Integration with Open-Teleop

### Architecture Position

```
ROBOT:
├── ROS Gateway (ROS2 messages)
├── A/V Node (ROS2 A/V topics) 
└── Media Server (direct hardware streaming) ← NEW
```

### Message Flow

```
Hardware → Media Server → Multiple Outputs:
                       ├── ZeroMQ → Controller (high priority)
                       ├── WebRTC → Web interfaces
                       ├── GStreamer → Custom endpoints
                       └── ROS2 topics → A/V Node (compatibility)
```

### User Choice

Users can choose their approach based on requirements:

```yaml
# Option 1: ROS2 only (existing, development/testing)
video_streams:
  - source: "ros2:/camera/image_raw"
    
# Option 2: High-performance (new, production)  
video_streams:
  - source: "media-server:navigation_camera"
    
# Option 3: Hybrid (both)
video_streams:
  - source: "ros2:/status/thumbnail"        # Low-res status
  - source: "media-server:main_camera"      # High-res navigation
```

## Implementation Phases

### Phase 1: Core Media Server
- Device discovery and enumeration
- Basic GStreamer pipeline creation
- ZeroMQ transport integration
- YAML configuration parsing

### Phase 2: Advanced Features
- Multiple transport backends (WebRTC)
- Hardware acceleration support
- Dynamic pipeline reconfiguration
- Performance monitoring

### Phase 3: Integration & Polish
- Web UI integration
- Configuration validation
- Error handling and recovery
- Documentation and examples

## Design Principles

1. **Optional Component**: Users can still use A/V Node for ROS2-only setups
2. **Backward Compatibility**: Existing A/V Node functionality unchanged
3. **Performance First**: Optimized for low-latency, high-quality streaming
4. **Hardware Agnostic**: Support for various camera and audio devices
5. **Transport Flexible**: Multiple output formats and protocols
6. **Configuration Driven**: YAML-based setup with capability validation

## Future Considerations

- **Multi-camera Support**: Multiple Media Server instances
- **Cloud Integration**: Direct streaming to cloud endpoints
- **AI Integration**: Real-time video analysis pipelines
- **Compression Options**: Advanced codecs (AV1, HEVC)
- **Network Adaptation**: Adaptive bitrate streaming

## Conclusion

The Media Server represents a significant enhancement to Open-Teleop's capabilities, addressing the fundamental limitations of ROS2 image transport while maintaining compatibility with existing ROS2-based workflows. By leveraging GStreamer's proven capabilities and providing a clean configuration interface, we enable production-quality teleoperation video streaming while keeping the system accessible to developers and researchers using ROS2. 