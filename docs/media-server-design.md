# Media Gateway Design Document

## Overview

This document captures the design decisions and architecture for the Open-Teleop Media Gateway, a dedicated component for high-performance audio/video capture and streaming in teleoperation systems.

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
1. **ROS2 transport overhead** is the primary bottleneck for frame rate
2. **Camera hardware** performs adequately when accessed directly
3. **Frame drops** occur at 720p due to ROS2 serialization/transport overhead
4. **Downstream processing pipeline** is highly efficient (~2ms total overhead)
5. **Existing transport chain** (Flatbuffer → Controller → WebSocket → Web UI) works well

### Industry Reality
- Most robotics companies avoid ROS2 for real-time video streaming
- Production teleoperation systems use dedicated streaming solutions (GStreamer, WebRTC, RTSP)
- ROS2 A/V topics are typically used for low-bandwidth status/diagnostic imagery

## Architecture Decision

### Component Separation

We decided to maintain **two distinct components** rather than combining them:

#### A/V Node (Existing)
- **Purpose**: ROS2 A/V topics ↔ Open Teleop protocol bridge
- **Use Cases**: 
  - Development and testing
  - ROS2 ecosystem compatibility
  - Low-bandwidth status/diagnostic video
  - Compressed image transport
- **Limitations**: Frame drops at higher resolutions due to ROS2 overhead

#### Media Gateway (New)
- **Purpose**: Hardware devices ↔ Open Teleop protocol bridge
- **Use Cases**:
  - Production teleoperation
  - Low-latency navigation video
  - High-quality audio streaming
- **Capabilities**: Direct hardware access, multiple transport backends

### Rationale for Separation

1. **Separation of Concerns**
   - A/V Node: ROS2 ↔ Open Teleop translation
   - Media Gateway: Hardware ↔ Open Teleop translation

2. **Performance Isolation**
   - Real-time media capture isolated from ROS message processing
   - Independent failure domains

3. **Deployment Flexibility**
   - ROS2 robots: Both components
   - Non-ROS robots: Media Gateway only
   - Multiple cameras: Multiple Media Gateway instances

4. **Technology Stack Optimization**
   - Media Gateway: GStreamer, V4L2, ALSA (C/C++)
   - A/V Node: ROS2, message serialization

5. **Gateway Pattern Consistency**
   - ros-gateway: ROS2 ↔ Open Teleop protocol
   - media-gateway: Hardware ↔ Open Teleop protocol

## Media Gateway Architecture

### Core Components

```
Media Gateway
├── Device Discovery (V4L2, ALSA enumeration)
├── Pipeline Manager (Dynamic GStreamer pipelines)
├── Stream Encoder (H.264, Opus, etc.)
├── Transport Layer (ZeroMQ, same as A/V Node)
└── Configuration Manager (Same pattern as ros-gateway)
```

### Technology Stack Alignment

- **Core Framework**: GStreamer (same as A/V Node)
- **Video Capture**: V4L2, USB cameras, RTSP sources
- **Audio Capture**: ALSA, PulseAudio
- **Encoding**: H.264 (hardware accelerated), MJPEG, Opus (same formats as A/V Node)
- **Transport**: ZeroMQ with Flatbuffers (same protocol as A/V Node)
- **Configuration**: Same request/response pattern as ros-gateway

### Device Discovery & Capabilities

#### Discovery Message (OUTBOUND)
The Media Gateway periodically sends available device information:

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

Extending the existing teleop configuration with a new `media_mappings` section, parallel to `topic_mappings`:

```yaml
# Existing ROS2 mappings remain unchanged
topic_mappings:
  - ros_topic: "/camera/image_raw"
    ott: "teleop.video.status_camera"
    encoding_format: "video/h264"
    encoder_params:
      width: 640
      height: 480
      framerate: 30
      bitrate: 1000
      gop_size: 10

# New media gateway mappings (same structure, different source)
media_mappings:
  video:
    - device_id: "video0"  # References discovered device
      ott: "teleop.video.navigation_camera"
      topic_id: "f0000001-aaaa-bbbb-cccc-d701748f0007"
      encoding_format: "video/h264"
      encoder_params:
        width: 1920
        height: 1080
        framerate: 30
        bitrate: 4000
        gop_size: 15
      priority: "HIGH"
    
    - device_id: "video2"
      ott: "teleop.video.rear_camera"
      topic_id: "f0000001-aaaa-bbbb-cccc-d701748f0009"
      encoding_format: "video/mjpeg"
      encoder_params:
        width: 640
        height: 480
        framerate: 30
        quality: 80
      priority: "STANDARD"

  audio:
    - device_id: "audio0"
      ott: "teleop.audio.main_microphone"
      topic_id: "f0000001-aaaa-bbbb-cccc-d701748f0008"
      encoding_format: "audio/opus"
      encoder_params:
        bitrate: 64000
        sample_rate: 48000
        channels: 1
      priority: "STANDARD"
```

### Configuration Request Pattern

Media Gateway uses the same configuration pattern as ros-gateway:

```go
// Same config request pattern as other components
func (mg *MediaGateway) RequestConfig() (*Config, error) {
    // Same HTTP/gRPC call to Controller as ros-gateway
    // Parse media_mappings section instead of topic_mappings
}

// Same parsing approach, different section
func (mg *MediaGateway) ParseMediaConfig(config *Config) error {
    // Parse media_mappings section
    // Create GStreamer pipelines based on device_id
    // Setup ZeroMQ publishers with same protocol as A/V Node
}
```

### User Experience Flow

1. **Discovery**: Media Gateway automatically detects available devices
2. **Configuration**: User configures devices via same YAML as other components
3. **Validation**: System ensures selected parameters match device capabilities
4. **Deployment**: Media Gateway creates optimized GStreamer pipelines

## GStreamer Implementation

### Pipeline Examples

#### Video Capture Pipeline
```bash
# V4L2 → H.264 → ZeroMQ (same output format as A/V Node)
gst-launch-1.0 \
  v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  videoconvert ! \
  x264enc bitrate=2000 tune=zerolatency speed-preset=ultrafast profile=baseline ! \
  h264parse ! \
  appsink name=videosink
```

#### Audio Capture Pipeline
```bash
# ALSA → Opus → ZeroMQ (same output format as A/V Node)
gst-launch-1.0 \
  alsasrc device=hw:0,0 ! \
  audio/x-raw,rate=48000,channels=1 ! \
  opusenc bitrate=64000 ! \
  appsink name=audiosink
```

### Encoder Parameter Alignment

Media Gateway uses the same encoder parameter format as A/V Node:

```go
// Same encoder parameter structure
type EncoderParams struct {
    Width       int    `yaml:"width"`
    Height      int    `yaml:"height"`
    Framerate   int    `yaml:"framerate"`
    Bitrate     int    `yaml:"bitrate"`     // kbps for video, bps for audio
    GopSize     int    `yaml:"gop_size"`    // For H.264
    Quality     int    `yaml:"quality"`     // For MJPEG (0-100)
    SampleRate  int    `yaml:"sample_rate"` // For audio (Hz)
    Channels    int    `yaml:"channels"`    // For audio
    Profile     string `yaml:"profile"`     // "baseline", "main", "high"
    Preset      string `yaml:"preset"`      // "ultrafast", "fast", "medium"
}
```

## Integration with Open-Teleop

### Gateway Pattern Architecture

```
ROBOT:
├── ros-gateway (ROS2 ↔ Open Teleop protocol)
└── media-gateway (Hardware ↔ Open Teleop protocol) ← NEW
```

### Message Flow Alignment

Both gateways output to the same transport protocol:

```
Hardware → Media Gateway ↘
                          ↘ ZeroMQ (Flatbuffers) → Controller → Web UI
ROS2 Topics → A/V Node → ros-gateway ↗
```

### Transport Protocol Compatibility

Media Gateway outputs the same message format as A/V Node:

```go
// Same EncodedFrame format as A/V Node
type EncodedFrame struct {
    Header         Header  // Same timestamp format
    OttTopic       string  // Same topic naming convention
    EncodingFormat string  // Same MIME types ("video/h264", "audio/opus")
    FrameType      uint8   // Same frame type constants
    Width          uint32  // Same dimensions format
    Height         uint32
    Data           []byte  // Same H.264/Opus format
}

// Same Flatbuffer serialization as A/V Node
// Same ZeroMQ transport as A/V Node
// Controller handles identically to A/V Node frames
```

### User Choice

Users can choose their approach based on requirements:

```yaml
# Option 1: ROS2 only (existing, development/testing)
video_streams:
  - source: "ros2:/camera/image_raw"
    
# Option 2: High-performance (new, production)  
video_streams:
  - source: "hardware:navigation_camera"
    
# Option 3: Hybrid (both)
video_streams:
  - source: "ros2:/status/thumbnail"        # Low-res status via A/V Node
  - source: "hardware:main_camera"          # High-res navigation via Media Gateway
```

## Implementation Phases

### Phase 1: Core Media Gateway (Weeks 1-3)
- Device discovery and enumeration (V4L2, ALSA)
- Basic GStreamer pipeline creation
- ZeroMQ transport integration (same protocol as A/V Node)
- Configuration request/response (same pattern as ros-gateway)

### Phase 2: Integration Testing (Weeks 4-5)
- End-to-end validation: Hardware → Media Gateway → Controller → Web UI
- Performance comparison: A/V Node vs Media Gateway
- Multiple device support
- Error handling and recovery

### Phase 3: Production Readiness (Week 6)
- Configuration validation against device capabilities
- Documentation and deployment guides
- Performance optimization
- Integration with existing deployment scripts

## Performance Validation

### Test Matrix
```
Baseline (A/V Node):
├── 480p @ 20 FPS: Current performance
└── 720p @ 10 FPS: With frame drops

Target (Media Gateway):
├── 480p @ 30 FPS: Validation test
├── 720p @ 30 FPS: Improvement test
├── 1080p @ 30 FPS: Production target
└── Multiple streams: Scaling test
```

### Success Criteria
- **1080p @ 30 FPS**: Consistent delivery without frame drops
- **Sub-100ms latency**: Hardware to Web UI display
- **CPU efficiency**: Lower CPU usage than ROS2 approach
- **Stability**: 24-hour continuous operation

## Design Principles

1. **Gateway Pattern Consistency**: Same architectural pattern as ros-gateway
2. **Transport Protocol Reuse**: Same ZeroMQ + Flatbuffers as A/V Node
3. **Configuration Alignment**: Same request/response pattern as other components
4. **Backward Compatibility**: Existing A/V Node functionality unchanged
5. **Performance First**: Optimized for low-latency, high-quality streaming
6. **Hardware Agnostic**: Support for various camera and audio devices
7. **Deployment Flexible**: Can run alongside or instead of A/V Node

## Future Considerations

- **Multi-camera Support**: Multiple Media Gateway instances
- **RTSP Input Sources**: Network camera integration
- **Hardware Acceleration**: GPU encoding support
- **Advanced Codecs**: AV1, HEVC support
- **WebRTC Direct**: Media Gateway → WebRTC (bypass Controller)
- **Cloud Integration**: Direct streaming to cloud endpoints

## Conclusion

The Media Gateway addresses the fundamental ROS2 performance limitations while maintaining full compatibility with the existing Open Teleop architecture. By following the established gateway pattern and reusing proven transport protocols, it provides a seamless upgrade path for production teleoperation systems. The parallel deployment capability ensures that both ROS2-based development workflows and high-performance production deployments are supported within the same architecture. 