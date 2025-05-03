# Open Teleop - Video Streaming Implementation Plan

This document outlines the agreed-upon design and implementation plan for handling real-time video streaming within the Open Teleop platform.

## 1. Requirements

*   **Input:** Standard ROS 2 image topics (`sensor_msgs/msg/Image`, `sensor_msgs/msg/CompressedImage`).
*   **Processing:** Capture ROS image topics, encode into video format (H.264, VP8/VP9), prepare for network transmission.
*   **Transport:** Transmit encoded video frames efficiently from ROS Gateway to Controller.
*   **Output:** Controller routes video streams to the operator interface (Web App) via WebRTC (primary) or WebSocket (fallback).
*   **Configuration:** Video sources (ROS topics), target Open Teleop Topics (`ott`), priority, and encoding parameters configurable via YAML.
*   **Performance:** Real-time, low-latency transmission.
*   **Flexibility:** Pluggable encoding backend (GStreamer initially).
*   **Audio:** Plan for future audio handling using a similar pipeline.

## 2. Design Decision

*   **Chosen Approach:** Dedicated ROS 2 Node (`open_teleop_av_node`), functioning as a **dynamically configurable service** for media encoding.
*   **Rationale:**
    *   **Modularity:** Clear separation of concerns (media processing vs. gateway logic).
    *   **Maintainability:** Easier updates/replacement of the media backend.
    *   **Dependency Management:** Isolates media library dependencies (e.g., GStreamer).
    *   **Extensibility:** Facilitates community contributions or custom backends.
*   **Alternatives Considered:**
    *   Integrating A/V processing into ROS Gateway (rejected: monolithic, dependency bloat).
    *   Direct WebRTC from Robot/Gateway (rejected: complexity, firewall issues).

## 3. Interface Definition

### 3.1. AV Node <-> ROS Gateway

*   **Mechanism:**
    *   **Data:** Standard ROS 2 Publish/Subscribe for encoded frames.
    *   **Control/Configuration:** ROS 2 Services.
*   **Data Message:** Custom ROS 2 Message `open_teleop_msgs/msg/EncodedFrame` (remains the same as previously defined).
    ```proto
    # open_teleop_msgs/msg/EncodedFrame.msg

    std_msgs/Header header       # Standard ROS header (timestamp from original image, frame_id)
    string ott_topic            # Target Open Teleop Topic (e.g., "teleop.video.primary")
    string encoding_format      # MIME type style, e.g., "video/h264", "video/vp8", "audio/opus", "image/jpeg"
    # Frame Type Constants
    uint8 FRAME_TYPE_UNKNOWN = 0
    uint8 FRAME_TYPE_KEY = 1      # Key frame (I-frame for video)
    uint8 FRAME_TYPE_DELTA = 2    # Delta frame (P/B-frame for video)
    uint8 frame_type
    # Optional fields for display hints
    uint32 width                # Frame width in pixels (if applicable)
    uint32 height               # Frame height in pixels (if applicable)
    # Encoded data payload
    uint8[] data
    ```
*   **Topic Naming (Data):** AV node publishes encoded data to `~/encoded_frames`. Gateway subscribes.
*   **Control Services (Defined in `open_teleop_msgs`):**
    *   **`ManageStream.srv`**: Add, remove, update, enable, or disable a specific encoding stream.
        ```proto
        # open_teleop_msgs/srv/ManageStream.srv

        # --- Request ---
        # Action definitions
        uint8 ACTION_ADD = 0
        uint8 ACTION_REMOVE = 1
        uint8 ACTION_UPDATE = 2
        uint8 ACTION_ENABLE = 3  # Start/Resume publishing
        uint8 ACTION_DISABLE = 4 # Stop/Pause publishing

        uint8 action

        string stream_id          # Unique ID. Required for non-ADD actions.

        # Fields relevant for ADD/UPDATE actions
        string input_ros_topic    # e.g., "/robot/camera/image_raw"
        string output_ott_topic   # e.g., "teleop.video.primary"
        string encoding_format    # e.g., "video/h264"
        string encoder_params     # YAML/JSON string for params (bitrate, quality, etc.)

        # --- Response ---
        bool success
        string message            # Status or error description
        string assigned_stream_id # ID assigned by the node (useful if empty on ADD)
        ```
    *   **`GetStatus.srv`**: Query the current state of the node and its streams.
        ```proto
        # open_teleop_msgs/srv/GetStatus.srv

        # --- Request ---
        # (Empty)

        # --- Response ---
        # Depends on StreamStatus message defined below
        open_teleop_msgs/StreamStatus[] active_streams
        string node_status            # Overall status ("OK", "DEGRADED", "ERROR")
        ```
*   **Supporting Message (Defined in `open_teleop_msgs`):**
    *   **`StreamStatus.msg`**: Holds status details for a single stream.
        ```proto
        # open_teleop_msgs/msg/StreamStatus.msg

        string stream_id
        string input_ros_topic
        string output_ott_topic
        string encoding_format
        bool is_enabled          # Is the stream actively publishing?
        string status_message    # e.g., "Running", "Paused", "Error", "Initializing"
        # Optional runtime metrics
        float64 frame_rate_actual
        uint64 bitrate_actual
        ```

### 3.2. ROS Gateway <-> Controller

*   **Mechanism:** ZeroMQ (existing infrastructure).
*   **Message:** Flatbuffers schema (`OpenTeleopEncodedMedia.fbs` - to be defined). The Gateway parses the `EncodedFrame` ROS message and serializes its content into this Flatbuffer schema before sending over ZMQ. **This interface remains unchanged by the addition of control services.**

## 4. High-Level Implementation Plan

1.  **`open_teleop_msgs` Package:**
    *   Create ROS 2 package.
    *   Define `EncodedFrame.msg`.
    *   Define `ManageStream.srv`.
    *   Define `GetStatus.srv`.
    *   Define `StreamStatus.msg`.
2.  **Flatbuffer Schema:**
    *   Define `OpenTeleopEncodedMedia.fbs` (fields matching essential `EncodedFrame.msg` content plus any transport-level needs like timestamp synchronization).
3.  **`open_teleop_av_node` (ROS 2 Node - Python/C++):**
    *   Create package structure.
    *   Implement ROS 2 **Service Servers** for `ManageStream` and `GetStatus`. This includes logic to manage internal state of multiple stream pipelines (adding, removing, updating, enabling/disabling GStreamer pipelines dynamically).
    *   Implement ROS 2 **Publisher** for `EncodedFrame` messages for all enabled streams.
    *   Implement GStreamer pipeline creation/management logic based on service calls (`appsrc ! videoconvert ! <encoder> ! appsink`). **Pipelines will be constructed programmatically using the GStreamer API (e.g., `gst_element_factory_make`, `gst_element_link`) to allow for dynamic parameter adjustments.**
    *   (Initial configuration can still be read from parameters at startup to pre-configure default streams via internal calls to the ManageStream logic).
4.  **ROS Gateway Updates (Go/Python):**
    *   Add ROS 2 **Service Client** logic for `ManageStream` and `GetStatus`.
    *   On startup, read video configuration and call `ManageStream` service on the `open_teleop_av_node` to set up initial streams.
    *   Implement logic to handle potential dynamic reconfiguration requests (likely originating from the Controller) by calling `ManageStream`.
    *   Add **Subscription** logic for `EncodedFrame` topics.
    *   Parse incoming `EncodedFrame` messages.
    *   Serialize data into `OpenTeleopEncodedMedia` Flatbuffer schema.
    *   Send Flatbuffer binary over ZeroMQ to the Controller.
5.  **Controller Updates (Go/Python):**
    *   Update ZeroMQ receiver/director to handle `OpenTeleopEncodedMedia` Flatbuffers.
    *   Route frames based on `ott_topic`.
    *   Update/Implement WebRTC interface to handle video tracks and forward received frames.
    *   Update/Implement WebSocket interface (fallback) if needed.
6.  **Web Application Updates (React/JS):**
    *   Implement WebRTC logic for video tracks.
    *   Render received video streams (using browser decoders or MediaSource Extensions).
7.  **Configuration Updates:**
    *   Define YAML schema for video stream configuration.
    *   Update config loading logic in relevant components.
8.  **Testing:** Implement unit and integration tests for the new components and interactions.
9.  **Documentation:** Update overall architecture diagrams and component documentation.

## 5. Open Questions / Future Considerations

*   Detailed GStreamer pipeline tuning for latency/quality trade-offs.
*   Specific QoS settings for ROS 2 topics (`EncodedFrame`).
*   Error handling (encoder failures, network issues).
*   Audio integration plan.
*   Benchmarking and performance optimization.
*   WebRTC TURN/STUN server configuration and deployment. 