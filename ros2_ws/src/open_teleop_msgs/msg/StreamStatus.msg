# open_teleop_msgs/msg/StreamStatus.msg
# Holds status details for a single stream managed by the AV Node

string stream_id         # Unique ID assigned to the stream
string input_ros_topic   # The source ROS topic being processed
string output_ott_topic  # The target Open Teleop Topic
string encoding_format   # Configured encoding format (e.g., "video/h264")

bool is_enabled         # Is the stream currently configured to be active/publishing?
string status_message    # High-level status (e.g., "Running", "Paused", "Error: Input topic not found", "Initializing")

# Optional runtime metrics
float64 frame_rate_actual # Current processing/output frame rate (Hz)
uint64 bitrate_actual    # Current output bitrate (bits per second) 