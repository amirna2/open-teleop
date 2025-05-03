# open_teleop_msgs/msg/EncodedFrame.msg

std_msgs/Header header       # Standard ROS header (timestamp from original image, frame_id)

string ott_topic            # Target Open Teleop Topic (e.g., "teleop.video.primary")

string encoding_format      # MIME type style, e.g., "video/h264", "video/vp8", "audio/opus", "image/jpeg"

# Frame Type Constants
uint8 FRAME_TYPE_UNKNOWN = 0
uint8 FRAME_TYPE_KEY = 1      # Key frame (I-frame for video)
uint8 FRAME_TYPE_DELTA = 2    # Delta frame (P/B-frame for video)
# Add others as needed (e.g., for audio, config frames)

uint8 frame_type

# Optional fields for display hints
uint32 width                # Frame width in pixels (if applicable)
uint32 height               # Frame height in pixels (if applicable)

# Encoded data payload
uint8[] data 