package processing

import (
	"encoding/json"
	"fmt"

	"github.com/open-teleop/controller/domain/video"
	message "github.com/open-teleop/controller/pkg/flatbuffers/open_teleop/message"
	customlog "github.com/open-teleop/controller/pkg/log"
	"github.com/open-teleop/controller/pkg/rosparser"
)

// RosMessageProcessor processes ROS messages from flatbuffers
type RosMessageProcessor struct {
	logger        customlog.Logger
	topicRegistry *TopicRegistry
}

// NewRosMessageProcessor creates a new ROS message processor
func NewRosMessageProcessor(logger customlog.Logger, topicRegistry *TopicRegistry) *RosMessageProcessor {
	return &RosMessageProcessor{
		logger:        logger,
		topicRegistry: topicRegistry,
	}
}

// ProcessMessage processes an OttMessage and returns its JSON representation
func (p *RosMessageProcessor) ProcessMessage(ottMsg *message.OttMessage) (map[string]interface{}, error) {
	topic := string(ottMsg.Ott())

	// Get the message type from the topic registry
	messageType, exists := p.topicRegistry.GetMessageType(topic)
	if !exists {
		return nil, fmt.Errorf("unknown message type for topic '%s'", topic)
	}

	// Get the payload bytes
	payloadBytes := ottMsg.PayloadBytes()
	if len(payloadBytes) == 0 {
		return nil, fmt.Errorf("empty payload for topic '%s'", topic)
	}

	// Check if this is a JSON command (e.g., joystick/teleop commands)
	if ottMsg.ContentType() == message.ContentTypeJSON_COMMAND {
		// For JSON commands, parse the JSON directly without ROS parsing
		var jsonData map[string]interface{}
		if err := json.Unmarshal(payloadBytes, &jsonData); err != nil {
			return nil, fmt.Errorf("failed to parse JSON command for topic '%s': %w", topic, err)
		}

		// For Twist messages, ensure numeric values are properly formatted
		if messageType == "geometry_msgs/msg/Twist" {
			if linear, ok := jsonData["linear"].(map[string]interface{}); ok {
				for k, v := range linear {
					if num, ok := v.(float64); ok {
						linear[k] = float64(num)
					}
				}
			}
			if angular, ok := jsonData["angular"].(map[string]interface{}); ok {
				for k, v := range angular {
					if num, ok := v.(float64); ok {
						angular[k] = float64(num)
					}
				}
			}
		}

		// Return in the exact format expected by ROS Gateway
		return map[string]interface{}{
			"topic": topic,
			"data":  jsonData,
		}, nil
	}

	// Special handling for image messages
	if messageType == "sensor_msgs/msg/Image" || messageType == "sensor_msgs/msg/CompressedImage" {
		return p.ProcessImageMessage(topic, messageType, ottMsg.TimestampNs(), payloadBytes)
	}

	// For all other messages, use the ROS parser
	p.logger.Debugf("Processing ROS message for topic '%s' (type: %s, %d bytes)",
		topic, messageType, len(payloadBytes))

	parsedMsg, err := rosparser.ParseToJSON(messageType, payloadBytes)
	if err != nil {
		return nil, fmt.Errorf("failed to parse message for topic '%s': %w", topic, err)
	}

	// Add metadata
	result := map[string]interface{}{
		"topic":     topic,
		"type":      messageType,
		"timestamp": ottMsg.TimestampNs(),
		"data":      parsedMsg,
	}

	return result, nil
}

// ProcessImageMessage processes specifically image messages, extracting their metadata
// separately from the raw image data for more efficient transport
func (p *RosMessageProcessor) ProcessImageMessage(
	topic string,
	messageType string,
	timestamp int64,
	payloadBytes []byte,
) (map[string]interface{}, error) {
	p.logger.Debugf("Processing image message for topic '%s' (type: %s, %d bytes)",
		topic, messageType, len(payloadBytes))

	// Get the image data and metadata using the specialized function
	imageData, err := rosparser.ExtractImageData(messageType, payloadBytes)
	if err != nil {
		return nil, fmt.Errorf("failed to extract image data for topic '%s': %w", topic, err)
	}

	// Log some information about the image
	isCompressed := messageType == "sensor_msgs/msg/CompressedImage"
	if isCompressed {
		if format, ok := imageData.Metadata["format"].(string); ok {
			p.logger.Debugf("Image format: %s, size: %d bytes", format, len(imageData.RawData))
		}
	} else {
		width, _ := imageData.Metadata["width"].(float64)
		height, _ := imageData.Metadata["height"].(float64)
		encoding, _ := imageData.Metadata["encoding"].(string)
		p.logger.Debugf("Image dimensions: %dx%d, encoding: %s, size: %d bytes",
			int(width), int(height), encoding, len(imageData.RawData))
	}

	// Add metadata and topic info (but don't include the raw data here)
	result := map[string]interface{}{
		"topic":     topic,
		"type":      messageType,
		"timestamp": timestamp,
		"metadata":  imageData.Metadata,
		"is_image":  true,
		"data_size": len(imageData.RawData),
	}

	return result, nil
}

// CreateProcessorFunc creates a MessageProcessor function that can be used with the MessageDirector
func (p *RosMessageProcessor) CreateProcessorFunc() MessageProcessor {
	return func(ottMsg *message.OttMessage) (map[string]interface{}, error) {
		return p.ProcessMessage(ottMsg)
	}
}

func NewHighPriorityProcessor(videoService *video.VideoService, rosProcessor MessageProcessor) MessageProcessor {
	return func(ottMsg *message.OttMessage) (map[string]interface{}, error) {
		if ottMsg.ContentType() == message.ContentTypeENCODED_VIDEO_FRAME {
			topic := string(ottMsg.Ott())
			timestamp := ottMsg.TimestampNs()
			payload := ottMsg.PayloadBytes()
			videoService.BroadcastVideoFrame(topic, timestamp, payload)
			return map[string]interface{}{"status": "video_frame_broadcast"}, nil
		}
		// Else, process as ROS2 message
		return rosProcessor(ottMsg)
	}
}
