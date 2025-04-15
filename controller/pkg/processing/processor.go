package processing

import (
	"fmt"

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

	// Special handling for image messages
	if messageType == "sensor_msgs/msg/Image" || messageType == "sensor_msgs/msg/CompressedImage" {
		return p.ProcessImageMessage(topic, messageType, ottMsg.TimestampNs(), payloadBytes)
	}

	// Parse the message using the ROS parser
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
