package rosparser

import (
	"encoding/json"
	"fmt"
	"sync"

	message "github.com/open-teleop/controller/pkg/flatbuffers/open_teleop/message"
	customlog "github.com/open-teleop/controller/pkg/log"
)

// TopicProcessor handles processing of ROS messages for specific topics
type TopicProcessor struct {
	// Logger for this processor
	logger customlog.Logger

	// MessageTypeMapping maps OTT topics to ROS message types
	messageTypeMapping map[string]string

	// Mutex for thread-safety
	mu sync.RWMutex
}

// NewTopicProcessor creates a new topic processor
func NewTopicProcessor(logger customlog.Logger) *TopicProcessor {
	return &TopicProcessor{
		logger:             logger,
		messageTypeMapping: make(map[string]string),
	}
}

// RegisterMessageType registers a mapping from an OTT topic to a ROS message type
func (p *TopicProcessor) RegisterMessageType(ottTopic, messageType string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.messageTypeMapping[ottTopic] = messageType
	p.logger.Infof("Registered message type '%s' for topic '%s'", messageType, ottTopic)
}

// GetMessageType returns the ROS message type for an OTT topic
func (p *TopicProcessor) GetMessageType(ottTopic string) (string, bool) {
	p.mu.RLock()
	defer p.mu.RUnlock()
	msgType, exists := p.messageTypeMapping[ottTopic]
	return msgType, exists
}

// ProcessRosMessage processes a ROS message from a FlatBuffer
func (p *TopicProcessor) ProcessRosMessage(ottMsg *message.OttMessage) (map[string]interface{}, error) {
	// Get the OTT topic
	ottTopic := string(ottMsg.Ott())

	// Get the message type
	messageType, exists := p.GetMessageType(ottTopic)
	if !exists {
		return nil, fmt.Errorf("unknown message type for topic '%s'", ottTopic)
	}

	// Get the payload bytes
	payloadBytes := ottMsg.PayloadBytes()
	if len(payloadBytes) == 0 {
		return nil, fmt.Errorf("empty payload for topic '%s'", ottTopic)
	}

	// Special handling for image messages
	if messageType == "sensor_msgs/msg/Image" || messageType == "sensor_msgs/msg/CompressedImage" {
		return p.ProcessImageMessage(ottTopic, messageType, ottMsg.TimestampNs(), payloadBytes)
	}

	// Regular message processing for non-image types
	p.logger.Debugf("Processing ROS message for topic '%s' (type: %s, %d bytes)",
		ottTopic, messageType, len(payloadBytes))

	parsedMsg, err := ParseToJSON(messageType, payloadBytes)
	if err != nil {
		return nil, fmt.Errorf("failed to parse message for topic '%s': %w", ottTopic, err)
	}

	// Add metadata
	result := map[string]interface{}{
		"topic":     ottTopic,
		"type":      messageType,
		"timestamp": ottMsg.TimestampNs(),
		"data":      parsedMsg,
	}

	return result, nil
}

// ProcessImageMessage processes specifically image messages, extracting their metadata
// separately from the raw image data for more efficient transport
func (p *TopicProcessor) ProcessImageMessage(
	ottTopic string,
	messageType string,
	timestamp int64,
	payloadBytes []byte,
) (map[string]interface{}, error) {
	p.logger.Debugf("Processing image message for topic '%s' (type: %s, %d bytes)",
		ottTopic, messageType, len(payloadBytes))

	// Get the image data and metadata using the specialized function
	imageData, err := ExtractImageData(messageType, payloadBytes)
	if err != nil {
		return nil, fmt.Errorf("failed to extract image data for topic '%s': %w", ottTopic, err)
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
		"topic":     ottTopic,
		"type":      messageType,
		"timestamp": timestamp,
		"metadata":  imageData.Metadata,
		"is_image":  true,
		"data_size": len(imageData.RawData),
	}

	// In a real implementation, we might do something like:
	// 1. If using WebRTC: Send the raw image data as a video frame
	//    webrtcManager.SendFrame(imageData.RawData, ottTopic, timestamp)
	// 2. If raw web socket is available: Send the binary data separately
	//    websocketManager.SendBinary(imageData.RawData, ottTopic, timestamp)

	return result, nil
}

// CreateZeroMQProcessor creates a function that processes OttMessages for use with ZeroMQ
// It returns a function compatible with the ZeroMQService's RegisterTopicProcessor method
func (p *TopicProcessor) CreateZeroMQProcessor(ottTopic string) (func(*message.OttMessage, []byte) ([]byte, error), error) {
	// Verify that we have a message type mapping for this topic
	messageType, exists := p.GetMessageType(ottTopic)
	if !exists {
		return nil, fmt.Errorf("no message type mapping for topic '%s'", ottTopic)
	}

	p.logger.Infof("Creating processor for topic '%s' with message type '%s'", ottTopic, messageType)

	// Create and return the processor function
	return func(ottMsg *message.OttMessage, payloadBytes []byte) ([]byte, error) {
		// Process the message
		result, err := p.ProcessRosMessage(ottMsg)
		if err != nil {
			return nil, fmt.Errorf("processing error: %w", err)
		}

		// Create a JSON response
		response := struct {
			Type      string                 `json:"type"`
			Timestamp int64                  `json:"timestamp"`
			Data      map[string]interface{} `json:"data"`
		}{
			Type:      "PROCESSED_MESSAGE",
			Timestamp: ottMsg.TimestampNs(),
			Data:      result,
		}

		// Serialize to JSON
		responseJSON, err := json.Marshal(response)
		if err != nil {
			return nil, fmt.Errorf("failed to serialize response: %w", err)
		}

		return responseJSON, nil
	}, nil
}
