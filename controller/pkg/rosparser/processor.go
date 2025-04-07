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

	// Parse the message using the ROS parser
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
