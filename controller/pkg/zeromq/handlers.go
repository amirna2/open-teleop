package zeromq

import (
	"encoding/base64"
	"encoding/json"
	"fmt"
	"log"
	"time"

	"github.com/open-teleop/controller/pkg/config"
)

// ConfigHandler handles CONFIG_REQUEST messages
type ConfigHandler struct {
	config *config.Config
	logger *log.Logger
}

// NewConfigHandler creates a new handler for configuration requests
func NewConfigHandler(cfg *config.Config, logger *log.Logger) *ConfigHandler {
	return &ConfigHandler{
		config: cfg,
		logger: logger,
	}
}

// HandleMessage processes a CONFIG_REQUEST message and returns a CONFIG_RESPONSE
func (h *ConfigHandler) HandleMessage(data []byte) ([]byte, error) {
	// Parse the message to ensure it's valid
	var msg ZeroMQMessage
	if err := json.Unmarshal(data, &msg); err != nil {
		return nil, fmt.Errorf("failed to parse message: %w", err)
	}

	// Verify it's a CONFIG_REQUEST
	if msg.Type != MsgTypeConfigRequest {
		return nil, fmt.Errorf("unexpected message type: %s", msg.Type)
	}

	h.logger.Printf("Processing configuration request")

	// Create response with the current configuration
	response := ZeroMQMessage{
		Type:      MsgTypeConfigResponse,
		Timestamp: float64(time.Now().Unix()),
		Data:      h.config,
	}

	// Serialize the response
	responseData, err := json.Marshal(response)
	if err != nil {
		h.logger.Printf("Error serializing response: %v", err)
		return nil, fmt.Errorf("failed to serialize response: %w", err)
	}

	h.logger.Printf("Sending configuration response (%d bytes)", len(responseData))
	return responseData, nil
}

// +++ START NEW HANDLER +++
// FlatbufferTopicData defines the expected structure within the Data field for flatbuffer messages
type FlatbufferTopicData struct {
	OttTopic   string `json:"ott_topic"`
	Base64Data string `json:"base64_data"`
}

// FlatbufferMessageHandler handles FLATBUFFER_TOPIC_MESSAGE messages
type FlatbufferMessageHandler struct {
	logger *log.Logger
	// Add any dependencies needed to process the flatbuffer, e.g., a router or processor
}

// NewFlatbufferMessageHandler creates a new handler for flatbuffer messages
func NewFlatbufferMessageHandler(logger *log.Logger) *FlatbufferMessageHandler {
	return &FlatbufferMessageHandler{
		logger: logger,
	}
}

// HandleMessage processes a FLATBUFFER_TOPIC_MESSAGE
func (h *FlatbufferMessageHandler) HandleMessage(data []byte) ([]byte, error) {
	var msg ZeroMQMessage
	if err := json.Unmarshal(data, &msg); err != nil {
		return nil, fmt.Errorf("failed to parse flatbuffer wrapper message: %w", err)
	}

	if msg.Type != "FLATBUFFER_TOPIC_MESSAGE" {
		return nil, fmt.Errorf("unexpected message type for FlatbufferMessageHandler: %s", msg.Type)
	}

	// Use type assertion to get the data field as map[string]interface{}
	dataMap, ok := msg.Data.(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("invalid data structure for flatbuffer message")
	}

	// Extract fields (handle potential type issues)
	ottTopic, topicOk := dataMap["ott_topic"].(string)
	base64Data, dataOk := dataMap["base64_data"].(string)
	if !topicOk || !dataOk {
		return nil, fmt.Errorf("missing or invalid ott_topic or base64_data in flatbuffer message data")
	}

	h.logger.Printf("Processing Flatbuffer message for topic: %s (%d base64 chars)", ottTopic, len(base64Data))

	// Decode base64 data
	flatbufferBytes, err := base64.StdEncoding.DecodeString(base64Data)
	if err != nil {
		return nil, fmt.Errorf("failed to decode base64 data for topic %s: %w", ottTopic, err)
	}

	// TODO: Add actual processing logic for the flatbufferBytes based on ottTopic
	h.logger.Printf("Successfully decoded Flatbuffer for topic %s (%d bytes) - PROCESSING NOT IMPLEMENTED", ottTopic, len(flatbufferBytes))

	// Send back a simple ACK response
	ackResponse := ZeroMQMessage{
		Type:      "ACK",
		Timestamp: float64(time.Now().Unix()),
		Data: map[string]interface{}{ // Use map for generic ACK data
			"status":  "OK",
			"topic":   ottTopic,
			"message": "Flatbuffer received",
		},
	}

	responseData, err := json.Marshal(ackResponse)
	if err != nil {
		// Log error but still attempt to inform client something went wrong with ack
		h.logger.Printf("Error serializing ACK response for %s: %v", ottTopic, err)
		return nil, fmt.Errorf("failed to serialize ACK response: %w", err) // Propagate error
	}

	return responseData, nil
}

// +++ END NEW HANDLER +++
