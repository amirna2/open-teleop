package zeromq

import (
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
