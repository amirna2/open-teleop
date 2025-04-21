package processing

import (
	"encoding/json"

	customlog "github.com/open-teleop/controller/pkg/log"
)

// MessagePublisher defines the interface for publishing messages
type MessagePublisher interface {
	PublishMessage(topic string, data []byte) error
}

// LoggingResultHandler logs processing results and publishes them to ZMQ
type LoggingResultHandler struct {
	logger    customlog.Logger
	publisher MessagePublisher
}

// NewLoggingResultHandler creates a new logging result handler
func NewLoggingResultHandler(logger customlog.Logger, publisher MessagePublisher) *LoggingResultHandler {
	return &LoggingResultHandler{
		logger:    logger,
		publisher: publisher,
	}
}

// HandleResult handles a processed message result
func (h *LoggingResultHandler) HandleResult(result *ProcessResult) {
	if result.Error != nil {
		h.logger.Errorf("Error processing message for topic '%s': %v", result.Topic, result.Error)
		return
	}

	// Log basic information about the processed message
	h.logger.Debugf("Successfully processed message for topic '%s' (timestamp: %d)",
		result.Topic, result.Timestamp)

	// Log data summary if it's available
	if result.Data != nil {
		// Try to get a summary of the data
		jsonData, err := json.Marshal(result.Data)
		if err == nil {
			if len(jsonData) > 100 {
				h.logger.Debugf("Data: %s...", string(jsonData[:100]))
			} else {
				h.logger.Debugf("Data: %s", string(jsonData))
			}
		}

		// Publish the processed message
		if h.publisher != nil {
			if err := h.publisher.PublishMessage(result.Topic, jsonData); err != nil {
				h.logger.Errorf("Failed to publish message for topic '%s': %v", result.Topic, err)
			} else {
				h.logger.Debugf("Published message for topic '%s'", result.Topic)
			}
		}
	}
}

// CreateHandlerFunc creates a ResultHandler function for the ProcessingPool
func (h *LoggingResultHandler) CreateHandlerFunc() ResultHandler {
	return func(processResult *ProcessResult) {
		if processResult == nil {
			h.logger.Errorf("Received nil ProcessResult")
			return
		}
		h.HandleResult(processResult)
	}
}
