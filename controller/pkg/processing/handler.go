package processing

import (
	"encoding/json"

	customlog "github.com/open-teleop/controller/pkg/log"
)

// LoggingResultHandler logs processing results and could deliver them to clients
type LoggingResultHandler struct {
	logger customlog.Logger
}

// NewLoggingResultHandler creates a new logging result handler
func NewLoggingResultHandler(logger customlog.Logger) *LoggingResultHandler {
	return &LoggingResultHandler{
		logger: logger,
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
	}

	// TODO: In the future, this could deliver the processed data to clients
	// For now, we just log it
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
