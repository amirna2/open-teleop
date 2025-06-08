package zeromq

import (
	message "github.com/open-teleop/controller/pkg/flatbuffers/open_teleop/message"
	customlog "github.com/open-teleop/controller/pkg/log"
	"github.com/open-teleop/controller/pkg/processing"
)

// DirectorWrapper provides a wrapper for the MessageDirector
// that makes type assertions safer when called from the ZeroMQ service
type DirectorWrapper struct {
	director *processing.MessageDirector
	logger   customlog.Logger
}

// NewDirectorWrapper creates a new DirectorWrapper
func NewDirectorWrapper(director *processing.MessageDirector, logger customlog.Logger) *DirectorWrapper {
	return &DirectorWrapper{
		director: director,
		logger:   logger,
	}
}

// Initialize initializes the director
func (w *DirectorWrapper) Initialize(cfg interface{}) {
	// No-op, director is already initialized
}

// Start starts the director
func (w *DirectorWrapper) Start() {
	// No-op, director is already started
}

// Stop stops the director
func (w *DirectorWrapper) Stop() {
	// No-op, director is already stopped by the main application
}

// EnqueueMessage enqueues a message to process
func (w *DirectorWrapper) EnqueueMessage(msg *processing.Message) bool {
	if w.director == nil {
		return false
	}
	return w.director.EnqueueMessage(msg)
}

// RouteMessage routes an OttMessage to the director
func (w *DirectorWrapper) RouteMessage(ottMsg *message.OttMessage) error {
	if w.director == nil {
		return nil
	}
	return w.director.RouteMessage(ottMsg)
}

// SetProcessor sets the message processor
func (w *DirectorWrapper) SetProcessor(processor processing.MessageProcessor) {
	if w.director == nil {
		return
	}
	w.director.SetProcessor(processor)
}

// SetResultHandler sets the result handler
func (w *DirectorWrapper) SetResultHandler(handler processing.ResultHandler) {
	if w.director == nil {
		return
	}
	w.director.SetResultHandler(handler)
}
