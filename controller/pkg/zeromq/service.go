package zeromq

import (
	"encoding/json"
	"errors"
	"fmt"
	"sync"
	"syscall"
	"time"

	// Assuming flatbuffer Go runtime is imported
	"github.com/open-teleop/controller/pkg/config"
	message "github.com/open-teleop/controller/pkg/flatbuffers/open_teleop/message"
	customlog "github.com/open-teleop/controller/pkg/log"
	"github.com/open-teleop/controller/pkg/processing"
	"github.com/pebbe/zmq4"
)

// Common errors
var (
	ErrServiceClosed      = errors.New("zeromq service is closed")
	ErrInvalidMessage     = errors.New("invalid message format")
	ErrUnknownMessageType = errors.New("unknown message type")
)

// Message types
const (
	MsgTypeConfigRequest  = "CONFIG_REQUEST"
	MsgTypeConfigResponse = "CONFIG_RESPONSE"
	MsgTypeError          = "ERROR"
)

// Socket types
const (
	ZMQ_REP = 4 // REP socket type
	ZMQ_PUB = 1 // PUB socket type
)

// ZMQ flags
const (
	ZMQ_POLLIN  = 1 // POLLIN flag
	ZMQ_SNDMORE = 2 // SNDMORE flag
)

// ZeroMQMessage represents a generic message structure for ZeroMQ communication
type ZeroMQMessage struct {
	Type      string      `json:"type"`
	Timestamp float64     `json:"timestamp"`
	Data      interface{} `json:"data,omitempty"`
}

// ErrorResponse represents an error response message
type ErrorResponse struct {
	Message string `json:"message"`
	Code    int    `json:"code"`
}

// MessageHandler defines the interface for handlers that process specific message types
type MessageHandler interface {
	HandleMessage(data []byte) ([]byte, error)
}

// HandlerFunc is a function type that implements MessageHandler
type HandlerFunc func(data []byte) ([]byte, error)

// HandleMessage calls the function
func (f HandlerFunc) HandleMessage(data []byte) ([]byte, error) {
	return f(data)
}

// MessageReceiver handles receiving messages from a ZeroMQ socket
type MessageReceiver struct {
	socket     *zmq4.Socket
	dispatcher *MessageDispatcher
	poller     *zmq4.Poller
	logger     customlog.Logger
	running    bool
	wg         *sync.WaitGroup
}

// newMessageReceiver creates a new MessageReceiver
func newMessageReceiver(ctx *zmq4.Context, bindAddress string, dispatcher *MessageDispatcher, logger customlog.Logger, wg *sync.WaitGroup) (*MessageReceiver, error) {
	// Create REP socket for receiving requests
	socket, err := ctx.NewSocket(zmq4.Type(zmq4.REP))
	if err != nil {
		return nil, fmt.Errorf("failed to create REP socket: %w", err)
	}

	// Bind to the address from config
	if err := socket.Bind(bindAddress); err != nil {
		socket.Close()
		return nil, fmt.Errorf("failed to bind to %s: %w", bindAddress, err)
	}

	// Configure socket options
	if err := socket.SetLinger(0); err != nil {
		socket.Close()
		return nil, fmt.Errorf("failed to set linger option: %w", err)
	}

	// Set receive and send timeouts to prevent indefinite blocking during shutdown
	const socketTimeout = 1 * time.Second
	if err := socket.SetRcvtimeo(socketTimeout); err != nil {
		socket.Close()
		return nil, fmt.Errorf("failed to set receive timeout: %w", err)
	}
	if err := socket.SetSndtimeo(socketTimeout); err != nil {
		socket.Close()
		return nil, fmt.Errorf("failed to set send timeout: %w", err)
	}

	// Create poller for non-blocking receives
	poller := zmq4.NewPoller()
	poller.Add(socket, zmq4.State(zmq4.POLLIN))

	logger.Infof("MessageReceiver initialized on %s", bindAddress)

	return &MessageReceiver{
		socket:     socket,
		dispatcher: dispatcher,
		poller:     poller,
		logger:     logger,
		running:    false,
		wg:         wg,
	}, nil
}

// Start begins the message receiving loop
func (r *MessageReceiver) Start() {
	if r.running {
		return
	}

	r.running = true
	r.wg.Add(1)

	go func() {
		defer r.wg.Done()
		r.logger.Infof("MessageReceiver started")

		for r.running {
			// Poll for messages with timeout to allow for clean shutdown
			sockets, err := r.poller.Poll(500 * time.Millisecond)
			if err != nil {
				if r.running {
					r.logger.Errorf("Error polling socket: %v", err)
				}
				continue
			}

			if len(sockets) == 0 {
				// No messages, continue polling
				continue
			}

			// Receive message
			msg, err := r.socket.RecvBytes(0)
			if err != nil {
				// Check specifically for timeout/non-blocking error (EAGAIN)
				if zmqErr, ok := err.(zmq4.Errno); ok && zmqErr == zmq4.Errno(syscall.EAGAIN) {
					// Timeout is expected, continue loop
					continue
				}
				// Log other receive errors
				if r.running {
					r.logger.Errorf("Error receiving message: %v", err)
				}
				continue
			}

			r.logger.Debugf("Received message (%d bytes)", len(msg))

			// Process message through dispatcher
			response, err := r.dispatcher.Dispatch(msg)
			if err != nil {
				r.logger.Errorf("Error dispatching message: %v", err)

				// Create and send error response
				errorResp := ZeroMQMessage{
					Type:      MsgTypeError,
					Timestamp: float64(time.Now().Unix()),
					Data: ErrorResponse{
						Message: err.Error(),
						Code:    500,
					},
				}

				errData, marshalErr := json.Marshal(errorResp)
				if marshalErr != nil {
					r.logger.Errorf("Failed to marshal error response: %v", marshalErr)
					continue // Skip sending if marshal fails
				}
				if _, sendErr := r.socket.SendBytes(errData, 0); sendErr != nil && r.running {
					r.logger.Errorf("Error sending error response: %v", sendErr)
				}
				continue
			}

			// Send response
			if _, sendErr := r.socket.SendBytes(response, 0); sendErr != nil && r.running {
				r.logger.Errorf("Error sending response: %v", sendErr)
			}
		}
	}()
}

// Stop halts the message receiving loop
func (r *MessageReceiver) Stop() {
	if !r.running {
		return
	}
	r.running = false
	if r.socket != nil {
		r.logger.Infof("MessageReceiver: Closing socket to interrupt blocking calls")
		r.socket.Close()
	}
}

// Close cleans up resources (socket might already be closed by Stop)
func (r *MessageReceiver) Close() {
	r.Stop()
	if r.socket != nil {
		// Optional: Log if closing again, though Stop should handle it.
		// r.logger.Debugf("Socket already closed by Stop or closing again.")
	}
	r.socket = nil
}

// MessageSender handles sending messages to ZeroMQ sockets
type MessageSender struct {
	socket  *zmq4.Socket
	logger  customlog.Logger
	running bool
	mu      sync.Mutex
}

// newMessageSender creates a new MessageSender
func newMessageSender(ctx *zmq4.Context, bindAddress string, logger customlog.Logger) (*MessageSender, error) {
	// Create PUB socket for publishing messages
	socket, err := ctx.NewSocket(zmq4.Type(zmq4.PUB))
	if err != nil {
		return nil, fmt.Errorf("failed to create PUB socket: %w", err)
	}

	// Bind to the address from config
	if err := socket.Bind(bindAddress); err != nil {
		socket.Close()
		return nil, fmt.Errorf("failed to bind to %s: %w", bindAddress, err)
	}

	// Configure socket options
	if err := socket.SetLinger(0); err != nil {
		socket.Close()
		return nil, fmt.Errorf("failed to set linger option: %w", err)
	}

	// Set send timeout for the publisher
	const socketTimeout = 1 * time.Second
	if err := socket.SetSndtimeo(socketTimeout); err != nil {
		socket.Close()
		return nil, fmt.Errorf("failed to set send timeout for PUB socket: %w", err)
	}

	logger.Infof("MessageSender initialized on %s", bindAddress)

	return &MessageSender{
		socket:  socket,
		logger:  logger,
		running: true, // Assume running on creation for sender
	}, nil
}

// PublishMessage sends a message with the given topic
func (s *MessageSender) PublishMessage(topic string, message []byte) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if !s.running {
		return ErrServiceClosed
	}

	s.logger.Debugf("Publishing message to topic '%s' (%d bytes)", topic, len(message))

	// Send two messages in sequence (topic first, then message)
	_, err := s.socket.Send(topic, zmq4.Flag(zmq4.SNDMORE))
	if err != nil {
		return fmt.Errorf("failed to send topic: %w", err)
	}

	_, err = s.socket.SendBytes(message, 0)
	if err != nil {
		return fmt.Errorf("failed to send message: %w", err)
	}

	return nil
}

// Close cleans up resources
func (s *MessageSender) Close() {
	s.mu.Lock()
	defer s.mu.Unlock()

	s.running = false
	if s.socket != nil {
		s.logger.Infof("MessageSender: Closing socket")
		s.socket.Close()
		s.socket = nil
	}
}

// MessageDispatcher routes messages to the appropriate handlers
type MessageDispatcher struct {
	handlers        map[string]MessageHandler
	logger          customlog.Logger
	mu              sync.RWMutex
	topicProcessors map[string]func(*message.OttMessage, []byte) ([]byte, error)
	messageDirector *processing.MessageDirector
}

// NewMessageDispatcher creates a new message dispatcher
func NewMessageDispatcher(logger customlog.Logger) *MessageDispatcher {
	return &MessageDispatcher{
		handlers:        make(map[string]MessageHandler),
		logger:          logger,
		mu:              sync.RWMutex{},
		topicProcessors: make(map[string]func(*message.OttMessage, []byte) ([]byte, error)),
	}
}

// RegisterHandler adds a handler for a specific message type
func (d *MessageDispatcher) RegisterHandler(messageType string, handler MessageHandler) {
	d.mu.Lock()
	defer d.mu.Unlock()

	d.handlers[messageType] = handler
	d.logger.Infof("Registered handler for message type: %s", messageType)
}

// RegisterTopicProcessor registers a processor for a specific topic
func (d *MessageDispatcher) RegisterTopicProcessor(
	topic string,
	processor func(*message.OttMessage, []byte) ([]byte, error)) {

	d.mu.Lock()
	defer d.mu.Unlock()

	d.topicProcessors[topic] = processor
	d.logger.Infof("Registered processor for topic: %s", topic)
}

// Dispatch processes a message and routes it to the appropriate handler
func (d *MessageDispatcher) Dispatch(data []byte) ([]byte, error) {
	// First, try to parse as standard JSON message
	var msg ZeroMQMessage
	if err := json.Unmarshal(data, &msg); err == nil {
		// Successfully parsed as JSON, check if handler exists
		d.logger.Debugf("Dispatching JSON message of type: %s", msg.Type)
		d.mu.RLock()
		handler, exists := d.handlers[msg.Type]
		d.mu.RUnlock()

		if !exists {
			return nil, fmt.Errorf("%w: %s", ErrUnknownMessageType, msg.Type)
		}
		// Process with registered JSON handler
		return handler.HandleMessage(data)
	}

	// JSON parsing failed, assume it's a raw OttMessage FlatBuffer
	d.logger.Debugf("JSON parse failed, attempting to handle as raw Flatbuffer (%d bytes)", len(data))
	return d.handleRawFlatbuffer(data)
}

// handleRawFlatbuffer processes raw incoming data as an OttMessage FlatBuffer
func (d *MessageDispatcher) handleRawFlatbuffer(data []byte) ([]byte, error) {
	// +++ Add logging for received raw bytes +++
	if len(data) > 32 {
		d.logger.Debugf("DEBUG handleRawFlatbuffer: Received %d bytes. Data (hex): %x...", len(data), data[:32])
	} else {
		d.logger.Debugf("DEBUG handleRawFlatbuffer: Received %d bytes. Data (hex): %x", len(data), data)
	}

	// Parse the raw bytes as an OttMessage FlatBuffer
	ottMsg := message.GetRootAsOttMessage(data, 0)

	// Extract data from FlatBuffer
	ottTopic := string(ottMsg.Ott())
	contentType := ottMsg.ContentType()
	payloadBytes := ottMsg.PayloadBytes()
	timestampNs := ottMsg.TimestampNs()
	version := ottMsg.Version()

	d.logger.Debugf(
		"Successfully parsed raw Flatbuffer: Topic='%s', Type=%d, PayloadSize=%d, Timestamp=%d, Version=%d",
		ottTopic,
		contentType,
		len(payloadBytes),
		timestampNs,
		version,
	)

	d.mu.RLock()
	director := d.messageDirector
	d.mu.RUnlock()

	// If we have a message director, use it
	if director != nil {
		// Route message through MessageDirector
		d.logger.Debugf("Routing message for topic '%s' through MessageDirector", ottTopic)
		if err := (*director).RouteMessage(ottMsg); err != nil {
			d.logger.Errorf("Error routing message through MessageDirector: %v", err)

			// Create error response
			errResponse := ZeroMQMessage{
				Type:      "ERROR",
				Timestamp: float64(time.Now().Unix()),
				Data: map[string]interface{}{
					"status":  "ERROR",
					"topic":   ottTopic,
					"message": fmt.Sprintf("Failed to route message: %v", err),
				},
			}

			responseData, jsonErr := json.Marshal(errResponse)
			if jsonErr != nil {
				return nil, fmt.Errorf("failed to create error response: %w", jsonErr)
			}

			return responseData, nil // Return error response but don't propagate error
		}

		// Send back success ACK response
		ackResponse := ZeroMQMessage{
			Type:      "ACK",
			Timestamp: float64(time.Now().Unix()),
			Data: map[string]interface{}{
				"status":  "OK",
				"topic":   ottTopic,
				"message": "Message routed for processing",
			},
		}

		responseData, jsonErr := json.Marshal(ackResponse)
		if jsonErr != nil {
			return nil, fmt.Errorf("failed to create ACK response: %w", jsonErr)
		}

		return responseData, nil
	}

	// Fall back to existing topic processor logic if MessageDirector not available
	d.mu.RLock()
	processor, exists := d.topicProcessors[ottTopic]
	d.mu.RUnlock()

	var responseData []byte
	var err error

	if exists {
		// Process the message using the registered processor
		d.logger.Debugf("Processing Flatbuffer for topic '%s' using registered processor", ottTopic)
		responseData, err = processor(ottMsg, payloadBytes)
		if err != nil {
			d.logger.Errorf("Error processing topic '%s': %v", ottTopic, err)
		}
	} else {
		// No processor registered for this topic
		d.logger.Warnf("Flatbuffer Processing for topic '%s' NOT IMPLEMENTED", ottTopic)

		// Send back a simple ACK response (as JSON string)
		ackResponse := ZeroMQMessage{
			Type:      "ACK",
			Timestamp: float64(time.Now().Unix()),
			Data: map[string]interface{}{ // Use map for generic ACK data
				"status":  "OK",
				"topic":   ottTopic, // Include topic in ACK
				"message": "Raw Flatbuffer received, but no processor registered",
			},
		}

		responseData, err = json.Marshal(ackResponse)
		if err != nil {
			d.logger.Errorf("Error serializing ACK response for raw flatbuffer topic %s: %v", ottTopic, err)
			return nil, fmt.Errorf("failed to serialize ACK response: %w", err)
		}
	}

	// Return the response data
	return responseData, nil
}

// SetMessageDirector sets the MessageDirector for the dispatcher
func (d *MessageDispatcher) SetMessageDirector(director *processing.MessageDirector) {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.messageDirector = director
}

// ZeroMQService coordinates ZeroMQ communications for the controller
type ZeroMQService struct {
	config     *config.Config
	ctx        *zmq4.Context
	receiver   *MessageReceiver
	sender     *MessageSender
	dispatcher *MessageDispatcher
	logger     customlog.Logger
	running    bool
	wg         *sync.WaitGroup
}

// NewZeroMQService creates a new ZeroMQService
// Accepts specific bind addresses from bootstrap config and the operational zeromq config.
func NewZeroMQService(requestBindAddr string, publishBindAddr string, operationalCfg config.ZeroMQConfig, logger customlog.Logger) (*ZeroMQService, error) {
	// Create ZeroMQ context
	ctx, err := zmq4.NewContext()
	if err != nil {
		return nil, fmt.Errorf("failed to create ZeroMQ context: %w", err)
	}

	var wg sync.WaitGroup

	// Create dispatcher
	dispatcher := NewMessageDispatcher(logger)

	// Create message receiver
	receiver, err := newMessageReceiver(ctx, requestBindAddr, dispatcher, logger, &wg) // Pass requestBindAddr
	if err != nil {
		ctx.Term()
		return nil, fmt.Errorf("failed to create message receiver: %w", err)
	}

	// Create message sender
	sender, err := newMessageSender(ctx, publishBindAddr, logger) // Pass publishBindAddr
	if err != nil {
		receiver.Close()
		ctx.Term()
		return nil, fmt.Errorf("failed to create message sender: %w", err)
	}

	logger.Infof("ZeroMQService initialized")

	return &ZeroMQService{
		// config:     cfg, // No longer need the full operational config here directly
		ctx:        ctx,
		receiver:   receiver,
		sender:     sender,
		dispatcher: dispatcher,
		logger:     logger,
		running:    false,
		wg:         &wg,
	}, nil
}

// RegisterHandler adds a handler for a specific message type
func (s *ZeroMQService) RegisterHandler(messageType string, handler MessageHandler) {
	s.dispatcher.RegisterHandler(messageType, handler)
}

// RegisterHandlerFunc adds a handler function for a specific message type
func (s *ZeroMQService) RegisterHandlerFunc(messageType string, handler func([]byte) ([]byte, error)) {
	s.dispatcher.RegisterHandler(messageType, HandlerFunc(handler))
}

// RegisterTopicProcessor adds a processor for a specific topic
func (s *ZeroMQService) RegisterTopicProcessor(
	topic string,
	processor func(*message.OttMessage, []byte) ([]byte, error)) {

	s.dispatcher.RegisterTopicProcessor(topic, processor)
}

// Start begins the ZeroMQ service
func (s *ZeroMQService) Start() error {
	if s.running {
		return nil
	}

	s.running = true
	s.logger.Infof("Starting ZeroMQ service")

	// Start the receiver
	s.receiver.Start()

	return nil
}

// Stop halts the ZeroMQ service
func (s *ZeroMQService) Stop() {
	if !s.running {
		return
	}

	s.logger.Infof("Stopping ZeroMQ service")
	s.running = false

	// Stop the receiver - THIS NOW CLOSES THE SOCKET TOO
	s.receiver.Stop()

	// Close the sender
	s.sender.Close()

	// Wait for goroutines to finish
	s.logger.Infof("Waiting for receiver goroutine to finish...")
	s.wg.Wait()
	s.logger.Infof("Receiver goroutine finished.")

	// Clean up ZeroMQ context
	if s.ctx != nil {
		s.logger.Infof("Terminating ZeroMQ context")
		s.ctx.Term()
		s.ctx = nil
	}

	s.logger.Infof("ZeroMQ service stopped")
}

// PublishMessage sends a message with the given topic
func (s *ZeroMQService) PublishMessage(topic string, message []byte) error {
	if !s.running {
		return ErrServiceClosed
	}
	return s.sender.PublishMessage(topic, message)
}

// PublishJSON publishes a JSON-serializable message with the given topic
func (s *ZeroMQService) PublishJSON(topic string, messageType string, data interface{}) error {
	msg := ZeroMQMessage{
		Type:      messageType,
		Timestamp: float64(time.Now().Unix()),
		Data:      data,
	}

	msgData, err := json.Marshal(msg)
	if err != nil {
		return fmt.Errorf("failed to marshal message: %w", err)
	}

	return s.PublishMessage(topic, msgData)
}

// SetMessageDirector sets the message director for processing incoming messages
func (s *ZeroMQService) SetMessageDirector(director *processing.MessageDirector) {
	if s.dispatcher != nil {
		s.dispatcher.SetMessageDirector(director)
	}
}
