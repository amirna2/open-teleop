package zeromq

import (
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"sync"
	"time"

	"github.com/open-teleop/controller/pkg/config"
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
	logger     *log.Logger
	running    bool
	wg         *sync.WaitGroup
}

// newMessageReceiver creates a new MessageReceiver
func newMessageReceiver(ctx *zmq4.Context, cfg *config.Config, dispatcher *MessageDispatcher, logger *log.Logger, wg *sync.WaitGroup) (*MessageReceiver, error) {
	// Create REP socket for receiving requests
	socket, err := ctx.NewSocket(zmq4.Type(zmq4.REP))
	if err != nil {
		return nil, fmt.Errorf("failed to create REP socket: %w", err)
	}

	// Bind to the address from config
	if err := socket.Bind(cfg.ZeroMQ.ControllerAddress); err != nil {
		socket.Close()
		return nil, fmt.Errorf("failed to bind to %s: %w", cfg.ZeroMQ.ControllerAddress, err)
	}

	// Configure socket options
	if err := socket.SetLinger(0); err != nil {
		socket.Close()
		return nil, fmt.Errorf("failed to set linger option: %w", err)
	}

	// Create poller for non-blocking receives
	poller := zmq4.NewPoller()
	poller.Add(socket, zmq4.State(zmq4.POLLIN))

	logger.Printf("MessageReceiver initialized on %s", cfg.ZeroMQ.ControllerAddress)

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
		r.logger.Printf("MessageReceiver started")

		for r.running {
			// Poll for messages with timeout to allow for clean shutdown
			sockets, err := r.poller.Poll(500 * time.Millisecond)
			if err != nil {
				if r.running {
					r.logger.Printf("Error polling socket: %v", err)
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
				if r.running {
					r.logger.Printf("Error receiving message: %v", err)
				}
				continue
			}

			r.logger.Printf("Received message (%d bytes)", len(msg))

			// Process message through dispatcher
			response, err := r.dispatcher.Dispatch(msg)
			if err != nil {
				r.logger.Printf("Error dispatching message: %v", err)

				// Create and send error response
				errorResp := ZeroMQMessage{
					Type:      MsgTypeError,
					Timestamp: float64(time.Now().Unix()),
					Data: ErrorResponse{
						Message: err.Error(),
						Code:    500,
					},
				}

				errData, _ := json.Marshal(errorResp)
				if _, err := r.socket.SendBytes(errData, 0); err != nil && r.running {
					r.logger.Printf("Error sending error response: %v", err)
				}
				continue
			}

			// Send response
			if _, err := r.socket.SendBytes(response, 0); err != nil && r.running {
				r.logger.Printf("Error sending response: %v", err)
			}
		}
	}()
}

// Stop halts the message receiving loop
func (r *MessageReceiver) Stop() {
	r.running = false
}

// Close cleans up resources
func (r *MessageReceiver) Close() {
	r.Stop()
	if r.socket != nil {
		r.socket.Close()
		r.socket = nil
	}
}

// MessageSender handles sending messages to ZeroMQ sockets
type MessageSender struct {
	socket  *zmq4.Socket
	logger  *log.Logger
	running bool
	mu      sync.Mutex
}

// newMessageSender creates a new MessageSender
func newMessageSender(ctx *zmq4.Context, cfg *config.Config, logger *log.Logger) (*MessageSender, error) {
	// Create PUB socket for publishing messages
	socket, err := ctx.NewSocket(zmq4.Type(zmq4.PUB))
	if err != nil {
		return nil, fmt.Errorf("failed to create PUB socket: %w", err)
	}

	// Bind to the address from config
	pubAddress := cfg.ZeroMQ.GatewayAddress
	if err := socket.Bind(pubAddress); err != nil {
		socket.Close()
		return nil, fmt.Errorf("failed to bind to %s: %w", pubAddress, err)
	}

	// Configure socket options
	if err := socket.SetLinger(0); err != nil {
		socket.Close()
		return nil, fmt.Errorf("failed to set linger option: %w", err)
	}

	logger.Printf("MessageSender initialized on %s", pubAddress)

	return &MessageSender{
		socket:  socket,
		logger:  logger,
		running: true,
		mu:      sync.Mutex{},
	}, nil
}

// PublishMessage sends a message with the given topic
func (s *MessageSender) PublishMessage(topic string, message []byte) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if !s.running {
		return ErrServiceClosed
	}

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
		s.socket.Close()
		s.socket = nil
	}
}

// MessageDispatcher routes messages to the appropriate handlers
type MessageDispatcher struct {
	handlers map[string]MessageHandler
	logger   *log.Logger
	mu       sync.RWMutex
}

// NewMessageDispatcher creates a new message dispatcher
func NewMessageDispatcher(logger *log.Logger) *MessageDispatcher {
	return &MessageDispatcher{
		handlers: make(map[string]MessageHandler),
		logger:   logger,
		mu:       sync.RWMutex{},
	}
}

// RegisterHandler adds a handler for a specific message type
func (d *MessageDispatcher) RegisterHandler(messageType string, handler MessageHandler) {
	d.mu.Lock()
	defer d.mu.Unlock()

	d.handlers[messageType] = handler
	d.logger.Printf("Registered handler for message type: %s", messageType)
}

// Dispatch processes a message and routes it to the appropriate handler
func (d *MessageDispatcher) Dispatch(data []byte) ([]byte, error) {
	// Parse message to determine type
	var msg ZeroMQMessage
	if err := json.Unmarshal(data, &msg); err != nil {
		return nil, fmt.Errorf("failed to parse message: %w", err)
	}

	d.logger.Printf("Dispatching message of type: %s", msg.Type)

	// Find handler for message type
	d.mu.RLock()
	handler, exists := d.handlers[msg.Type]
	d.mu.RUnlock()

	if !exists {
		return nil, fmt.Errorf("%w: %s", ErrUnknownMessageType, msg.Type)
	}

	// Process message with handler
	return handler.HandleMessage(data)
}

// ZeroMQService coordinates ZeroMQ communications for the controller
type ZeroMQService struct {
	config     *config.Config
	ctx        *zmq4.Context
	receiver   *MessageReceiver
	sender     *MessageSender
	dispatcher *MessageDispatcher
	logger     *log.Logger
	running    bool
	wg         sync.WaitGroup
}

// NewZeroMQService creates a new ZeroMQ service
func NewZeroMQService(cfg *config.Config, logger *log.Logger) (*ZeroMQService, error) {
	// Create ZeroMQ context
	ctx, err := zmq4.NewContext()
	if err != nil {
		return nil, fmt.Errorf("failed to create ZMQ context: %w", err)
	}

	// Create components
	dispatcher := NewMessageDispatcher(logger)

	// Set up wait group for clean shutdown
	var wg sync.WaitGroup

	// Create receiver
	receiver, err := newMessageReceiver(ctx, cfg, dispatcher, logger, &wg)
	if err != nil {
		ctx.Term()
		return nil, err
	}

	// Create sender
	sender, err := newMessageSender(ctx, cfg, logger)
	if err != nil {
		receiver.Close()
		ctx.Term()
		return nil, err
	}

	return &ZeroMQService{
		config:     cfg,
		ctx:        ctx,
		receiver:   receiver,
		sender:     sender,
		dispatcher: dispatcher,
		logger:     logger,
		running:    false,
		wg:         wg,
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

// Start begins the ZeroMQ service
func (s *ZeroMQService) Start() error {
	if s.running {
		return nil
	}

	s.running = true
	s.logger.Printf("Starting ZeroMQ service")

	// Start the receiver
	s.receiver.Start()

	return nil
}

// Stop halts the ZeroMQ service
func (s *ZeroMQService) Stop() {
	if !s.running {
		return
	}

	s.logger.Printf("Stopping ZeroMQ service")
	s.running = false

	// Stop the receiver
	s.receiver.Stop()

	// Close the sender
	s.sender.Close()

	// Wait for goroutines to finish
	s.wg.Wait()

	// Clean up ZeroMQ context
	if s.ctx != nil {
		s.ctx.Term()
		s.ctx = nil
	}

	s.logger.Printf("ZeroMQ service stopped")
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
