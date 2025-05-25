package video

import (
	"sync"

	"github.com/gofiber/contrib/websocket"
	customlog "github.com/open-teleop/controller/pkg/log"
)

// VideoService handles video streaming from robots
// Now manages WebSocket clients for video streaming
// Future: can be extended for WebRTC, etc.
type VideoService struct {
	mu      sync.Mutex
	clients map[*websocket.Conn]struct{}
	logger  customlog.Logger
}

// NewVideoService creates a new video service instance
func NewVideoService(logger customlog.Logger) *VideoService {
	return &VideoService{
		clients: make(map[*websocket.Conn]struct{}),
		logger:  logger,
	}
}

// RegisterWebSocketClient adds a new WebSocket client for video streaming
func (s *VideoService) RegisterWebSocketClient(conn *websocket.Conn) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.clients[conn] = struct{}{}
	s.logger.Infof("Video WS client connected: %s (total: %d)", conn.RemoteAddr(), len(s.clients))
}

// UnregisterWebSocketClient removes a WebSocket client
func (s *VideoService) UnregisterWebSocketClient(conn *websocket.Conn) {
	s.mu.Lock()
	defer s.mu.Unlock()
	delete(s.clients, conn)
	s.logger.Infof("Video WS client disconnected: %s (total: %d)", conn.RemoteAddr(), len(s.clients))
}

// BroadcastVideoFrame sends a video frame to all connected WebSocket clients
// Fast relay: sends the entire FlatBuffer message as-is to maintain performance
func (s *VideoService) BroadcastVideoFrame(topic string, timestamp int64, messageData []byte) {
	s.mu.Lock()
	defer s.mu.Unlock()

	// Fast relay: send the entire FlatBuffer message as-is
	for conn := range s.clients {
		if err := conn.WriteMessage(websocket.BinaryMessage, messageData); err != nil {
			s.logger.Warnf("Failed to send video frame to %s: %v. Removing client.", conn.RemoteAddr(), err)
			conn.Close()
			delete(s.clients, conn)
		}
	}
	s.logger.Infof("Broadcasted video frame (%d bytes) to %d clients (topic: %s, timestamp: %d)", len(messageData), len(s.clients), topic, timestamp)
}

// GetClientCount returns the number of currently connected clients
func (s *VideoService) GetClientCount() int {
	s.mu.Lock()
	defer s.mu.Unlock()
	return len(s.clients)
}
