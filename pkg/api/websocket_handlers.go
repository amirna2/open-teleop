package api

import (
	"encoding/json"
	"errors"
	"syscall" // Use standard import name
	"time"

	"github.com/gofiber/contrib/websocket"
	flatbuffers "github.com/google/flatbuffers/go"

	message "github.com/open-teleop/controller/pkg/flatbuffers/open_teleop/message"
	customlog "github.com/open-teleop/controller/pkg/log"
	"github.com/open-teleop/controller/pkg/processing"
)

// ControlWebSocketHandler handles incoming WebSocket messages for robot control.
// It receives JSON Twist commands, wraps them in an OttMessage FlatBuffer
// with the appropriate content type, and routes them through the MessageDirector.
func ControlWebSocketHandler(conn *websocket.Conn, logger customlog.Logger, messageDirector *processing.MessageDirector, topicRegistry *processing.TopicRegistry) {
	// Note: topicRegistry is currently unused in this handler but kept for signature consistency
	// if other WS handlers might need it. Remove if definitely not needed.
	logger.Infof("Control WebSocket connected: %s", conn.RemoteAddr())
	var (
		mt  int
		msg []byte // Raw message bytes (JSON in this case)
		err error
	)
	for {
		if mt, msg, err = conn.ReadMessage(); err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				logger.Errorf("Control WS read error: %v", err)
			} else {
				// Don't log normal closures as errors
				// Use errors.Is for checking specific syscall errors portably
				if err != websocket.ErrCloseSent && !errors.Is(err, syscall.EPIPE) && !errors.Is(err, syscall.ECONNRESET) { // Now uses syscall directly
					logger.Infof("Control WS connection closed: %v", err)
				} else {
					logger.Infof("Control WS connection closed normally.")
				}
			}
			break // Exit loop on error/close
		}

		if mt == websocket.TextMessage {
			// Validate JSON and extract values for logging
			var twist TwistMsg // Uses type from pkg/api/types.go
			if err := json.Unmarshal(msg, &twist); err != nil {
				logger.Warnf("Failed to unmarshal Twist command from WS: %v. Message: %s", err, string(msg))
				continue // Skip malformed message
			}

			// Log the received command values
			logger.Infof("Received Twist command via WS: LinearX=%.2f, AngularZ=%.2f", twist.Linear.X, twist.Angular.Z)

			// --- Convert JSON Payload to FlatBuffers OttMessage and Route ---
			builder := flatbuffers.NewBuilder(1024)
			ottTopic := "teleop.control.velocity" // The target OTT topic
			topicOffset := builder.CreateString(ottTopic)
			// Use the raw JSON bytes as the payload
			payloadOffset := builder.CreateByteVector(msg)

			message.OttMessageStart(builder)
			message.OttMessageAddOtt(builder, topicOffset)
			message.OttMessageAddTimestampNs(builder, time.Now().UnixNano())
			// Set the content type to indicate JSON payload
			message.OttMessageAddContentType(builder, message.ContentTypeJSON_COMMAND)
			message.OttMessageAddPayload(builder, payloadOffset)
			ottMessageOffset := message.OttMessageEnd(builder)

			builder.Finish(ottMessageOffset)
			fbBytes := builder.FinishedBytes()

			// Deserialize FlatBuffer to pass the correct object type to RouteMessage
			ottMsg := message.GetRootAsOttMessage(fbBytes, 0)

			// Route the FlatBuffer message via the MessageDirector
			// messageDirector is passed in as a parameter
			routeErr := messageDirector.RouteMessage(ottMsg)
			if routeErr != nil {
				logger.Errorf("Failed to route message for topic %s: %v", ottTopic, routeErr)
			} else {
				// Log that the director accepted it; processing result/publishing happens later
				logger.Debugf("MessageDirector accepted message for topic %s", ottTopic)
			}

		} else {
			logger.Infof("Ignoring non-text Control WS message type: %d", mt)
		}
	}
	logger.Infof("Control WebSocket disconnected: %s", conn.RemoteAddr())
}
