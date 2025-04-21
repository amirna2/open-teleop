package api

import (
	"encoding/json"
	"errors"
	syscall "syscall" // Renamed to avoid conflict if necessary
	"time"

	"github.com/gofiber/contrib/websocket"
	flatbuffers "github.com/google/flatbuffers/go" // Corrected import path
	customlog "github.com/open-teleop/controller/pkg/log"
	"github.com/open-teleop/controller/pkg/processing"

	// Import generated flatbuffer code
	message "github.com/open-teleop/controller/pkg/flatbuffers/open_teleop/message"
	// common "github.com/open-teleop/controller/pkg/flatbuffers/open_teleop/common" // Not needed here
)

// ControlWebSocketHandler handles incoming WebSocket messages for robot control.
func ControlWebSocketHandler(conn *websocket.Conn, logger customlog.Logger, messageDirector *processing.MessageDirector, topicRegistry *processing.TopicRegistry) {
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
				if err != websocket.ErrCloseSent && !errors.Is(err, syscall.EPIPE) && !errors.Is(err, syscall.ECONNRESET) {
					logger.Infof("Control WS connection closed: %v", err)
				} else {
					logger.Infof("Control WS connection closed normally.")
				}
			}
			break // Exit loop on error/close
		}
		// logger.Debugf("Control WS message received (type %d): %s", mt, string(msg))

		if mt == websocket.TextMessage {
			// Try to unmarshal just to validate JSON and log values, but use raw bytes for payload
			var twist TwistMsg // Use the type defined in this package (pkg/api/types.go)
			if err := json.Unmarshal(msg, &twist); err != nil {
				logger.Warnf("Failed to unmarshal Twist command from WS: %v. Message: %s", err, string(msg))
				continue // Skip malformed message
			}

			// Log the received command values from parsed struct
			logger.Infof("Received Twist command via WS: LinearX=%.2f, AngularZ=%.2f", twist.Linear.X, twist.Angular.Z)

			// --- Convert JSON Payload to FlatBuffers OttMessage and Route ---
			builder := flatbuffers.NewBuilder(1024)
			ottTopic := "teleop.control.velocity" // The target OTT topic
			topicOffset := builder.CreateString(ottTopic)
			payloadOffset := builder.CreateByteVector(msg) // Use raw JSON message bytes

			message.OttMessageStart(builder)
			message.OttMessageAddOtt(builder, topicOffset)
			message.OttMessageAddTimestampNs(builder, time.Now().UnixNano())
			message.OttMessageAddContentType(builder, message.ContentTypeJSON_COMMAND) // Use correct enum
			message.OttMessageAddPayload(builder, payloadOffset)
			ottMessageOffset := message.OttMessageEnd(builder)

			builder.Finish(ottMessageOffset)
			fbBytes := builder.FinishedBytes()

			// Deserialize to pass the required object type to RouteMessage
			ottMsg := message.GetRootAsOttMessage(fbBytes, 0)

			// Route the FlatBuffer message via the MessageDirector
			routeErr := messageDirector.RouteMessage(ottMsg) // Renamed err variable
			if routeErr != nil {
				logger.Errorf("Failed to route message for topic %s: %v", ottTopic, routeErr)
			} else {
				logger.Debugf("MessageDirector accepted message for topic %s", ottTopic)
			}

		} else {
			logger.Infof("Ignoring non-text Control WS message type: %d", mt)
		}
	}
	logger.Infof("Control WebSocket disconnected: %s", conn.RemoteAddr())
}
