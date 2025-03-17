package test

import (
	"encoding/json"
	"fmt"
	"testing"
	"time"

	"github.com/pebbe/zmq4"
)

// Message represents a generic message structure for ZeroMQ communication
type Message struct {
	Type      string      `json:"type"`
	Timestamp float64     `json:"timestamp"`
	Data      interface{} `json:"data,omitempty"`
}

// TestRequestClient tests sending a CONFIG_REQUEST to the ZeroMQ service
func TestRequestClient(t *testing.T) {
	// This test is meant to be run manually when a controller is running
	// It simulates a gateway sending a CONFIG_REQUEST

	// Create a REQ socket to connect to the controller
	ctx, err := zmq4.NewContext()
	if err != nil {
		t.Fatalf("Failed to create ZMQ context: %v", err)
	}
	defer ctx.Term()

	socket, err := ctx.NewSocket(zmq4.Type(zmq4.REQ))
	if err != nil {
		t.Fatalf("Failed to create REQ socket: %v", err)
	}
	defer socket.Close()

	// Connect to the controller
	err = socket.Connect("tcp://localhost:5555")
	if err != nil {
		t.Fatalf("Failed to connect to controller: %v", err)
	}

	// Create a CONFIG_REQUEST message
	req := Message{
		Type:      "CONFIG_REQUEST",
		Timestamp: float64(time.Now().Unix()),
	}

	// Serialize the message
	reqData, err := json.Marshal(req)
	if err != nil {
		t.Fatalf("Failed to marshal request: %v", err)
	}

	// Send the request
	_, err = socket.SendBytes(reqData, 0)
	if err != nil {
		t.Fatalf("Failed to send request: %v", err)
	}

	fmt.Println("Sent CONFIG_REQUEST, waiting for response...")

	// Set a timeout
	socket.SetRcvtimeo(5 * time.Second)

	// Receive the response
	respData, err := socket.RecvBytes(0)
	if err != nil {
		t.Fatalf("Failed to receive response: %v", err)
	}

	// Parse the response
	var resp Message
	err = json.Unmarshal(respData, &resp)
	if err != nil {
		t.Fatalf("Failed to unmarshal response: %v", err)
	}

	// Verify the response
	if resp.Type != "CONFIG_RESPONSE" {
		t.Errorf("Expected response type 'CONFIG_RESPONSE', got '%s'", resp.Type)
	}

	fmt.Printf("Received CONFIG_RESPONSE:\nType: %s\nTimestamp: %f\nData: %v\n",
		resp.Type, resp.Timestamp, resp.Data)
}

// TestSubscriber tests subscribing to configuration updates from the ZeroMQ service
func TestSubscriber(t *testing.T) {
	// This test is meant to be run manually when a controller is running
	// It simulates a gateway subscribing to configuration updates

	// Create a SUB socket to connect to the controller
	ctx, err := zmq4.NewContext()
	if err != nil {
		t.Fatalf("Failed to create ZMQ context: %v", err)
	}
	defer ctx.Term()

	socket, err := ctx.NewSocket(zmq4.Type(zmq4.SUB))
	if err != nil {
		t.Fatalf("Failed to create SUB socket: %v", err)
	}
	defer socket.Close()

	// Connect to the controller
	err = socket.Connect("tcp://localhost:5556")
	if err != nil {
		t.Fatalf("Failed to connect to controller: %v", err)
	}

	// Subscribe to the configuration topic
	err = socket.SetSubscribe("configuration.")
	if err != nil {
		t.Fatalf("Failed to set subscription: %v", err)
	}

	fmt.Println("Subscribed to 'configuration.' topics, waiting for messages...")
	fmt.Println("Press Ctrl+C to stop")

	// Receive messages in a loop
	for {
		// Set a timeout
		socket.SetRcvtimeo(1 * time.Second)

		// Receive the topic
		topic, err := socket.Recv(0)
		if err != nil {
			if err.Error() == "resource temporarily unavailable" {
				// Timeout, continue
				continue
			}
			t.Fatalf("Failed to receive topic: %v", err)
		}

		// Receive the message
		msgData, err := socket.RecvBytes(0)
		if err != nil {
			t.Fatalf("Failed to receive message: %v", err)
		}

		// Parse the message
		var msg Message
		err = json.Unmarshal(msgData, &msg)
		if err != nil {
			t.Fatalf("Failed to unmarshal message: %v", err)
		}

		fmt.Printf("Received message on topic '%s':\nType: %s\nTimestamp: %f\n",
			topic, msg.Type, msg.Timestamp)
	}
}
