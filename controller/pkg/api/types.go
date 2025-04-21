package api

// --- Data Structures for WebSocket Messages ---

// Vector3 defines a standard 3D vector.
type Vector3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// TwistMsg represents a command velocity message, matching geometry_msgs/Twist.
type TwistMsg struct {
	Linear  Vector3 `json:"linear"`
	Angular Vector3 `json:"angular"`
}
