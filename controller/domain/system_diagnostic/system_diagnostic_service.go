package diagnostic

import (
	"sync"
	"time"

	"github.com/gofiber/fiber/v2"
)

// SystemMetrics represents system diagnostics information
type SystemMetrics struct {
	Timestamp   time.Time `json:"timestamp"`
	CPUUsage    float64   `json:"cpu_usage"`    // Percentage of CPU used
	MemoryUsage float64   `json:"memory_usage"` // Percentage of memory used
	DiskUsage   float64   `json:"disk_usage"`   // Percentage of disk used
	RobotID     string    `json:"robot_id"`
	NodeStatus  []NodeStatus `json:"node_status"`
}

// NodeStatus represents the status of a ROS2 node
type NodeStatus struct {
	Name   string `json:"name"`
	Status string `json:"status"` // "active", "error", "warning", etc.
	PID    int    `json:"pid"`
}

// DiagnosticService handles system diagnostics
type DiagnosticService struct {
	mu      sync.RWMutex
	metrics SystemMetrics
}

// NewDiagnosticService creates a new diagnostic service instance
func NewDiagnosticService() *DiagnosticService {
	return &DiagnosticService{
		metrics: SystemMetrics{
			Timestamp:   time.Now(),
			CPUUsage:    0.0,
			MemoryUsage: 0.0,
			DiskUsage:   0.0,
			RobotID:     "",
			NodeStatus:  []NodeStatus{},
		},
	}
}

// GetMetricsHandler handles API requests for system metrics
func (s *DiagnosticService) GetMetricsHandler(c *fiber.Ctx) error {
	s.mu.RLock()
	defer s.mu.RUnlock()
	
	return c.JSON(fiber.Map{
		"status":  "success",
		"metrics": s.metrics,
	})
}

// UpdateMetrics updates the stored system metrics
func (s *DiagnosticService) UpdateMetrics(metrics SystemMetrics) {
	s.mu.Lock()
	defer s.mu.Unlock()
	
	s.metrics = metrics
	s.metrics.Timestamp = time.Now()
}

// GetMetrics returns the current system metrics
func (s *DiagnosticService) GetMetrics() SystemMetrics {
	s.mu.RLock()
	defer s.mu.RUnlock()
	
	return s.metrics
}

// StartMetricsListener starts listening for metrics from the ROS2 bridge
// This would be implemented to connect to the ROS2 diagnostic bridge
func (s *DiagnosticService) StartMetricsListener() error {
	// TODO: Implement connection to ROS2 diagnostic topics
	// This could use a message queue, WebSocket, or other IPC mechanism
	
	return nil
} 