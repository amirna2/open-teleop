package zeromq

import (
	"log"
	"time"

	zmq "github.com/pebbe/zmq4"
	"github.com/open-teleop/controller/domain/diagnostic"
	fb "github.com/open-teleop/controller/pkg/flatbuffers/open_teleop/diagnostic"
)

// MetricsListener listens for metrics updates from ROS2 bridges
type MetricsListener struct {
	socket      *zmq.Socket
	diagService *diagnostic.DiagnosticService
	running     bool
}

// NewMetricsListener creates a new ZeroMQ metrics listener
func NewMetricsListener(diagService *diagnostic.DiagnosticService) (*MetricsListener, error) {
	socket, err := zmq.NewSocket(zmq.SUB)
	if err != nil {
		return nil, err
	}
	
	// Subscribe to all messages
	err = socket.SetSubscribe("")
	if err != nil {
		socket.Close()
		return nil, err
	}
	
	return &MetricsListener{
		socket:      socket,
		diagService: diagService,
		running:     false,
	}, nil
}

// Start begins listening for metrics
func (l *MetricsListener) Start(address string) error {
	err := l.socket.Bind(address)
	if err != nil {
		return err
	}
	
	l.running = true
	go l.receiveLoop()
	
	log.Printf("Metrics listener started on %s", address)
	return nil
}

// Stop stops the metrics listener
func (l *MetricsListener) Stop() {
	l.running = false
	l.socket.Close()
}

// receiveLoop continuously receives and processes metrics
func (l *MetricsListener) receiveLoop() {
	for l.running {
		msg, err := l.socket.RecvBytes(0)
		if err != nil {
			log.Printf("Error receiving message: %v", err)
			time.Sleep(100 * time.Millisecond)
			continue
		}
		
		// Parse FlatBuffer message
		metrics := fb.GetRootAsSystemMetrics(msg, 0)
		
		// Convert to domain model
		sysMetrics := diagnostic.SystemMetrics{
			Timestamp:   time.Unix(0, metrics.Timestamp()*int64(time.Millisecond)),
			CPUUsage:    metrics.CpuUsage(),
			MemoryUsage: metrics.Memory(nil).UsedPercent(),
			DiskUsage:   metrics.DiskUsage(),
			RobotID:     string(metrics.RobotId()),
			NodeStatus:  make([]diagnostic.NodeStatus, metrics.NodeStatusLength()),
		}
		
		// Extract node statuses if present
		for i := 0; i < metrics.NodeStatusLength(); i++ {
			var ns fb.NodeStatus
			if metrics.NodeStatus(&ns, i) {
				sysMetrics.NodeStatus[i] = diagnostic.NodeStatus{
					Name:   string(ns.Name()),
					Status: string(ns.Status()),
					PID:    int(ns.Pid()),
				}
			}
		}
		
		// Update service with new metrics
		l.diagService.UpdateMetrics(sysMetrics)
		
		log.Printf("Received metrics from robot %s: CPU=%v%%, Memory=%v%%, Disk=%v%%",
			sysMetrics.RobotID, sysMetrics.CPUUsage, sysMetrics.MemoryUsage, sysMetrics.DiskUsage)
	}
} 