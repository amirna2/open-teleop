package processing

import (
	"sync"
	"time"

	message "github.com/open-teleop/controller/pkg/flatbuffers/open_teleop/message"
	customlog "github.com/open-teleop/controller/pkg/log"
)

// ProcessResult is the result of processing a message
type ProcessResult struct {
	Topic     string
	Data      map[string]interface{}
	Timestamp int64
	Error     error
}

// ResultHandler is a function that handles processed results
type ResultHandler func(result *ProcessResult)

// MessageProcessor processes messages in a worker
type MessageProcessor func(ottMsg *message.OttMessage) (map[string]interface{}, error)

// ProcessingPool represents a priority-based worker pool
type ProcessingPool struct {
	name          string
	workerCount   int
	logger        customlog.Logger
	messageQueue  chan *message.OttMessage
	running       bool
	wg            sync.WaitGroup
	mu            sync.Mutex
	processor     MessageProcessor
	resultHandler ResultHandler
	queueSize     int
	metrics       *PoolMetrics
}

// PoolMetrics tracks metrics for a processing pool
type PoolMetrics struct {
	ProcessedCount    int64
	ErrorCount        int64
	QueuedCount       int64
	LastProcessedTime int64
	ProcessingTimeAvg int64 // in microseconds
	ProcessingTimeMax int64 // in microseconds
	mu                sync.Mutex
}

// NewProcessingPool creates a new processing pool
func NewProcessingPool(
	name string,
	workerCount int,
	queueSize int,
	logger customlog.Logger,
) *ProcessingPool {
	return &ProcessingPool{
		name:         name,
		workerCount:  workerCount,
		queueSize:    queueSize,
		logger:       logger,
		messageQueue: make(chan *message.OttMessage, queueSize),
		running:      false,
		wg:           sync.WaitGroup{},
		mu:           sync.Mutex{},
		metrics:      &PoolMetrics{mu: sync.Mutex{}},
	}
}

// SetProcessor sets the message processor function
func (p *ProcessingPool) SetProcessor(processor MessageProcessor) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.processor = processor
}

// SetResultHandler sets the result handler function
func (p *ProcessingPool) SetResultHandler(handler ResultHandler) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.resultHandler = handler
}

// ProcessMessage adds a message to the queue for processing
func (p *ProcessingPool) ProcessMessage(ottMsg *message.OttMessage) bool {
	p.mu.Lock()
	running := p.running
	p.mu.Unlock()

	if !running {
		p.logger.Warnf("%s pool not running, discarding message", p.name)
		return false
	}

	// Update metrics
	p.metrics.mu.Lock()
	p.metrics.QueuedCount++
	p.metrics.mu.Unlock()

	// Add message to queue (non-blocking if queue is full)
	select {
	case p.messageQueue <- ottMsg:
		// Message added to queue
		return true
	default:
		// Queue is full, log and discard
		p.logger.Warnf("%s pool queue is full, discarding message", p.name)
		return false
	}
}

// Start starts the processing pool workers
func (p *ProcessingPool) Start() {
	p.mu.Lock()
	defer p.mu.Unlock()

	if p.running {
		return
	}

	p.running = true
	p.logger.Infof("Starting %s priority pool with %d workers", p.name, p.workerCount)

	// Start workers
	for i := 0; i < p.workerCount; i++ {
		p.wg.Add(1)
		go p.worker(i)
	}
}

// Stop stops the processing pool
func (p *ProcessingPool) Stop() {
	p.mu.Lock()

	if !p.running {
		p.mu.Unlock()
		return
	}

	p.running = false
	p.mu.Unlock() // Unlock before closing channel to avoid deadlock

	close(p.messageQueue)

	p.logger.Infof("Stopping %s priority pool", p.name)

	// Wait for workers to finish
	p.wg.Wait()
	p.logger.Infof("%s priority pool stopped", p.name)

	// Log final metrics
	p.logMetrics()
}

// worker processes messages from the queue
func (p *ProcessingPool) worker(id int) {
	defer p.wg.Done()

	p.logger.Debugf("%s pool worker %d started", p.name, id)

	for ottMsg := range p.messageQueue {
		topic := string(ottMsg.Ott())
		p.logger.Debugf("%s pool worker %d processing message for topic %s", p.name, id, topic)

		// Process message using the processor function
		p.mu.Lock()
		processor := p.processor
		resultHandler := p.resultHandler
		p.mu.Unlock()

		if processor == nil {
			p.logger.Errorf("No message processor set for %s pool", p.name)
			continue
		}

		// Track processing time
		startTime := time.Now()

		// Process the message
		result, err := processor(ottMsg)

		// Calculate processing time
		processingTime := time.Since(startTime).Microseconds()

		// Update metrics
		p.metrics.mu.Lock()
		p.metrics.ProcessedCount++
		p.metrics.LastProcessedTime = time.Now().UnixNano()

		// Update processing time metrics
		if p.metrics.ProcessingTimeAvg == 0 {
			p.metrics.ProcessingTimeAvg = processingTime
		} else {
			// Simple moving average
			p.metrics.ProcessingTimeAvg = (p.metrics.ProcessingTimeAvg + processingTime) / 2
		}

		if processingTime > p.metrics.ProcessingTimeMax {
			p.metrics.ProcessingTimeMax = processingTime
		}

		if err != nil {
			p.metrics.ErrorCount++
		}
		p.metrics.mu.Unlock()

		// Create result
		processResult := &ProcessResult{
			Topic:     topic,
			Data:      result,
			Timestamp: ottMsg.TimestampNs(),
			Error:     err,
		}

		if err != nil {
			p.logger.Errorf("Error processing message in %s pool: %v", p.name, err)
		}

		// Handle result
		if resultHandler != nil {
			resultHandler(processResult)
		}
	}

	p.logger.Debugf("%s pool worker %d stopped", p.name, id)
}

// GetMetrics returns a copy of the current metrics
func (p *ProcessingPool) GetMetrics() PoolMetrics {
	p.metrics.mu.Lock()
	defer p.metrics.mu.Unlock()

	return PoolMetrics{
		ProcessedCount:    p.metrics.ProcessedCount,
		ErrorCount:        p.metrics.ErrorCount,
		QueuedCount:       p.metrics.QueuedCount,
		LastProcessedTime: p.metrics.LastProcessedTime,
		ProcessingTimeAvg: p.metrics.ProcessingTimeAvg,
		ProcessingTimeMax: p.metrics.ProcessingTimeMax,
	}
}

// logMetrics logs the current metrics
func (p *ProcessingPool) logMetrics() {
	metrics := p.GetMetrics()

	p.logger.Infof("%s pool metrics: processed=%d, errors=%d, avg_time=%dµs, max_time=%dµs",
		p.name, metrics.ProcessedCount, metrics.ErrorCount,
		metrics.ProcessingTimeAvg, metrics.ProcessingTimeMax)
}

// GetName returns the pool name
func (p *ProcessingPool) GetName() string {
	return p.name
}

// GetQueueLength returns the current length of the message queue
func (p *ProcessingPool) GetQueueLength() int {
	return len(p.messageQueue)
}

// GetQueueCapacity returns the capacity of the message queue
func (p *ProcessingPool) GetQueueCapacity() int {
	return p.queueSize
}
