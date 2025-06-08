package processing

import (
	"fmt"
	"sync"
	"time"

	"github.com/open-teleop/controller/domain/video"
	"github.com/open-teleop/controller/pkg/config"
	message "github.com/open-teleop/controller/pkg/flatbuffers/open_teleop/message"
	customlog "github.com/open-teleop/controller/pkg/log"
)

// Message represents a message to be processed
type Message struct {
	Topic     string
	Data      []byte
	Timestamp int64
}

// GetCurrentTimestamp gets the current timestamp in nanoseconds
func GetCurrentTimestamp() int64 {
	return time.Now().UnixNano()
}

// Constants for priority levels
const (
	PriorityHigh     = "HIGH"
	PriorityStandard = "STANDARD"
	PriorityLow      = "LOW"
)

// MessageDirector routes messages to the appropriate processing pool based on priority
type MessageDirector struct {
	logger           customlog.Logger
	highPriorityPool *ProcessingPool
	standardPool     *ProcessingPool
	lowPriorityPool  *ProcessingPool
	topicRegistry    *TopicRegistry
	processor        MessageProcessor
	resultHandler    ResultHandler
	running          bool
	mu               sync.RWMutex
	videoService     *video.VideoService

	// Default settings
	defaultQueueSize int
}

// DirectorOptions holds configuration options for the MessageDirector
type DirectorOptions struct {
	DefaultQueueSize int
}

// NewMessageDirector creates a new message director
func NewMessageDirector(
	cfg *config.Config,
	logger customlog.Logger,
	topicRegistry *TopicRegistry,
	options *DirectorOptions,
) *MessageDirector {
	// Set default options if not provided
	if options == nil {
		options = &DirectorOptions{
			DefaultQueueSize: 100, // Default queue size
		}
	}

	return &MessageDirector{
		logger:           logger,
		topicRegistry:    topicRegistry,
		defaultQueueSize: options.DefaultQueueSize,
		running:          false,
		mu:               sync.RWMutex{},
	}
}

// Initialize creates the processing pools based on the provided worker counts
func (d *MessageDirector) Initialize(highWorkers, standardWorkers, lowWorkers int) {
	d.mu.Lock()
	defer d.mu.Unlock()

	// Create processing pools
	d.highPriorityPool = NewProcessingPool(
		"HIGH",
		highWorkers,
		d.defaultQueueSize,
		d.logger,
	)

	d.standardPool = NewProcessingPool(
		"STANDARD",
		standardWorkers,
		d.defaultQueueSize,
		d.logger,
	)

	d.lowPriorityPool = NewProcessingPool(
		"LOW",
		lowWorkers,
		d.defaultQueueSize,
		d.logger,
	)

	d.logger.Infof("Message Director initialized with pools: HIGH(%d), STANDARD(%d), LOW(%d)",
		highWorkers, standardWorkers, lowWorkers)
}

// SetProcessor sets the message processor function for all pools
func (d *MessageDirector) SetProcessor(processor MessageProcessor) {
	d.mu.Lock()
	defer d.mu.Unlock()

	d.processor = processor

	// Set processor for each pool
	if d.highPriorityPool != nil {
		// Use high-priority processor if VideoService is available, otherwise use the provided processor
		if d.videoService != nil {
			d.logger.Debugf("Setting high-priority processor with VideoService for HIGH pool")
			highPriorityProcessor := NewHighPriorityProcessor(d.videoService, processor)
			d.highPriorityPool.SetProcessor(highPriorityProcessor)
		} else {
			d.logger.Debugf("VideoService not available, using standard processor for HIGH pool")
			d.highPriorityPool.SetProcessor(processor)
		}
	}

	if d.standardPool != nil {
		d.standardPool.SetProcessor(processor)
	}

	if d.lowPriorityPool != nil {
		d.lowPriorityPool.SetProcessor(processor)
	}
}

// SetResultHandler sets the result handler function for all pools
func (d *MessageDirector) SetResultHandler(handler ResultHandler) {
	d.mu.Lock()
	defer d.mu.Unlock()

	d.resultHandler = handler

	// Set handler for each pool
	if d.highPriorityPool != nil {
		d.highPriorityPool.SetResultHandler(handler)
	}

	if d.standardPool != nil {
		d.standardPool.SetResultHandler(handler)
	}

	if d.lowPriorityPool != nil {
		d.lowPriorityPool.SetResultHandler(handler)
	}
}

// SetVideoService sets the VideoService for the director
func (d *MessageDirector) SetVideoService(service *video.VideoService) {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.videoService = service
}

// GetVideoService gets the VideoService from the director
func (d *MessageDirector) GetVideoService() *video.VideoService {
	d.mu.RLock()
	defer d.mu.RUnlock()
	return d.videoService
}

// RouteMessage routes a message to the appropriate processing pool based on its priority
func (d *MessageDirector) RouteMessage(ottMsg *message.OttMessage) error {
	d.mu.Lock()
	running := d.running
	d.mu.Unlock()

	if !running {
		return fmt.Errorf("message director is not running")
	}

	topic := string(ottMsg.Ott())

	// Step 1: Get topic priority from registry.
	// TODO: This is unnecessarily complex. We should assume all messages have a priority.
	// I believe we currently allow a topic_mapping to have no priority and then use the config default.
	// It's better to be explicit and require a priority for all topics.
	priority, exists := d.topicRegistry.GetTopicPriority(topic)
	if !exists {
		// For video frames, always use HIGH priority
		if ottMsg.ContentType() == message.ContentTypeENCODED_VIDEO_FRAME {
			priority = PriorityHigh
			d.logger.Debugf("Setting HIGH priority for ENCODED_VIDEO_FRAME message (topic: %s)", topic)
		} else {
			d.logger.Warnf("No priority found for topic '%s', using STANDARD", topic)
			priority = PriorityStandard
		}

		// Register the topic with determined priority
		d.topicRegistry.UpdateTopicStats(topic, ottMsg.TimestampNs())
	} else {
		// Update stats
		d.topicRegistry.UpdateTopicStats(topic, ottMsg.TimestampNs())
	}

	// Step 2: Route to appropriate pool
	var successful bool
	switch priority {
	case PriorityHigh:
		d.logger.Debugf("Routing message for topic '%s' to HIGH priority pool", topic)
		successful = d.highPriorityPool.ProcessMessage(ottMsg)
	case PriorityLow:
		d.logger.Debugf("Routing message for topic '%s' to LOW priority pool", topic)
		successful = d.lowPriorityPool.ProcessMessage(ottMsg)
	default:
		d.logger.Debugf("Routing message for topic '%s' to STANDARD priority pool", topic)
		successful = d.standardPool.ProcessMessage(ottMsg)
	}

	if !successful {
		return fmt.Errorf("failed to enqueue message for topic '%s' (priority: %s)", topic, priority)
	}

	return nil
}

// Start starts all processing pools
func (d *MessageDirector) Start() {
	d.mu.Lock()
	defer d.mu.Unlock()

	if d.running {
		return
	}

	d.running = true
	d.logger.Infof("Starting Message Director")

	// Start the pools
	d.highPriorityPool.Start()
	d.standardPool.Start()
	d.lowPriorityPool.Start()
}

// Stop stops all processing pools
func (d *MessageDirector) Stop() {
	d.mu.Lock()
	running := d.running
	d.running = false
	d.mu.Unlock()

	if !running {
		return
	}

	d.logger.Infof("Stopping Message Director")

	// Stop the pools
	d.highPriorityPool.Stop()
	d.standardPool.Stop()
	d.lowPriorityPool.Stop()

	d.logger.Infof("Message Director stopped")
}

// GetPoolMetrics returns metrics for all pools
func (d *MessageDirector) GetPoolMetrics() map[string]PoolMetrics {
	d.mu.Lock()
	defer d.mu.Unlock()

	metrics := make(map[string]PoolMetrics)

	if d.highPriorityPool != nil {
		metrics[PriorityHigh] = d.highPriorityPool.GetMetrics()
	}

	if d.standardPool != nil {
		metrics[PriorityStandard] = d.standardPool.GetMetrics()
	}

	if d.lowPriorityPool != nil {
		metrics[PriorityLow] = d.lowPriorityPool.GetMetrics()
	}

	return metrics
}

// EnqueueMessage enqueues a message to process
func (d *MessageDirector) EnqueueMessage(msg *Message) bool {
	d.mu.Lock()
	running := d.running
	d.mu.Unlock()

	if !running {
		d.logger.Warnf("Message director is not running, cannot enqueue message for topic '%s'", msg.Topic)
		return false
	}

	// Get topic priority from registry - skip using the priority variable for now
	_, exists := d.topicRegistry.GetTopicPriority(msg.Topic)
	if !exists {
		d.logger.Warnf("No priority found for topic '%s', using STANDARD", msg.Topic)
		// Register with standard priority in future implementation
	}

	// Convert to OttMessage (placeholder for now)
	// In a real implementation, this would contain actual message data
	// Currently just use empty flatbuffer to route to right pool

	// Update stats in registry
	d.topicRegistry.UpdateTopicStats(msg.Topic, msg.Timestamp)

	// TODO: Create proper OttMessage wrapper around raw data
	// For now, just log that we received a message
	d.logger.Debugf("Received raw message for topic '%s' (%d bytes)", msg.Topic, len(msg.Data))

	// Placeholder for successful processing
	return true
}
