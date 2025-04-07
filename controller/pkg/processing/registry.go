package processing

import (
	"sync"

	"github.com/open-teleop/controller/pkg/config"
	customlog "github.com/open-teleop/controller/pkg/log"
)

// TopicInfo holds metadata for a topic
type TopicInfo struct {
	OttTopic     string
	MessageType  string
	Priority     string
	Direction    string
	StatCount    int64
	LastReceived int64
}

// TopicRegistry maintains information about topics
type TopicRegistry struct {
	logger customlog.Logger
	topics map[string]*TopicInfo
	mu     sync.RWMutex
}

// NewTopicRegistry creates a new topic registry
func NewTopicRegistry(logger customlog.Logger) *TopicRegistry {
	return &TopicRegistry{
		logger: logger,
		topics: make(map[string]*TopicInfo),
		mu:     sync.RWMutex{},
	}
}

// LoadFromConfig loads topic information from the config
func (r *TopicRegistry) LoadFromConfig(cfg *config.Config) {
	r.mu.Lock()
	defer r.mu.Unlock()

	// Clear existing topics
	r.topics = make(map[string]*TopicInfo)

	// Load topics from config
	for _, mapping := range cfg.TopicMappings {
		// Apply defaults if needed
		priority := mapping.Priority
		if priority == "" {
			priority = cfg.Defaults.Priority
		}

		r.topics[mapping.OttTopic] = &TopicInfo{
			OttTopic:     mapping.OttTopic,
			MessageType:  mapping.MessageType,
			Priority:     priority,
			Direction:    mapping.Direction,
			StatCount:    0,
			LastReceived: 0,
		}
	}

	r.logger.Infof("Loaded %d topics into registry", len(r.topics))
}

// GetTopicPriority gets the priority for a topic
func (r *TopicRegistry) GetTopicPriority(topic string) (string, bool) {
	r.mu.RLock()
	defer r.mu.RUnlock()

	info, exists := r.topics[topic]
	if !exists {
		return "", false
	}

	return info.Priority, true
}

// GetTopicInfo gets information for a topic
func (r *TopicRegistry) GetTopicInfo(topic string) (*TopicInfo, bool) {
	r.mu.RLock()
	defer r.mu.RUnlock()

	info, exists := r.topics[topic]
	if !exists {
		return nil, false
	}

	// Return a copy to avoid race conditions
	infoCopy := &TopicInfo{
		OttTopic:     info.OttTopic,
		MessageType:  info.MessageType,
		Priority:     info.Priority,
		Direction:    info.Direction,
		StatCount:    info.StatCount,
		LastReceived: info.LastReceived,
	}

	return infoCopy, true
}

// UpdateTopicStats updates statistics for a topic
func (r *TopicRegistry) UpdateTopicStats(topic string, timestamp int64) {
	r.mu.Lock()
	defer r.mu.Unlock()

	info, exists := r.topics[topic]
	if !exists {
		// Create new topic if it doesn't exist
		info = &TopicInfo{
			OttTopic:     topic,
			Priority:     "STANDARD", // Default priority
			StatCount:    0,
			LastReceived: 0,
		}
		r.topics[topic] = info
	}

	// Update stats
	info.StatCount++
	info.LastReceived = timestamp
}

// GetMessageType gets the message type for a topic
func (r *TopicRegistry) GetMessageType(topic string) (string, bool) {
	r.mu.RLock()
	defer r.mu.RUnlock()

	info, exists := r.topics[topic]
	if !exists {
		return "", false
	}

	return info.MessageType, true
}

// GetAllTopics returns a list of all registered topics
func (r *TopicRegistry) GetAllTopics() []string {
	r.mu.RLock()
	defer r.mu.RUnlock()

	topics := make([]string, 0, len(r.topics))
	for topic := range r.topics {
		topics = append(topics, topic)
	}

	return topics
}

// GetTopicStats returns a map of topic statistics
func (r *TopicRegistry) GetTopicStats() map[string]map[string]interface{} {
	r.mu.RLock()
	defer r.mu.RUnlock()

	stats := make(map[string]map[string]interface{})

	for topic, info := range r.topics {
		stats[topic] = map[string]interface{}{
			"count":         info.StatCount,
			"last_received": info.LastReceived,
			"type":          info.MessageType,
			"priority":      info.Priority,
			"direction":     info.Direction,
		}
	}

	return stats
}
