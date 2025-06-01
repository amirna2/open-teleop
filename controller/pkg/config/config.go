package config

import (
	"fmt"
	"io/ioutil"

	"gopkg.in/yaml.v3"
)

// Config represents the controller configuration
type Config struct {
	Version       string           `yaml:"version" json:"version"`
	ConfigID      string           `yaml:"config_id" json:"config_id"`
	LastUpdated   string           `yaml:"lastUpdated" json:"lastUpdated"`
	RobotID       string           `yaml:"robot_id" json:"robot_id"`
	Controller    ControllerConfig `yaml:"controller" json:"controller"`
	ZeroMQ        ZeroMQConfig     `yaml:"zeromq" json:"zeromq"`
	TopicMappings []TopicMapping   `yaml:"topic_mappings" json:"topic_mappings"`
	MediaMappings MediaMappings    `yaml:"media_mappings" json:"media_mappings"`
	Defaults      DefaultsConfig   `yaml:"defaults" json:"defaults"`
	ThrottleRates ThrottleConfig   `yaml:"throttle_rates" json:"throttle_rates"`
}

// ControllerConfig represents controller-specific configuration
type ControllerConfig struct {
	Server ServerConfig `yaml:"server" json:"server"`
	// Processing ProcessingConfig `yaml:"processing" json:"processing"` // Moved to BootstrapConfig
}

// ZeroMQConfig holds ZeroMQ-specific configuration
type ZeroMQConfig struct {
	ControllerAddress       string `yaml:"controller_address" json:"controller_address"`
	GatewayAddress          string `yaml:"gateway_address" json:"gateway_address"`
	GatewayConnectAddress   string `yaml:"gateway_connect_address" json:"gateway_connect_address"`
	GatewaySubscribeAddress string `yaml:"gateway_subscribe_address" json:"gateway_subscribe_address"`
	MessageBufferSize       int    `yaml:"message_buffer_size" json:"message_buffer_size"`
	ReconnectIntervalMs     int    `yaml:"reconnect_interval_ms" json:"reconnect_interval_ms"`
}

// ServerConfig holds HTTP server configuration
type ServerConfig struct {
	Port           int `yaml:"port" json:"port"`
	RequestTimeout int `yaml:"request_timeout" json:"request_timeout"`
	MaxRequestSize int `yaml:"max_request_size" json:"max_request_size"`
}

// TopicMapping represents a mapping between ROS topics and Open-Teleop topics
type TopicMapping struct {
	TopicID       string                 `yaml:"topic_id" json:"topic_id"`
	RosTopic      string                 `yaml:"ros_topic" json:"ros_topic"`
	OttTopic      string                 `yaml:"ott" json:"ott"`
	MessageType   string                 `yaml:"message_type" json:"message_type"`
	Priority      string                 `yaml:"priority" json:"priority"`
	Direction     string                 `yaml:"direction" json:"direction"`
	SourceType    string                 `yaml:"source_type" json:"source_type"`
	EncoderParams map[string]interface{} `yaml:"encoder_params,omitempty" json:"encoder_params,omitempty"`
}

// MediaMappings represents the media gateway configuration sections
type MediaMappings struct {
	Video []VideoMapping `yaml:"video" json:"video"`
	Audio []AudioMapping `yaml:"audio" json:"audio"`
}

// VideoMapping represents a video stream configuration for media gateway
type VideoMapping struct {
	TopicID        string                 `yaml:"topic_id" json:"topic_id"`
	DeviceID       string                 `yaml:"device_id" json:"device_id"`
	OttTopic       string                 `yaml:"ott" json:"ott"`
	EncodingFormat string                 `yaml:"encoding_format" json:"encoding_format"`
	Priority       string                 `yaml:"priority" json:"priority"`
	EncoderParams  map[string]interface{} `yaml:"encoder_params,omitempty" json:"encoder_params,omitempty"`
}

// AudioMapping represents an audio stream configuration for media gateway
type AudioMapping struct {
	TopicID        string                 `yaml:"topic_id" json:"topic_id"`
	DeviceID       string                 `yaml:"device_id" json:"device_id"`
	OttTopic       string                 `yaml:"ott" json:"ott"`
	EncodingFormat string                 `yaml:"encoding_format" json:"encoding_format"`
	Priority       string                 `yaml:"priority" json:"priority"`
	EncoderParams  map[string]interface{} `yaml:"encoder_params,omitempty" json:"encoder_params,omitempty"`
}

// DefaultsConfig holds default values for topic mappings
type DefaultsConfig struct {
	Priority   string `yaml:"priority" json:"priority"`
	Direction  string `yaml:"direction" json:"direction"`
	SourceType string `yaml:"source_type" json:"source_type"`
}

// ThrottleConfig holds throttling configuration for different priority levels
type ThrottleConfig struct {
	HighHz     int `yaml:"high_hz" json:"high_hz"`
	StandardHz int `yaml:"standard_hz" json:"standard_hz"`
	LowHz      int `yaml:"low_hz" json:"low_hz"`
}

// LoadConfig loads configuration from the specified file path
// and applies environment variable overrides
func LoadConfig(path string) (*Config, error) {
	// Read the config file
	data, err := ioutil.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("error reading config file: %w", err)
	}

	// Parse the YAML
	var config Config
	if err := yaml.Unmarshal(data, &config); err != nil {
		return nil, fmt.Errorf("error parsing config file: %w", err)
	}

	return &config, nil
}

// GetTopicMappingsByDirection returns topic mappings filtered by direction
func (c *Config) GetTopicMappingsByDirection(direction string) []TopicMapping {
	var result []TopicMapping

	for _, mapping := range c.TopicMappings {
		// If mapping doesn't have direction, use default
		mappingDirection := mapping.Direction
		if mappingDirection == "" {
			mappingDirection = c.Defaults.Direction
		}

		if mappingDirection == direction {
			// Create a copy with defaults applied
			mappingWithDefaults := applyDefaults(mapping, c.Defaults)
			result = append(result, mappingWithDefaults)
		}
	}

	return result
}

// GetTopicMappingByOttTopic returns a topic mapping for a specific Open-Teleop topic
func (c *Config) GetTopicMappingByOttTopic(ottTopic string) (TopicMapping, bool) {
	for _, mapping := range c.TopicMappings {
		if mapping.OttTopic == ottTopic {
			// Apply defaults
			return applyDefaults(mapping, c.Defaults), true
		}
	}

	return TopicMapping{}, false
}

// GetVideoMappingByOttTopic returns a video mapping for a specific Open-Teleop topic
func (c *Config) GetVideoMappingByOttTopic(ottTopic string) (VideoMapping, bool) {
	for _, mapping := range c.MediaMappings.Video {
		if mapping.OttTopic == ottTopic {
			return mapping, true
		}
	}
	return VideoMapping{}, false
}

// GetAudioMappingByOttTopic returns an audio mapping for a specific Open-Teleop topic
func (c *Config) GetAudioMappingByOttTopic(ottTopic string) (AudioMapping, bool) {
	for _, mapping := range c.MediaMappings.Audio {
		if mapping.OttTopic == ottTopic {
			return mapping, true
		}
	}
	return AudioMapping{}, false
}

// GetVideoMappingByDeviceID returns a video mapping for a specific device ID
func (c *Config) GetVideoMappingByDeviceID(deviceID string) (VideoMapping, bool) {
	for _, mapping := range c.MediaMappings.Video {
		if mapping.DeviceID == deviceID {
			return mapping, true
		}
	}
	return VideoMapping{}, false
}

// GetAudioMappingByDeviceID returns an audio mapping for a specific device ID
func (c *Config) GetAudioMappingByDeviceID(deviceID string) (AudioMapping, bool) {
	for _, mapping := range c.MediaMappings.Audio {
		if mapping.DeviceID == deviceID {
			return mapping, true
		}
	}
	return AudioMapping{}, false
}

// applyDefaults merges default values into a topic mapping where fields are empty
func applyDefaults(mapping TopicMapping, defaults DefaultsConfig) TopicMapping {
	result := mapping

	// Apply defaults for empty fields
	if result.Priority == "" {
		result.Priority = defaults.Priority
	}

	if result.Direction == "" {
		result.Direction = defaults.Direction
	}

	if result.SourceType == "" {
		result.SourceType = defaults.SourceType
	}

	return result
}
