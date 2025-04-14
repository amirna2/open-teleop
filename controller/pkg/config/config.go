package config

import (
	"fmt"
	"io/ioutil"
	"os"
	"strconv"

	"gopkg.in/yaml.v3"
)

// Config represents the controller configuration
type Config struct {
	Version       string           `yaml:"version" json:"version"`
	ConfigID      string           `yaml:"config_id" json:"config_id"`
	LastUpdated   string           `yaml:"lastUpdated" json:"lastUpdated"`
	Environment   string           `yaml:"environment" json:"environment"`
	RobotID       string           `yaml:"robot_id" json:"robot_id"`
	Controller    ControllerConfig `yaml:"controller" json:"controller"`
	ZeroMQ        ZeroMQConfig     `yaml:"zeromq" json:"zeromq"`
	TopicMappings []TopicMapping   `yaml:"topic_mappings" json:"topic_mappings"`
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
	RosTopic    string `yaml:"ros_topic" json:"ros_topic"`
	OttTopic    string `yaml:"ott" json:"ott"`
	MessageType string `yaml:"message_type" json:"message_type"`
	Priority    string `yaml:"priority" json:"priority"`
	Direction   string `yaml:"direction" json:"direction"`
	SourceType  string `yaml:"source_type" json:"source_type"`
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

	// Apply environment variable overrides
	ApplyEnvironmentOverrides(&config)

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

// ApplyEnvironmentOverrides applies environment variable overrides to the configuration
// Exported function to be called manually after loading config.
func ApplyEnvironmentOverrides(config *Config) {
	// ZeroMQ overrides (These apply to the operational config struct)
	if addr := os.Getenv("TELEOP_ZMQ_CONTROLLER_ADDRESS"); addr != "" {
		config.ZeroMQ.ControllerAddress = addr
	}

	if addr := os.Getenv("TELEOP_ZMQ_GATEWAY_ADDRESS"); addr != "" {
		config.ZeroMQ.GatewayAddress = addr
	}

	if addr := os.Getenv("TELEOP_ZMQ_GATEWAY_CONNECT_ADDRESS"); addr != "" {
		config.ZeroMQ.GatewayConnectAddress = addr
	}

	if addr := os.Getenv("TELEOP_ZMQ_GATEWAY_SUBSCRIBE_ADDRESS"); addr != "" {
		config.ZeroMQ.GatewaySubscribeAddress = addr
	}

	if bufferSize := os.Getenv("TELEOP_ZMQ_BUFFER_SIZE"); bufferSize != "" {
		if size, err := strconv.Atoi(bufferSize); err == nil {
			config.ZeroMQ.MessageBufferSize = size
		}
	}

	if interval := os.Getenv("TELEOP_ZMQ_RECONNECT_INTERVAL_MS"); interval != "" {
		if ms, err := strconv.Atoi(interval); err == nil {
			config.ZeroMQ.ReconnectIntervalMs = ms
		}
	}

	// Server overrides
	if port := os.Getenv("TELEOP_SERVER_PORT"); port != "" {
		if p, err := strconv.Atoi(port); err == nil {
			config.Controller.Server.Port = p
		}
	}

	if timeout := os.Getenv("TELEOP_SERVER_REQUEST_TIMEOUT"); timeout != "" {
		if seconds, err := strconv.Atoi(timeout); err == nil {
			config.Controller.Server.RequestTimeout = seconds
		}
	}

	if maxSize := os.Getenv("TELEOP_SERVER_MAX_REQUEST_SIZE"); maxSize != "" {
		if size, err := strconv.Atoi(maxSize); err == nil {
			config.Controller.Server.MaxRequestSize = size
		}
	}

	// Throttle rates overrides
	if rate := os.Getenv("TELEOP_THROTTLE_HIGH_HZ"); rate != "" {
		if hz, err := strconv.Atoi(rate); err == nil {
			config.ThrottleRates.HighHz = hz
		}
	}

	if rate := os.Getenv("TELEOP_THROTTLE_STANDARD_HZ"); rate != "" {
		if hz, err := strconv.Atoi(rate); err == nil {
			config.ThrottleRates.StandardHz = hz
		}
	}

	if rate := os.Getenv("TELEOP_THROTTLE_LOW_HZ"); rate != "" {
		if hz, err := strconv.Atoi(rate); err == nil {
			config.ThrottleRates.LowHz = hz
		}
	}
}
