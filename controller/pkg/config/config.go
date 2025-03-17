package config

import (
	"fmt"
	"io/ioutil"
	"os"
	"path/filepath"
	"strconv"
	"strings"

	"gopkg.in/yaml.v3"
)

// Config represents the controller configuration
type Config struct {
	Version      string           `yaml:"version"`
	ConfigID     string           `yaml:"config_id"`
	LastUpdated  string           `yaml:"lastUpdated"`
	Environment  string           `yaml:"environment"`
	RobotID      string           `yaml:"robot_id"`
	Controller   ControllerConfig `yaml:"controller"`
	ZeroMQ       ZeroMQConfig     `yaml:"zeromq"`
	TopicMappings []TopicMapping   `yaml:"topic_mappings"`
	Defaults     DefaultsConfig   `yaml:"defaults"`
	ThrottleRates ThrottleConfig  `yaml:"throttle_rates"`
}

// ControllerConfig represents controller-specific configuration
type ControllerConfig struct {
	Server     ServerConfig     `yaml:"server"`
	Processing ProcessingConfig `yaml:"processing"`
}

// ZeroMQConfig holds ZeroMQ-specific configuration
type ZeroMQConfig struct {
	ControllerAddress    string `yaml:"controller_address"`
	GatewayAddress       string `yaml:"gateway_address"`
	GatewayConnectAddress string `yaml:"gateway_connect_address"`
	GatewaySubscribeAddress string `yaml:"gateway_subscribe_address"`
	MessageBufferSize   int    `yaml:"message_buffer_size"`
	ReconnectIntervalMs int    `yaml:"reconnect_interval_ms"`
}

// ServerConfig holds HTTP server configuration
type ServerConfig struct {
	Port           int `yaml:"port"`
	RequestTimeout int `yaml:"request_timeout"`
	MaxRequestSize int `yaml:"max_request_size"`
}

// ProcessingConfig holds message processing configuration
type ProcessingConfig struct {
	HighPriorityWorkers     int `yaml:"high_priority_workers"`
	StandardPriorityWorkers int `yaml:"standard_priority_workers"`
	LowPriorityWorkers      int `yaml:"low_priority_workers"`
}

// TopicMapping represents a mapping between ROS topics and Open-Teleop topics
type TopicMapping struct {
	RosTopic    string `yaml:"ros_topic"`
	OttTopic    string `yaml:"ott"`
	MessageType string `yaml:"message_type"`
	Priority    string `yaml:"priority"`
	Direction   string `yaml:"direction"`
	SourceType  string `yaml:"source_type"`
}

// DefaultsConfig holds default values for topic mappings
type DefaultsConfig struct {
	Priority   string `yaml:"priority"`
	Direction  string `yaml:"direction"`
	SourceType string `yaml:"source_type"`
}

// ThrottleConfig holds throttling configuration for different priority levels
type ThrottleConfig struct {
	HighHz     int `yaml:"high_hz"`
	StandardHz int `yaml:"standard_hz"`
	LowHz      int `yaml:"low_hz"`
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
	applyEnvironmentOverrides(&config)

	return &config, nil
}

// LoadConfigWithEnv loads configuration based on the environment
// Options are: development, testing, production
func LoadConfigWithEnv(configDir string, environment string) (*Config, error) {
	// Load the unified configuration
	unifiedConfigPath := filepath.Join(configDir, "open_teleop_config.yaml")
	if _, err := os.Stat(unifiedConfigPath); err != nil {
		return nil, fmt.Errorf("unified config file not found at %s: %w", unifiedConfigPath, err)
	}
	
	config, err := LoadConfig(unifiedConfigPath)
	if err != nil {
		return nil, fmt.Errorf("error loading unified config: %w", err)
	}
	
	// Set environment if specified
	if environment != "" {
		config.Environment = environment
	} else if config.Environment == "" {
		// Check environment variable
		config.Environment = os.Getenv("TELEOP_ENVIRONMENT")
		if config.Environment == "" {
			config.Environment = "development"
		}
	}
	
	// Normalize environment name
	config.Environment = strings.ToLower(config.Environment)
	
	// Validate environment
	switch config.Environment {
	case "development", "testing", "production":
		// Valid environment
	default:
		return nil, fmt.Errorf("invalid environment: %s", config.Environment)
	}
	
	return config, nil
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

// applyEnvironmentOverrides applies environment variable overrides to the configuration
func applyEnvironmentOverrides(config *Config) {
	// ZeroMQ overrides
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
	
	// Processing overrides
	if workers := os.Getenv("TELEOP_HIGH_PRIORITY_WORKERS"); workers != "" {
		if count, err := strconv.Atoi(workers); err == nil {
			config.Controller.Processing.HighPriorityWorkers = count
		}
	}
	
	if workers := os.Getenv("TELEOP_STANDARD_PRIORITY_WORKERS"); workers != "" {
		if count, err := strconv.Atoi(workers); err == nil {
			config.Controller.Processing.StandardPriorityWorkers = count
		}
	}
	
	if workers := os.Getenv("TELEOP_LOW_PRIORITY_WORKERS"); workers != "" {
		if count, err := strconv.Atoi(workers); err == nil {
			config.Controller.Processing.LowPriorityWorkers = count
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