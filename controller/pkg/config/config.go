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
	Version     string   `yaml:"version"`
	ConfigID    string   `yaml:"config_id"`
	LastUpdated string   `yaml:"lastUpdated"`
	Environment string   `yaml:"environment"`
	ZeroMQ      ZeroMQConfig   `yaml:"zeromq"`
	Server      ServerConfig   `yaml:"server"`
	Processing  ProcessingConfig `yaml:"processing"`
}

// ZeroMQConfig holds ZeroMQ-specific configuration
type ZeroMQConfig struct {
	ReceiverAddress  string `yaml:"receiver_address"`
	PublisherAddress string `yaml:"publisher_address"`
	MessageQueueSize int    `yaml:"message_queue_size"`
	TimeoutMs        int    `yaml:"timeout_ms"`
}

// ServerConfig holds HTTP server configuration
type ServerConfig struct {
	Port            int `yaml:"port"`
	RequestTimeout  int `yaml:"request_timeout"`
	MaxRequestSize  int `yaml:"max_request_size"`
}

// ProcessingConfig holds message processing configuration
type ProcessingConfig struct {
	HighPriorityWorkers    int `yaml:"high_priority_workers"`
	StandardPriorityWorkers int `yaml:"standard_priority_workers"`
	LowPriorityWorkers     int `yaml:"low_priority_workers"`
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
	// Default to development if no environment specified
	if environment == "" {
		// Check environment variable
		environment = os.Getenv("TELEOP_ENVIRONMENT")
		if environment == "" {
			environment = "development"
		}
	}

	// Normalize environment name
	environment = strings.ToLower(environment)

	// Validate environment
	switch environment {
	case "development", "testing", "production":
		// Valid environment
	default:
		return nil, fmt.Errorf("invalid environment: %s", environment)
	}

	// Build config file path
	configFile := fmt.Sprintf("controller_config_%s.yaml", environment)
	configPath := filepath.Join(configDir, configFile)

	// Check if the file exists
	if _, err := os.Stat(configPath); os.IsNotExist(err) {
		// Try the default config file
		configPath = filepath.Join(configDir, "controller_config.yaml")
		if _, err := os.Stat(configPath); os.IsNotExist(err) {
			return nil, fmt.Errorf("no configuration file found for environment: %s", environment)
		}
	}

	// Load the config
	config, err := LoadConfig(configPath)
	if err != nil {
		return nil, err
	}

	// Ensure the environment is set
	config.Environment = environment

	return config, nil
}

// applyEnvironmentOverrides applies environment variable overrides to the configuration
func applyEnvironmentOverrides(config *Config) {
	// ZeroMQ overrides
	if addr := os.Getenv("TELEOP_ZMQ_RECEIVER_ADDRESS"); addr != "" {
		config.ZeroMQ.ReceiverAddress = addr
	}
	
	if addr := os.Getenv("TELEOP_ZMQ_PUBLISHER_ADDRESS"); addr != "" {
		config.ZeroMQ.PublisherAddress = addr
	}
	
	if queueSize := os.Getenv("TELEOP_ZMQ_QUEUE_SIZE"); queueSize != "" {
		if size, err := strconv.Atoi(queueSize); err == nil {
			config.ZeroMQ.MessageQueueSize = size
		}
	}
	
	if timeout := os.Getenv("TELEOP_ZMQ_TIMEOUT_MS"); timeout != "" {
		if ms, err := strconv.Atoi(timeout); err == nil {
			config.ZeroMQ.TimeoutMs = ms
		}
	}
	
	// Server overrides
	if port := os.Getenv("TELEOP_SERVER_PORT"); port != "" {
		if p, err := strconv.Atoi(port); err == nil {
			config.Server.Port = p
		}
	}
	
	if timeout := os.Getenv("TELEOP_SERVER_REQUEST_TIMEOUT"); timeout != "" {
		if seconds, err := strconv.Atoi(timeout); err == nil {
			config.Server.RequestTimeout = seconds
		}
	}
	
	if maxSize := os.Getenv("TELEOP_SERVER_MAX_REQUEST_SIZE"); maxSize != "" {
		if size, err := strconv.Atoi(maxSize); err == nil {
			config.Server.MaxRequestSize = size
		}
	}
	
	// Processing overrides
	if workers := os.Getenv("TELEOP_HIGH_PRIORITY_WORKERS"); workers != "" {
		if count, err := strconv.Atoi(workers); err == nil {
			config.Processing.HighPriorityWorkers = count
		}
	}
	
	if workers := os.Getenv("TELEOP_STANDARD_PRIORITY_WORKERS"); workers != "" {
		if count, err := strconv.Atoi(workers); err == nil {
			config.Processing.StandardPriorityWorkers = count
		}
	}
	
	if workers := os.Getenv("TELEOP_LOW_PRIORITY_WORKERS"); workers != "" {
		if count, err := strconv.Atoi(workers); err == nil {
			config.Processing.LowPriorityWorkers = count
		}
	}
} 