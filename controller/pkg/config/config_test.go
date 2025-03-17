package config

import (
	"io/ioutil"
	"os"
	"path/filepath"
	"testing"
)

func TestLoadConfig(t *testing.T) {
	// Create a temporary test config file
	tempDir, err := ioutil.TempDir("", "config-test")
	if err != nil {
		t.Fatalf("Failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tempDir)

	configContent := `
version: "1.0"
config_id: "test-config"
lastUpdated: "2025-03-16T10:00:00Z"
environment: "testing"

zeromq:
  receiver_address: "tcp://localhost:5555"
  publisher_address: "tcp://localhost:5556"
  message_queue_size: 1000
  timeout_ms: 1000

server:
  port: 8080
  request_timeout: 30
  max_request_size: 10

processing:
  high_priority_workers: 4
  standard_priority_workers: 2
  low_priority_workers: 1
`

	configPath := filepath.Join(tempDir, "test_config.yaml")
	if err := ioutil.WriteFile(configPath, []byte(configContent), 0644); err != nil {
		t.Fatalf("Failed to write test config: %v", err)
	}

	// Test loading the config
	config, err := LoadConfig(configPath)
	if err != nil {
		t.Fatalf("LoadConfig failed: %v", err)
	}

	// Verify config values
	if config.Version != "1.0" {
		t.Errorf("Expected version 1.0, got %s", config.Version)
	}

	if config.ConfigID != "test-config" {
		t.Errorf("Expected config_id test-config, got %s", config.ConfigID)
	}

	if config.ZeroMQ.ReceiverAddress != "tcp://localhost:5555" {
		t.Errorf("Expected receiver_address tcp://localhost:5555, got %s", config.ZeroMQ.ReceiverAddress)
	}

	if config.Server.Port != 8080 {
		t.Errorf("Expected port 8080, got %d", config.Server.Port)
	}

	if config.Processing.HighPriorityWorkers != 4 {
		t.Errorf("Expected high_priority_workers 4, got %d", config.Processing.HighPriorityWorkers)
	}
}

func TestEnvironmentOverrides(t *testing.T) {
	// Create a temporary test config file
	tempDir, err := ioutil.TempDir("", "config-test")
	if err != nil {
		t.Fatalf("Failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tempDir)

	configContent := `
version: "1.0"
config_id: "test-config"
lastUpdated: "2025-03-16T10:00:00Z"
environment: "testing"

zeromq:
  receiver_address: "tcp://localhost:5555"
  publisher_address: "tcp://localhost:5556"
  message_queue_size: 1000
  timeout_ms: 1000

server:
  port: 8080
  request_timeout: 30
  max_request_size: 10

processing:
  high_priority_workers: 4
  standard_priority_workers: 2
  low_priority_workers: 1
`

	configPath := filepath.Join(tempDir, "test_config.yaml")
	if err := ioutil.WriteFile(configPath, []byte(configContent), 0644); err != nil {
		t.Fatalf("Failed to write test config: %v", err)
	}

	// Set environment variables
	os.Setenv("TELEOP_ZMQ_RECEIVER_ADDRESS", "tcp://override:5555")
	os.Setenv("TELEOP_SERVER_PORT", "9090")
	os.Setenv("TELEOP_HIGH_PRIORITY_WORKERS", "8")

	// Test loading the config with environment overrides
	config, err := LoadConfig(configPath)
	if err != nil {
		t.Fatalf("LoadConfig failed: %v", err)
	}

	// Verify overridden values
	if config.ZeroMQ.ReceiverAddress != "tcp://override:5555" {
		t.Errorf("Environment override failed: Expected receiver_address tcp://override:5555, got %s",
			config.ZeroMQ.ReceiverAddress)
	}

	if config.Server.Port != 9090 {
		t.Errorf("Environment override failed: Expected port 9090, got %d", config.Server.Port)
	}

	if config.Processing.HighPriorityWorkers != 8 {
		t.Errorf("Environment override failed: Expected high_priority_workers 8, got %d",
			config.Processing.HighPriorityWorkers)
	}

	// Clean up
	os.Unsetenv("TELEOP_ZMQ_RECEIVER_ADDRESS")
	os.Unsetenv("TELEOP_SERVER_PORT")
	os.Unsetenv("TELEOP_HIGH_PRIORITY_WORKERS")
}

func TestLoadConfigWithEnv(t *testing.T) {
	// Create a temporary test config files
	tempDir, err := ioutil.TempDir("", "config-test")
	if err != nil {
		t.Fatalf("Failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tempDir)

	// Development config
	devConfigContent := `
version: "1.0"
config_id: "dev-config"
lastUpdated: "2025-03-16T10:00:00Z"
environment: "development"
zeromq:
  receiver_address: "tcp://localhost:5555"
`

	// Testing config
	testConfigContent := `
version: "1.0"
config_id: "test-config"
lastUpdated: "2025-03-16T10:00:00Z"
environment: "testing"
zeromq:
  receiver_address: "tcp://*:5555"
`

	// Write config files
	devConfigPath := filepath.Join(tempDir, "controller_config_development.yaml")
	if err := ioutil.WriteFile(devConfigPath, []byte(devConfigContent), 0644); err != nil {
		t.Fatalf("Failed to write dev config: %v", err)
	}

	testConfigPath := filepath.Join(tempDir, "controller_config_testing.yaml")
	if err := ioutil.WriteFile(testConfigPath, []byte(testConfigContent), 0644); err != nil {
		t.Fatalf("Failed to write test config: %v", err)
	}

	// Test default (development) environment
	config, err := LoadConfigWithEnv(tempDir, "")
	if err != nil {
		t.Fatalf("LoadConfigWithEnv failed: %v", err)
	}

	if config.ConfigID != "dev-config" {
		t.Errorf("Expected config_id dev-config, got %s", config.ConfigID)
	}

	if config.ZeroMQ.ReceiverAddress != "tcp://localhost:5555" {
		t.Errorf("Expected receiver_address tcp://localhost:5555, got %s", config.ZeroMQ.ReceiverAddress)
	}

	// Test explicitly specified environment
	config, err = LoadConfigWithEnv(tempDir, "testing")
	if err != nil {
		t.Fatalf("LoadConfigWithEnv failed: %v", err)
	}

	if config.ConfigID != "test-config" {
		t.Errorf("Expected config_id test-config, got %s", config.ConfigID)
	}

	if config.ZeroMQ.ReceiverAddress != "tcp://*:5555" {
		t.Errorf("Expected receiver_address tcp://*:5555, got %s", config.ZeroMQ.ReceiverAddress)
	}

	// Test environment variable override
	os.Setenv("TELEOP_ENVIRONMENT", "testing")
	defer os.Unsetenv("TELEOP_ENVIRONMENT")

	config, err = LoadConfigWithEnv(tempDir, "")
	if err != nil {
		t.Fatalf("LoadConfigWithEnv failed: %v", err)
	}

	if config.ConfigID != "test-config" {
		t.Errorf("Expected config_id test-config, got %s", config.ConfigID)
	}
} 