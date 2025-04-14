package config

import (
	"io/ioutil"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestLoadConfig(t *testing.T) {
	// Create a temporary test config file
	tempDir, err := ioutil.TempDir("", "config-test")
	if err != nil {
		t.Fatalf("Failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tempDir)

	// Updated configContent matching open_teleop_config.yaml structure
	configContent := `
# Test config matching the structure of config/open_teleop_config.yaml
version: "1.0"
config_id: "test-teleop-config" # Example ID matching file structure
lastUpdated: "2024-01-01T00:00:00Z" # Example timestamp
robot_id: "test-robot-teleop" # Example robot ID

# Topic mappings section
topic_mappings:
  # Example OUTBOUND
  - ros_topic: "/test_sensors"
    ott: "teleop.test.sensors"
    priority: "STANDARD"
    message_type: "sensor_msgs/msg/Image" # Example type
    direction: "OUTBOUND"
    source_type: "ROS2_CDR"

  # Example INBOUND
  - ros_topic: "/test_control"
    ott: "teleop.test.control"
    priority: "HIGH"
    message_type: "geometry_msgs/msg/Twist" # Example type
    direction: "INBOUND"
    source_type: "ROS2_CDR"

# Defaults section
defaults:
  priority: "STANDARD"
  direction: "OUTBOUND"
  source_type: "ROS2_CDR"

# Throttle rates section
throttle_rates:
  high_hz: 0
  standard_hz: 15 # Example value
  low_hz: 2       # Example value
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

	// Verify metadata
	if config.Version != "1.0" {
		t.Errorf("Expected version 1.0, got %s", config.Version)
	}

	if config.ConfigID != "test-teleop-config" {
		t.Errorf("Expected config_id test-teleop-config, got %s", config.ConfigID)
	}

	if config.RobotID != "test-robot-teleop" {
		t.Errorf("Expected robot_id test-robot-teleop, got %s", config.RobotID)
	}

	// Verify topics
	if len(config.TopicMappings) != 2 {
		t.Errorf("Expected 2 topic mappings, got %d", len(config.TopicMappings))
	}

	// Verify throttle rates (using updated example values)
	if config.ThrottleRates.HighHz != 0 {
		t.Errorf("Expected high_hz 0, got %d", config.ThrottleRates.HighHz)
	}
	if config.ThrottleRates.StandardHz != 15 {
		t.Errorf("Expected standard_hz 15, got %d", config.ThrottleRates.StandardHz)
	}
	if config.ThrottleRates.LowHz != 2 {
		t.Errorf("Expected low_hz 2, got %d", config.ThrottleRates.LowHz)
	}
}

func TestTopicMappingHelpers(t *testing.T) {
	// Create a test config with topic mappings
	config := &Config{
		TopicMappings: []TopicMapping{
			{
				RosTopic:    "/battery_state",
				OttTopic:    "teleop.sensor.battery",
				Priority:    "STANDARD",
				MessageType: "sensor_msgs/msg/BatteryState",
				Direction:   "OUTBOUND",
				SourceType:  "ROS2_CDR",
			},
			{
				RosTopic:    "/cmd_vel",
				OttTopic:    "teleop.control.velocity",
				Priority:    "HIGH",
				MessageType: "geometry_msgs/msg/Twist",
				Direction:   "INBOUND",
				SourceType:  "ROS2_CDR",
			},
			{
				// Missing direction, will use default
				RosTopic:    "/robot_state",
				OttTopic:    "teleop.robot.state",
				MessageType: "std_msgs/msg/String",
				SourceType:  "ROS2_CDR",
			},
		},
		Defaults: DefaultsConfig{
			Priority:   "STANDARD",
			Direction:  "OUTBOUND",
			SourceType: "ROS2_CDR",
		},
	}

	// Test GetTopicMappingsByDirection
	inboundTopics := config.GetTopicMappingsByDirection("INBOUND")
	if len(inboundTopics) != 1 {
		t.Errorf("Expected 1 inbound topic, got %d", len(inboundTopics))
	}

	if inboundTopics[0].OttTopic != "teleop.control.velocity" {
		t.Errorf("Expected teleop.control.velocity, got %s", inboundTopics[0].OttTopic)
	}

	outboundTopics := config.GetTopicMappingsByDirection("OUTBOUND")
	if len(outboundTopics) != 2 {
		t.Errorf("Expected 2 outbound topics, got %d", len(outboundTopics))
	}

	// Test GetTopicMappingByOttTopic
	velocityTopic, found := config.GetTopicMappingByOttTopic("teleop.control.velocity")
	if !found {
		t.Errorf("Expected to find teleop.control.velocity topic")
	}

	if velocityTopic.Direction != "INBOUND" {
		t.Errorf("Expected INBOUND direction, got %s", velocityTopic.Direction)
	}

	if velocityTopic.RosTopic != "/cmd_vel" {
		t.Errorf("Expected /cmd_vel ROS topic, got %s", velocityTopic.RosTopic)
	}

	// Test defaults application
	robotStateTopic, found := config.GetTopicMappingByOttTopic("teleop.robot.state")
	if !found {
		t.Errorf("Expected to find teleop.robot.state topic")
	}

	if robotStateTopic.Direction != "OUTBOUND" {
		t.Errorf("Expected default OUTBOUND direction, got %s", robotStateTopic.Direction)
	}

	if robotStateTopic.Priority != "STANDARD" {
		t.Errorf("Expected default STANDARD priority, got %s", robotStateTopic.Priority)
	}

	// Test not found topic
	_, found = config.GetTopicMappingByOttTopic("teleop.nonexistent")
	if found {
		t.Errorf("Expected not to find teleop.nonexistent topic")
	}
}

func TestLoadBootstrapConfig(t *testing.T) {
	// Create a temporary directory for the bootstrap config
	tempDir, err := ioutil.TempDir("", "bootstrap-config-test")
	if err != nil {
		t.Fatalf("Failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tempDir)

	// Define the content for the temporary bootstrap config file
	bootstrapContent := `
logging:
  level: "debug"
  log_path: "/var/log/controller"
server:
  http_port: 9090
zeromq:
  request_bind_address: "tcp://*:6666"
  publish_bind_address: "tcp://*:7777"
  message_buffer_size: 2000
  reconnect_interval_ms: 500
data:
  directory: "/data/controller"
  teleop_config_file: "my_teleop_config.yaml"
processing:
  high_priority_workers: 5
  standard_priority_workers: 3
  low_priority_workers: 2
`
	// Write the temporary bootstrap config file
	configPath := filepath.Join(tempDir, "controller_config.yaml")
	if err := ioutil.WriteFile(configPath, []byte(bootstrapContent), 0644); err != nil {
		t.Fatalf("Failed to write test bootstrap config: %v", err)
	}

	// Test loading the bootstrap config
	bootstrapCfg, err := LoadBootstrapConfig(tempDir)
	if err != nil {
		t.Fatalf("LoadBootstrapConfig failed: %v", err)
	}

	// Verify loaded values
	if bootstrapCfg.Logging.Level != "debug" {
		t.Errorf("Expected logging level 'debug', got '%s'", bootstrapCfg.Logging.Level)
	}
	if bootstrapCfg.Logging.LogPath != "/var/log/controller" {
		t.Errorf("Expected log path '/var/log/controller', got '%s'", bootstrapCfg.Logging.LogPath)
	}
	if bootstrapCfg.Server.HTTPPort != 9090 {
		t.Errorf("Expected server http_port 9090, got %d", bootstrapCfg.Server.HTTPPort)
	}
	if bootstrapCfg.ZeroMQ.RequestBindAddress != "tcp://*:6666" {
		t.Errorf("Expected zeromq request_bind_address 'tcp://*:6666', got '%s'", bootstrapCfg.ZeroMQ.RequestBindAddress)
	}
	if bootstrapCfg.ZeroMQ.PublishBindAddress != "tcp://*:7777" {
		t.Errorf("Expected zeromq publish_bind_address 'tcp://*:7777', got '%s'", bootstrapCfg.ZeroMQ.PublishBindAddress)
	}
	if bootstrapCfg.ZeroMQ.MessageBufferSize != 2000 {
		t.Errorf("Expected zeromq message_buffer_size 2000, got %d", bootstrapCfg.ZeroMQ.MessageBufferSize)
	}
	if bootstrapCfg.ZeroMQ.ReconnectIntervalMs != 500 {
		t.Errorf("Expected zeromq reconnect_interval_ms 500, got %d", bootstrapCfg.ZeroMQ.ReconnectIntervalMs)
	}
	if bootstrapCfg.Data.Directory != "/data/controller" {
		t.Errorf("Expected data directory '/data/controller', got '%s'", bootstrapCfg.Data.Directory)
	}
	if bootstrapCfg.Data.TeleopConfigFilename != "my_teleop_config.yaml" {
		t.Errorf("Expected data teleop_config_file 'my_teleop_config.yaml', got '%s'", bootstrapCfg.Data.TeleopConfigFilename)
	}
	if bootstrapCfg.Processing.StandardPriorityWorkers != 3 {
		t.Errorf("Expected processing standard_priority_workers 3, got %d", bootstrapCfg.Processing.StandardPriorityWorkers)
	}
	if bootstrapCfg.Processing.HighPriorityWorkers != 5 {
		t.Errorf("Expected processing high_priority_workers 5, got %d", bootstrapCfg.Processing.HighPriorityWorkers)
	}
	if bootstrapCfg.Processing.LowPriorityWorkers != 2 {
		t.Errorf("Expected processing low_priority_workers 2, got %d", bootstrapCfg.Processing.LowPriorityWorkers)
	}
}

// Test case for missing required fields validation in LoadBootstrapConfig
func TestLoadBootstrapConfigMissingRequired(t *testing.T) {
	tempDir, err := ioutil.TempDir("", "bootstrap-config-missing-test")
	if err != nil {
		t.Fatalf("Failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tempDir)

	// Missing 'zeromq.request_bind_address'
	bootstrapContentMissing := `
logging:
  level: "info"
server:
  http_port: 8080
zeromq:
  # request_bind_address: "tcp://*:6666" # Missing
  publish_bind_address: "tcp://*:7777"
  message_buffer_size: 1000
  reconnect_interval_ms: 1000
data:
  directory: "/data"
  teleop_config_file: "op_config.yaml"
processing:
  high_priority_workers: 4
  standard_priority_workers: 2
  low_priority_workers: 1
`
	configPath := filepath.Join(tempDir, "controller_config.yaml")
	if err := ioutil.WriteFile(configPath, []byte(bootstrapContentMissing), 0644); err != nil {
		t.Fatalf("Failed to write test bootstrap config: %v", err)
	}

	// Attempt to load the config with missing field
	_, err = LoadBootstrapConfig(tempDir)
	if err == nil {
		t.Errorf("Expected error when loading bootstrap config with missing required fields, but got nil")
	}

	// Check if the error message contains the expected field name
	expectedErrorSubstr := "missing required field in bootstrap config: zeromq.request_bind_address"
	if err != nil && !strings.Contains(err.Error(), expectedErrorSubstr) {
		t.Errorf("Expected error message to contain '%s', but got: %v", expectedErrorSubstr, err)
	}

	// Note: Could add separate test cases for other required fields if desired
}
