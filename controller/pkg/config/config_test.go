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
robot_id: "test-robot-id"

controller:
  server:
    port: 8080
    request_timeout: 30
    max_request_size: 10

  processing:
    high_priority_workers: 4
    standard_priority_workers: 2
    low_priority_workers: 1

zeromq:
  controller_address: "tcp://*:5555"
  gateway_address: "tcp://*:5556"
  gateway_connect_address: "tcp://localhost:5555"
  gateway_subscribe_address: "tcp://localhost:5556"
  message_buffer_size: 1000
  reconnect_interval_ms: 1000

topic_mappings:
  - ros_topic: "/battery_state"
    ott: "teleop.sensor.battery"
    priority: "STANDARD"
    message_type: "sensor_msgs/msg/BatteryState"
    direction: "OUTBOUND"
    source_type: "ROS2_CDM"
  
  - ros_topic: "/cmd_vel"
    ott: "teleop.control.velocity"
    priority: "HIGH"
    message_type: "geometry_msgs/msg/Twist"
    direction: "INBOUND"
    source_type: "ROS2_CDM"

defaults:
  priority: "STANDARD"
  direction: "OUTBOUND"
  source_type: "ROS2_CDM"

throttle_rates:
  high_hz: 0
  standard_hz: 10
  low_hz: 1
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
	
	if config.ConfigID != "test-config" {
		t.Errorf("Expected config_id test-config, got %s", config.ConfigID)
	}
	
	if config.Environment != "testing" {
		t.Errorf("Expected environment testing, got %s", config.Environment)
	}
	
	if config.RobotID != "test-robot-id" {
		t.Errorf("Expected robot_id test-robot-id, got %s", config.RobotID)
	}

	// Verify controller config
	if config.Controller.Server.Port != 8080 {
		t.Errorf("Expected port 8080, got %d", config.Controller.Server.Port)
	}
	
	if config.Controller.Processing.HighPriorityWorkers != 4 {
		t.Errorf("Expected high_priority_workers 4, got %d", config.Controller.Processing.HighPriorityWorkers)
	}

	// Verify ZeroMQ config
	if config.ZeroMQ.ControllerAddress != "tcp://*:5555" {
		t.Errorf("Expected controller_address tcp://*:5555, got %s", config.ZeroMQ.ControllerAddress)
	}
	
	if config.ZeroMQ.MessageBufferSize != 1000 {
		t.Errorf("Expected message_buffer_size 1000, got %d", config.ZeroMQ.MessageBufferSize)
	}
	
	// Verify topics
	if len(config.TopicMappings) != 2 {
		t.Errorf("Expected 2 topic mappings, got %d", len(config.TopicMappings))
	}
	
	// Verify throttle rates
	if config.ThrottleRates.HighHz != 0 {
		t.Errorf("Expected high_hz 0, got %d", config.ThrottleRates.HighHz)
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
robot_id: "test-robot-id"

controller:
  server:
    port: 8080
    request_timeout: 30
    max_request_size: 10

  processing:
    high_priority_workers: 4
    standard_priority_workers: 2
    low_priority_workers: 1

zeromq:
  controller_address: "tcp://*:5555"
  gateway_address: "tcp://*:5556"
  gateway_connect_address: "tcp://localhost:5555"
  gateway_subscribe_address: "tcp://localhost:5556"
  message_buffer_size: 1000
  reconnect_interval_ms: 1000

topic_mappings:
  - ros_topic: "/battery_state"
    ott: "teleop.sensor.battery"
    priority: "STANDARD"
    message_type: "sensor_msgs/msg/BatteryState"
    direction: "OUTBOUND"
    source_type: "ROS2_CDM"
  
  - ros_topic: "/cmd_vel"
    ott: "teleop.control.velocity"
    priority: "HIGH"
    message_type: "geometry_msgs/msg/Twist"
    direction: "INBOUND"
    source_type: "ROS2_CDM"

defaults:
  priority: "STANDARD"
  direction: "OUTBOUND"
  source_type: "ROS2_CDM"

throttle_rates:
  high_hz: 0
  standard_hz: 10
  low_hz: 1
`

	configPath := filepath.Join(tempDir, "test_config.yaml")
	if err := ioutil.WriteFile(configPath, []byte(configContent), 0644); err != nil {
		t.Fatalf("Failed to write test config: %v", err)
	}

	// Set environment variables
	os.Setenv("TELEOP_ZMQ_CONTROLLER_ADDRESS", "tcp://*:5559")
	os.Setenv("TELEOP_SERVER_PORT", "9090")
	os.Setenv("TELEOP_HIGH_PRIORITY_WORKERS", "8")
	os.Setenv("TELEOP_THROTTLE_LOW_HZ", "5")
	defer func() {
		os.Unsetenv("TELEOP_ZMQ_CONTROLLER_ADDRESS")
		os.Unsetenv("TELEOP_SERVER_PORT")
		os.Unsetenv("TELEOP_HIGH_PRIORITY_WORKERS")
		os.Unsetenv("TELEOP_THROTTLE_LOW_HZ")
	}()

	// Test loading the config with environment overrides
	config, err := LoadConfig(configPath)
	if err != nil {
		t.Fatalf("LoadConfig failed: %v", err)
	}

	// Verify that environment variables override config values
	if config.ZeroMQ.ControllerAddress != "tcp://*:5559" {
		t.Errorf("Environment override failed. Expected controller_address tcp://*:5559, got %s", config.ZeroMQ.ControllerAddress)
	}
	
	if config.Controller.Server.Port != 9090 {
		t.Errorf("Environment override failed. Expected port 9090, got %d", config.Controller.Server.Port)
	}
	
	if config.Controller.Processing.HighPriorityWorkers != 8 {
		t.Errorf("Environment override failed. Expected high_priority_workers 8, got %d", config.Controller.Processing.HighPriorityWorkers)
	}
	
	if config.ThrottleRates.LowHz != 5 {
		t.Errorf("Environment override failed. Expected low_hz 5, got %d", config.ThrottleRates.LowHz)
	}
}

func TestLoadConfigWithEnv(t *testing.T) {
	// Create a temporary test directory
	tempDir, err := ioutil.TempDir("", "config-test")
	if err != nil {
		t.Fatalf("Failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tempDir)

	// Create unified config
	unifiedConfigContent := `
version: "1.0"
config_id: "unified-config"
lastUpdated: "2025-03-17T00:00:00Z"
robot_id: "d290f1ee-6c54-4b01-90e6-d701748f0851"

controller:
  server:
    port: 8080
    request_timeout: 30
    max_request_size: 10
  
  processing:
    high_priority_workers: 4
    standard_priority_workers: 2
    low_priority_workers: 1

zeromq:
  controller_address: "tcp://*:5555"
  gateway_address: "tcp://*:5556"
  gateway_connect_address: "tcp://localhost:5555"
  gateway_subscribe_address: "tcp://localhost:5556"
  message_buffer_size: 1000
  reconnect_interval_ms: 1000

topic_mappings:
  - ros_topic: "/battery_state"
    ott: "teleop.sensor.battery"
    priority: "STANDARD"
    message_type: "sensor_msgs/msg/BatteryState"
    direction: "OUTBOUND"
    source_type: "ROS2_CDM"
  
  - ros_topic: "/cmd_vel"
    ott: "teleop.control.velocity"
    priority: "HIGH"
    message_type: "geometry_msgs/msg/Twist"
    direction: "INBOUND"
    source_type: "ROS2_CDM"
    
  - ott: "teleop.diagnostic.system_metrics"
    priority: "LOW"
    source_type: "OPEN_TELEOP"
    direction: "OUTBOUND"

defaults:
  priority: "STANDARD"
  direction: "OUTBOUND"
  source_type: "ROS2_CDM"

throttle_rates:
  high_hz: 0
  standard_hz: 10
  low_hz: 1
`

	// Write unified config file
	unifiedConfigPath := filepath.Join(tempDir, "open_teleop_config.yaml")
	if err := ioutil.WriteFile(unifiedConfigPath, []byte(unifiedConfigContent), 0644); err != nil {
		t.Fatalf("Failed to write unified config: %v", err)
	}

	// Test loading config with default environment
	config, err := LoadConfigWithEnv(tempDir, "")
	if err != nil {
		t.Fatalf("LoadConfigWithEnv failed: %v", err)
	}

	// Verify that unified config was loaded
	if config.ConfigID != "unified-config" {
		t.Errorf("Expected unified config to be loaded, got config_id %s", config.ConfigID)
	}
	
	if config.Environment != "development" {
		t.Errorf("Expected default environment development, got %s", config.Environment)
	}

	// Test loading with explicit environment
	config, err = LoadConfigWithEnv(tempDir, "production")
	if err != nil {
		t.Fatalf("LoadConfigWithEnv failed: %v", err)
	}

	// Verify that environment is set correctly
	if config.Environment != "production" {
		t.Errorf("Expected environment to be set to production, got %s", config.Environment)
	}
	
	// Test with missing config file
	if err := os.Remove(unifiedConfigPath); err != nil {
		t.Fatalf("Failed to remove unified config: %v", err)
	}
	
	_, err = LoadConfigWithEnv(tempDir, "development")
	if err == nil {
		t.Errorf("Expected error when config file is missing, got nil")
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
				SourceType:  "ROS2_CDM",
			},
			{
				RosTopic:    "/cmd_vel",
				OttTopic:    "teleop.control.velocity",
				Priority:    "HIGH",
				MessageType: "geometry_msgs/msg/Twist",
				Direction:   "INBOUND",
				SourceType:  "ROS2_CDM",
			},
			{
				OttTopic:   "teleop.diagnostic.system_metrics",
				Priority:   "LOW",
				Direction:  "OUTBOUND",
				SourceType: "OPEN_TELEOP",
			},
			{
				// Missing direction, will use default
				RosTopic:    "/robot_state",
				OttTopic:    "teleop.robot.state",
				MessageType: "std_msgs/msg/String",
				SourceType:  "ROS2_CDM",
			},
		},
		Defaults: DefaultsConfig{
			Priority:   "STANDARD",
			Direction:  "OUTBOUND",
			SourceType: "ROS2_CDM",
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
	if len(outboundTopics) != 3 {
		t.Errorf("Expected 3 outbound topics, got %d", len(outboundTopics))
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