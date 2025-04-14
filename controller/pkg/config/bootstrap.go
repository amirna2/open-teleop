package config

import (
	"fmt"
	"io/ioutil"
	"path/filepath"

	"gopkg.in/yaml.v3"
)

// BootstrapConfig holds the initial configuration loaded from controller_config.yaml
type BootstrapConfig struct {
	Logging    LoggingConfig         `yaml:"logging"`
	Server     BootstrapServerConfig `yaml:"server"`
	ZeroMQ     ZeroMQBootstrap       `yaml:"zeromq"`
	Data       DataConfig            `yaml:"data"`
	Processing ProcessingConfig      `yaml:"processing"`
}

// LoggingConfig holds logging settings from bootstrap
type LoggingConfig struct {
	Level   string `yaml:"level"`
	LogPath string `yaml:"log_path,omitempty"`
}

// ServerConfig holds bootstrap server settings
// Renamed from original plan to avoid conflict if pkg/config/config.go ServerConfig is different
type BootstrapServerConfig struct {
	HTTPPort int `yaml:"http_port"`
}

// ZeroMQBootstrap holds ZeroMQ settings from bootstrap
type ZeroMQBootstrap struct {
	RequestBindAddress  string `yaml:"request_bind_address"`
	PublishBindAddress  string `yaml:"publish_bind_address"`
	MessageBufferSize   int    `yaml:"message_buffer_size"`
	ReconnectIntervalMs int    `yaml:"reconnect_interval_ms"`
}

// ProcessingConfig holds message processing worker configuration from bootstrap
type ProcessingConfig struct {
	HighPriorityWorkers     int `yaml:"high_priority_workers"`
	StandardPriorityWorkers int `yaml:"standard_priority_workers"`
	LowPriorityWorkers      int `yaml:"low_priority_workers"`
}

// DataConfig holds data directory settings from bootstrap
type DataConfig struct {
	Directory            string `yaml:"directory"`
	TeleopConfigFilename string `yaml:"teleop_config_file"`
}

// LoadBootstrapConfig loads the bootstrap configuration from controller_config.yaml
func LoadBootstrapConfig(configDir string) (*BootstrapConfig, error) {
	bootstrapConfigPath := filepath.Join(configDir, "controller_config.yaml")

	data, err := ioutil.ReadFile(bootstrapConfigPath)
	if err != nil {
		return nil, fmt.Errorf("error reading bootstrap config file '%s': %w", bootstrapConfigPath, err)
	}

	var bootstrapCfg BootstrapConfig
	if err := yaml.Unmarshal(data, &bootstrapCfg); err != nil {
		return nil, fmt.Errorf("error parsing bootstrap config file '%s': %w", bootstrapConfigPath, err)
	}

	// Basic validation (optional, add more as needed)
	if bootstrapCfg.ZeroMQ.RequestBindAddress == "" {
		return nil, fmt.Errorf("missing required field in bootstrap config: zeromq.request_bind_address")
	}
	if bootstrapCfg.ZeroMQ.PublishBindAddress == "" {
		return nil, fmt.Errorf("missing required field in bootstrap config: zeromq.publish_bind_address")
	}
	if bootstrapCfg.Data.Directory == "" {
		return nil, fmt.Errorf("missing required field in bootstrap config: data.directory")
	}
	if bootstrapCfg.Data.TeleopConfigFilename == "" {
		return nil, fmt.Errorf("missing required field in bootstrap config: data.teleop_config_file")
	}

	return &bootstrapCfg, nil
}
