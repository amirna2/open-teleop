package services

import (
	"fmt"
	"io/ioutil"
	"sync"

	"github.com/open-teleop/controller/pkg/config"
	customlog "github.com/open-teleop/controller/pkg/log"
	"gopkg.in/yaml.v3"
)

// ConfigPublisher defines the interface for publishing configuration updates.
// This avoids a direct dependency on the concrete ZeroMQService or ConfigPublisher implementation.
type ConfigPublisher interface {
	PublishConfigUpdatedNotification() error
	// Add other methods if needed, e.g., PublishConfigUpdate()
}

// TeleopConfigService defines the interface for managing the operational teleop configuration.
type TeleopConfigService interface {
	LoadConfig() error
	GetCurrentConfig() *config.Config
	GetCurrentConfigYAML() ([]byte, error)
	UpdateConfig(newConfigYAML []byte) error
	PersistConfig(yamlData []byte) error
	SetPublisher(p ConfigPublisher)
}

// teleopConfigService implements the TeleopConfigService interface.
type teleopConfigService struct {
	operationalConfigPath string
	logger                customlog.Logger
	configPublisher       ConfigPublisher
	currentConfig         *config.Config
	mu                    sync.RWMutex
}

// NewTeleopConfigService creates a new TeleopConfigService.
// Publisher can be set later via SetPublisher.
func NewTeleopConfigService(operationalConfigPath string, logger customlog.Logger) (TeleopConfigService, error) {
	if operationalConfigPath == "" {
		return nil, fmt.Errorf("operational configuration path cannot be empty")
	}
	if logger == nil {
		// Fallback to a basic logger if none provided, although ideally it should always be passed in
		logger, _ = customlog.NewLogrusLogger("info", "")
		logger.Warnf("No logger provided to TeleopConfigService, using default.")
	}

	service := &teleopConfigService{
		operationalConfigPath: operationalConfigPath,
		logger:                logger,
		configPublisher:       nil, // Initialize publisher as nil
		mu:                    sync.RWMutex{},
	}

	// Attempt initial load
	if err := service.LoadConfig(); err != nil {
		// Log the error but allow service creation, maybe the file doesn't exist yet
		// or needs to be created/provided via API.
		logger.Warnf("Initial load of operational config '%s' failed: %v. Service created, but config is nil.", operationalConfigPath, err)
		// Allow creation even if load fails initially
		return service, nil
	}

	logger.Infof("TeleopConfigService initialized successfully for path: %s", operationalConfigPath)
	return service, nil
}

// LoadConfig reads the operational config file from disk and updates the currentConfig.
func (s *teleopConfigService) LoadConfig() error {
	s.mu.Lock()
	defer s.mu.Unlock()

	s.logger.Infof("Loading operational configuration from: %s", s.operationalConfigPath)
	data, err := ioutil.ReadFile(s.operationalConfigPath)
	if err != nil {
		s.logger.Errorf("Error reading operational config file '%s': %v", s.operationalConfigPath, err)
		// Keep existing config if read fails? Or set to nil? Setting to nil for now.
		s.currentConfig = nil
		return fmt.Errorf("error reading operational config file '%s': %w", s.operationalConfigPath, err)
	}

	var cfg config.Config
	if err := yaml.Unmarshal(data, &cfg); err != nil {
		s.logger.Errorf("Error parsing operational config file '%s': %v", s.operationalConfigPath, err)
		// Keep existing config if parse fails? Or set to nil? Setting to nil for now.
		s.currentConfig = nil
		return fmt.Errorf("error parsing operational config file '%s': %w", s.operationalConfigPath, err)
	}

	// TODO: Add more comprehensive validation here if needed

	s.currentConfig = &cfg
	s.logger.Infof("Successfully loaded operational configuration ID: %s, Version: %s", cfg.ConfigID, cfg.Version)
	return nil
}

// GetCurrentConfig returns a pointer to the currently loaded operational configuration.
// It's read-only; modifications should go through UpdateConfig.
func (s *teleopConfigService) GetCurrentConfig() *config.Config {
	s.mu.RLock()
	defer s.mu.RUnlock()
	// Return a copy to prevent modification? For now, return pointer but emphasize read-only.
	return s.currentConfig
}

// GetCurrentConfigYAML reads the operational config file from disk and returns its raw YAML content.
// This is useful for the UI to display the current state before editing.
func (s *teleopConfigService) GetCurrentConfigYAML() ([]byte, error) {
	s.mu.RLock()
	path := s.operationalConfigPath
	s.mu.RUnlock() // Unlock before file I/O

	s.logger.Debugf("Reading raw operational configuration YAML from: %s", path)
	data, err := ioutil.ReadFile(path)
	if err != nil {
		s.logger.Errorf("Error reading operational config file '%s' for YAML export: %v", path, err)
		return nil, fmt.Errorf("error reading operational config file '%s': %w", path, err)
	}
	return data, nil
}

// UpdateConfig validates, persists, applies the new operational configuration, and publishes a notification.
// It takes the new configuration as a YAML byte slice.
func (s *teleopConfigService) UpdateConfig(newConfigYAML []byte) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	s.logger.Infof("Attempting to update operational configuration from provided YAML")

	// 1. Parse and Validate the new YAML
	var newCfg config.Config
	if err := yaml.Unmarshal(newConfigYAML, &newCfg); err != nil {
		s.logger.Errorf("Failed to parse provided YAML configuration: %v", err)
		return fmt.Errorf("invalid YAML format: %w", err)
	}
	// TODO: Add more validation logic here (e.g., check required fields, semantic checks)
	if newCfg.ConfigID == "" || newCfg.Version == "" || newCfg.RobotID == "" {
		s.logger.Errorf("Validation failed: Missing required fields (ConfigID, Version, RobotID) in provided YAML.")
		return fmt.Errorf("validation failed: missing required fields (ConfigID, Version, RobotID)")
	}
	// Basic validation passed

	// 2. Compare with current config (optional, but good for logging/events)
	// For simplicity now, we'll update regardless if validation passes.
	// A deep comparison could be added later (e.g., using reflect.DeepEqual or a library).
	isDifferent := true // Assume different for now
	if s.currentConfig != nil {
		// Perform actual comparison if needed
		// isDifferent = !reflect.DeepEqual(s.currentConfig, &newCfg)
		s.logger.Infof("New config (ID: %s) received. Assuming changes.", newCfg.ConfigID)
	} else {
		s.logger.Infof("New config (ID: %s) received. No current config loaded.", newCfg.ConfigID)
	}

	if !isDifferent {
		s.logger.Infof("Provided configuration is identical to the current one. No update needed.")
		return nil // Or return a specific "no change" error/status
	}

	// 3. Persist the new configuration YAML *before* applying it
	if err := s.persistConfigUnlocked(newConfigYAML); err != nil {
		// If persistence fails, we don't update the active config
		return err
	}

	// 4. Apply the new configuration (update in-memory representation)
	oldCfgId := "N/A"
	if s.currentConfig != nil {
		oldCfgId = s.currentConfig.ConfigID
	}
	s.currentConfig = &newCfg
	s.logger.Infof("Successfully updated and persisted operational configuration. ID %s -> %s, Version: %s", oldCfgId, s.currentConfig.ConfigID, s.currentConfig.Version)

	// 5. Trigger notification via ConfigPublisher (if available)
	if s.configPublisher != nil {
		// Publish notification in a separate goroutine to avoid blocking the update process
		go func(publisher ConfigPublisher) {
			s.logger.Debugf("Attempting to publish config update notification...")
			if err := publisher.PublishConfigUpdatedNotification(); err != nil {
				s.logger.Warnf("Failed to publish config update notification: %v", err)
			} else {
				s.logger.Infof("Published config update notification successfully.")
			}
		}(s.configPublisher) // Pass publisher to goroutine
	} else {
		s.logger.Infof("ConfigPublisher not configured, skipping update notification.")
	}

	// NOTE: Reloading dependent components (like TopicRegistry) still needs to be handled elsewhere
	// This service only publishes the notification.
	s.logger.Infof("Configuration updated. Notification sent (if publisher configured). Dependent components may require manual reload or restart.")

	return nil
}

// PersistConfig writes the given YAML data to the operational config file path.
// This is exposed publicly mainly for testing or potential external triggers.
func (s *teleopConfigService) PersistConfig(yamlData []byte) error {
	s.mu.Lock() // Lock to ensure path consistency if needed, although path itself doesn't change
	defer s.mu.Unlock()
	return s.persistConfigUnlocked(yamlData)
}

// persistConfigUnlocked is the internal implementation for writing the config file.
// It assumes the caller holds the necessary lock.
func (s *teleopConfigService) persistConfigUnlocked(yamlData []byte) error {
	s.logger.Infof("Persisting operational configuration to: %s", s.operationalConfigPath)
	// Write with permissions that allow the owner to read/write
	err := ioutil.WriteFile(s.operationalConfigPath, yamlData, 0644)
	if err != nil {
		s.logger.Errorf("Error writing operational config file '%s': %v", s.operationalConfigPath, err)
		return fmt.Errorf("error writing operational config file '%s': %w", s.operationalConfigPath, err)
	}
	s.logger.Infof("Successfully persisted configuration to %s", s.operationalConfigPath)
	return nil
}

// SetPublisher allows injecting the ConfigPublisher after initialization.
func (s *teleopConfigService) SetPublisher(p ConfigPublisher) {
	s.mu.Lock() // Lock needed as we modify the service state
	defer s.mu.Unlock()
	s.configPublisher = p
	s.logger.Infof("ConfigPublisher injected into TeleopConfigService.")
}
