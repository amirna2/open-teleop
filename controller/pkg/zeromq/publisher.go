package zeromq

import (
	// "log" // No longer use standard log

	"fmt"

	"github.com/open-teleop/controller/pkg/config"
	customlog "github.com/open-teleop/controller/pkg/log"
	"github.com/open-teleop/controller/services" // Import services package
)

// ConfigGetter defines the interface needed to get the current config.
// This avoids coupling directly to TeleopConfigService if only read access is needed.
type ConfigGetter interface {
	GetCurrentConfig() *config.Config
}

// ConfigPublisher publishes configuration updates to gateways
type ConfigPublisher struct {
	service   *ZeroMQService
	configSvc ConfigGetter     // Use the interface to get current config
	logger    customlog.Logger // Use custom logger interface
}

// NewConfigPublisher creates a new publisher for configuration updates
func NewConfigPublisher(service *ZeroMQService, configSvc ConfigGetter, logger customlog.Logger) *ConfigPublisher {
	if configSvc == nil {
		// Handle nil config service - maybe default to a static empty config?
		// For now, let's panic as it indicates a programming error during setup.
		panic("ConfigGetter cannot be nil in NewConfigPublisher")
	}
	return &ConfigPublisher{
		service:   service,
		configSvc: configSvc,
		logger:    logger,
	}
}

// PublishConfigUpdate publishes the current configuration to all subscribed gateways
func (p *ConfigPublisher) PublishConfigUpdate() error {
	currentCfg := p.configSvc.GetCurrentConfig()
	if currentCfg == nil {
		p.logger.Warnf("Cannot publish config update, current config is nil.")
		return fmt.Errorf("current config is nil")
	}
	p.logger.Infof("Publishing configuration update (ID: %s)", currentCfg.ConfigID)

	// Publish to the configuration topic
	return p.service.PublishJSON("configuration.update", MsgTypeConfigResponse, currentCfg)
}

// PublishConfigUpdatedNotification publishes a notification that the config has been updated
func (p *ConfigPublisher) PublishConfigUpdatedNotification() error {
	currentCfg := p.configSvc.GetCurrentConfig()
	if currentCfg == nil {
		p.logger.Warnf("Cannot publish config notification, current config is nil.")
		return fmt.Errorf("current config is nil")
	}
	p.logger.Infof("Publishing configuration update notification (ID: %s)", currentCfg.ConfigID)

	// Create a notification message with minimal info
	notification := map[string]interface{}{
		"config_id":    currentCfg.ConfigID,
		"version":      currentCfg.Version,
		"last_updated": currentCfg.LastUpdated,
	}

	// Publish to the notification topic
	return p.service.PublishJSON("configuration.notification", "CONFIG_UPDATED", notification)
}

// RegisterZmqConfigRequestHandler registers the handler for CONFIG_REQUEST messages.
// It accepts the TeleopConfigService to provide current config data to the handler.
func RegisterZmqConfigRequestHandler(service *ZeroMQService, configService services.TeleopConfigService, logger customlog.Logger) {
	// Create and register the configuration request handler, passing the service
	configHandler := NewConfigHandler(configService, logger)
	service.RegisterHandler(MsgTypeConfigRequest, configHandler)

	logger.Infof("Registered ZeroMQ handler for CONFIG_REQUEST messages.")
	// NOTE: This function no longer creates or returns the publisher.
}
