package zeromq

import (
	"log"

	"github.com/open-teleop/controller/pkg/config"
)

// ConfigPublisher publishes configuration updates to gateways
type ConfigPublisher struct {
	service *ZeroMQService
	config  *config.Config
	logger  *log.Logger
}

// NewConfigPublisher creates a new publisher for configuration updates
func NewConfigPublisher(service *ZeroMQService, cfg *config.Config, logger *log.Logger) *ConfigPublisher {
	return &ConfigPublisher{
		service: service,
		config:  cfg,
		logger:  logger,
	}
}

// PublishConfigUpdate publishes the current configuration to all subscribed gateways
func (p *ConfigPublisher) PublishConfigUpdate() error {
	p.logger.Printf("Publishing configuration update (ID: %s)", p.config.ConfigID)

	// Publish to the configuration topic
	return p.service.PublishJSON("configuration.update", MsgTypeConfigResponse, p.config)
}

// PublishConfigUpdatedNotification publishes a notification that the config has been updated
func (p *ConfigPublisher) PublishConfigUpdatedNotification() error {
	p.logger.Printf("Publishing configuration update notification")

	// Create a notification message with minimal info
	notification := map[string]interface{}{
		"config_id":    p.config.ConfigID,
		"version":      p.config.Version,
		"last_updated": p.config.LastUpdated,
	}

	// Publish to the notification topic
	return p.service.PublishJSON("configuration.notification", "CONFIG_UPDATED", notification)
}

// RegisterConfigHandlers registers config-related handlers and services
func RegisterConfigHandlers(service *ZeroMQService, cfg *config.Config, logger *log.Logger) *ConfigPublisher {
	// Create and register the configuration request handler
	configHandler := NewConfigHandler(cfg, logger)
	service.RegisterHandler(MsgTypeConfigRequest, configHandler)

	// Create the config publisher
	publisher := NewConfigPublisher(service, cfg, logger)

	logger.Printf("Registered configuration handlers and publisher")
	return publisher
}
