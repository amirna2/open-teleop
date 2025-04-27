package api

import (
	"fmt"
	"net/http" // Import net/http for status codes

	"github.com/gofiber/fiber/v2"
	customlog "github.com/open-teleop/controller/pkg/log"
	"github.com/open-teleop/controller/services"
)

// ConfigHandler holds dependencies for configuration API endpoints.
type ConfigHandler struct {
	configService services.TeleopConfigService
	logger        customlog.Logger
}

// NewConfigHandler creates a new handler for configuration endpoints.
func NewConfigHandler(configService services.TeleopConfigService, logger customlog.Logger) *ConfigHandler {
	// Basic validation
	if configService == nil {
		panic("ConfigService cannot be nil in NewConfigHandler")
	}
	if logger == nil {
		panic("Logger cannot be nil in NewConfigHandler")
	}
	return &ConfigHandler{
		configService: configService,
		logger:        logger,
	}
}

// RegisterConfigRoutes registers the configuration API endpoints with the Fiber app.
func RegisterConfigRoutes(app *fiber.App, configService services.TeleopConfigService, logger customlog.Logger) {
	h := NewConfigHandler(configService, logger)

	// Define the API group
	apiGroup := app.Group("/api/v1/config") // Group under /api/v1/config

	// GET endpoint to retrieve the current operational configuration as YAML
	apiGroup.Get("/teleop", h.handleGetTeleopConfig)

	// PUT endpoint to update the operational configuration
	apiGroup.Put("/teleop", h.handleUpdateTeleopConfig)

	logger.Infof("Registered teleop configuration API endpoints under /api/v1/config")
}

// handleGetTeleopConfig handles GET requests to retrieve the current teleop config YAML.
func (h *ConfigHandler) handleGetTeleopConfig(c *fiber.Ctx) error {
	h.logger.Debugf("Handling GET request for /api/v1/config/teleop")
	yamlData, err := h.configService.GetCurrentConfigYAML()
	if err != nil {
		h.logger.Errorf("Failed to get current teleop config YAML: %v", err)
		// Distinguish between file not found and other errors if necessary
		// For now, treat all errors as internal server error
		return c.Status(http.StatusInternalServerError).JSON(fiber.Map{
			"error": fmt.Sprintf("Failed to retrieve configuration: %v", err),
		})
	}

	if yamlData == nil {
		// This might happen if the file didn't exist on initial load and hasn't been set yet
		h.logger.Warnf("Teleop config file exists but content is empty or initial load failed.")
		return c.Status(http.StatusNotFound).JSON(fiber.Map{
			"error": "Teleop configuration not found or not yet set.",
		})
	}

	// Set content type to YAML
	c.Set(fiber.HeaderContentType, "application/x-yaml")
	return c.Send(yamlData)
}

// handleUpdateTeleopConfig handles PUT requests to update the teleop config YAML.
func (h *ConfigHandler) handleUpdateTeleopConfig(c *fiber.Ctx) error {
	h.logger.Debugf("Handling PUT request for /api/v1/config/teleop")

	// Check content type - expect YAML
	if c.Get(fiber.HeaderContentType) != "application/x-yaml" && c.Get(fiber.HeaderContentType) != "application/yaml" && c.Get(fiber.HeaderContentType) != "text/yaml" {
		h.logger.Warnf("Received PUT request with incorrect Content-Type: %s", c.Get(fiber.HeaderContentType))
		// Relaxed check, let's try to process anyway but log warning. Stricter check might be needed.
		// return c.Status(http.StatusUnsupportedMediaType).JSON(fiber.Map{
		// 	"error": "Invalid Content-Type. Expected application/x-yaml or similar.",
		// })
	}

	// Get the raw body content (byte slice)
	newConfigYAML := c.Body()
	if len(newConfigYAML) == 0 {
		h.logger.Errorf("Received empty body in PUT request for teleop config update.")
		return c.Status(http.StatusBadRequest).JSON(fiber.Map{
			"error": "Request body cannot be empty.",
		})
	}

	// Call the service to update the configuration
	err := h.configService.UpdateConfig(newConfigYAML)
	if err != nil {
		h.logger.Errorf("Failed to update teleop configuration: %v", err)
		// Check for specific error types (e.g., validation error vs. internal error)
		// For now, return a generic bad request or internal server error depending on the assumed cause
		// If the error message contains "validation failed" or "invalid YAML", assume bad request
		if _, ok := err.(interface{ IsValidationError() bool }); ok || err.Error() == "invalid YAML format" || err.Error() == "validation failed: missing required fields (ConfigID, Version, RobotID)" {
			// Simplified check based on potential error messages from config_service
			return c.Status(http.StatusBadRequest).JSON(fiber.Map{
				"error": fmt.Sprintf("Configuration update failed: %v", err),
			})
		} else {
			// Assume other errors are internal (e.g., file write error)
			return c.Status(http.StatusInternalServerError).JSON(fiber.Map{
				"error": fmt.Sprintf("Internal server error during configuration update: %v", err),
			})
		}
	}

	h.logger.Infof("Successfully processed PUT request to update teleop configuration.")
	return c.Status(http.StatusOK).JSON(fiber.Map{
		"message": "Teleop configuration updated successfully. Note: Gateway may require restart.",
	})
}
