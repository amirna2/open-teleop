package main

import (
	"context"
	"flag"
	"fmt"
	stdlog "log" // Alias standard logger
	"net/http"
	"os"
	"os/signal"
	"path/filepath"
	"syscall"
	"time"

	"github.com/gofiber/fiber/v2"
	fiberlogger "github.com/gofiber/fiber/v2/middleware/logger"
	"github.com/gofiber/fiber/v2/middleware/recover"

	// Import our configuration package
	"github.com/open-teleop/controller/pkg/config"
	// Import our new logger package
	customlog "github.com/open-teleop/controller/pkg/log"
	// Import our processing components
	"github.com/open-teleop/controller/pkg/processing"
	// Import our ROS parser
	"github.com/open-teleop/controller/pkg/rosparser"
	// Import our ZeroMQ server
	"github.com/open-teleop/controller/pkg/zeromq"
)

func main() {
	// Parse command line flags
	environment := flag.String("env", "", "Environment (development, testing, production)")
	configDir := flag.String("config-dir", "./config", "Path to configuration directory")
	logLevel := flag.String("log-level", "info", "Logging level (debug, info, warn, error, fatal)")
	logDir := flag.String("log-dir", "./logs/controller", "Directory to store log files (empty to disable file logging)")
	flag.Parse()

	// --- Initialize custom logger ---
	logger, err := customlog.NewLogrusLogger(*logLevel, *logDir)
	if err != nil {
		stdlog.Fatalf("Failed to initialize logger: %v", err) // Use standard logger for this critical error
	}
	logger.Infof("Logger initialized: Level=%s, Directory='%s'", *logLevel, *logDir)
	// --- End logger initialization ---

	// Find executable directory for relative paths
	execPath, err := os.Executable()
	if err != nil {
		logger.Fatalf("Error determining executable path: %v", err) // Use new logger
	}
	execDir := filepath.Dir(execPath)

	// If config-dir is relative, make it relative to the executable
	if !filepath.IsAbs(*configDir) {
		if *configDir == "./config" {
			// Special case for default, try to find config relative to project root
			// Check if "config" exists in current directory
			if _, err := os.Stat("./config"); os.IsNotExist(err) {
				// Try parent directory of executable
				parentDir := filepath.Dir(execDir)
				candidateConfigDir := filepath.Join(parentDir, "config")
				if _, err := os.Stat(candidateConfigDir); err == nil {
					*configDir = candidateConfigDir
					logger.Debugf("Using config directory found relative to parent: %s", *configDir)
				} else {
					logger.Debugf("Config directory not found in parent, checking GOPATH...")
					// Try relative to GOPATH
					gopath := os.Getenv("GOPATH")
					if gopath != "" {
						srcDir := filepath.Join(gopath, "src", "github.com", "open-teleop", "controller")
						candidateConfigDir = filepath.Join(srcDir, "..", "config")
						if _, err := os.Stat(candidateConfigDir); err == nil {
							*configDir = candidateConfigDir
							logger.Debugf("Using config directory found relative to GOPATH: %s", *configDir)
						}
					}
				}
			}
		} else {
			*configDir = filepath.Join(execDir, *configDir)
			logger.Debugf("Resolved relative config directory to: %s", *configDir)
		}
	}

	// Load configuration
	logger.Infof("Loading configuration from %s for environment '%s'", *configDir, *environment)
	cfg, err := config.LoadConfigWithEnv(*configDir, *environment)
	if err != nil {
		logger.Fatalf("Failed to load configuration: %v", err) // Use new logger
	}

	// Log the configuration
	logger.Infof("Loaded configuration for environment: %s", cfg.Environment)
	logger.Infof("Configuration ID: %s, Version: %s", cfg.ConfigID, cfg.Version)
	logger.Infof("Robot ID: %s", cfg.RobotID)

	// Log ZeroMQ settings
	logger.Infof("ZeroMQ controller address: %s", cfg.ZeroMQ.ControllerAddress)
	logger.Infof("ZeroMQ gateway address: %s", cfg.ZeroMQ.GatewayAddress)

	// Log topic mappings summary
	inboundTopics := cfg.GetTopicMappingsByDirection("INBOUND")
	outboundTopics := cfg.GetTopicMappingsByDirection("OUTBOUND")
	logger.Infof("Loaded %d topic mappings (%d inbound, %d outbound)",
		len(cfg.TopicMappings), len(inboundTopics), len(outboundTopics))

	// Initialize ZeroMQ service
	// *** Pass the custom logger instance now ***
	zmqService, err := zeromq.NewZeroMQService(cfg, logger)
	if err != nil {
		logger.Fatalf("Failed to initialize ZeroMQ service: %v", err) // Use new logger
	}

	// Initialize Topic Registry
	logger.Infof("Initializing Topic Registry")
	topicRegistry := processing.NewTopicRegistry(logger)
	topicRegistry.LoadFromConfig(cfg)

	// Initialize ROS Parser
	logger.Infof("Initializing ROS Parser")
	if err := rosparser.Initialize(); err != nil {
		logger.Fatalf("Failed to initialize ROS parser: %v", err)
	}
	defer rosparser.Shutdown()

	// Create ROS Message Processor
	logger.Infof("Creating ROS Message Processor")
	rosProcessor := processing.NewRosMessageProcessor(logger, topicRegistry)

	// Create Result Handler
	logger.Infof("Creating Result Handler")
	resultHandler := processing.NewLoggingResultHandler(logger)

	// Initialize and configure Message Director
	logger.Infof("Initializing Message Director")
	directorOptions := &processing.DirectorOptions{
		DefaultQueueSize: 1000, // Set a reasonable queue size
	}
	messageDirector := processing.NewMessageDirector(cfg, logger, topicRegistry, directorOptions)
	messageDirector.Initialize(cfg)

	// Set processor and result handler
	messageDirector.SetProcessor(rosProcessor.CreateProcessorFunc())
	messageDirector.SetResultHandler(resultHandler.CreateHandlerFunc())

	// Connect Message Director to ZeroMQ service
	zmqService.SetMessageDirector(messageDirector)

	// Start the Message Director
	messageDirector.Start()

	// Register configuration handlers and publisher
	// *** Pass the custom logger instance now ***
	configPublisher := zeromq.RegisterConfigHandlers(zmqService, cfg, logger)

	// Start the ZeroMQ service
	if err := zmqService.Start(); err != nil {
		logger.Fatalf("Failed to start ZeroMQ service: %v", err) // Use new logger
	}
	logger.Infof("ZeroMQ service started successfully")

	// Publish initial config notification
	if err := configPublisher.PublishConfigUpdatedNotification(); err != nil {
		logger.Warnf("Warning: Failed to publish config notification: %v", err) // Use new logger
	}

	// Create a new Fiber app
	app := fiber.New(fiber.Config{
		AppName:      "Open-Teleop Controller",
		ErrorHandler: customErrorHandler, // Keep custom error handler
		// Use config for server settings
		ReadTimeout: time.Duration(cfg.Controller.Server.RequestTimeout) * time.Second,
		BodyLimit:   cfg.Controller.Server.MaxRequestSize * 1024 * 1024, // Convert MB to bytes
	})

	// Add middleware
	app.Use(fiberlogger.New())
	app.Use(recover.New())

	// Set up basic routes
	app.Get("/", func(c *fiber.Ctx) error {
		return c.JSON(fiber.Map{
			"status":      "online",
			"version":     cfg.Version,
			"environment": cfg.Environment,
			"config_id":   cfg.ConfigID,
			"robot_id":    cfg.RobotID,
		})
	})

	app.Get("/health", func(c *fiber.Ctx) error {
		return c.JSON(fiber.Map{
			"status": "healthy",
		})
	})

	app.Get("/config", func(c *fiber.Ctx) error {
		return c.JSON(fiber.Map{
			"config_id":      cfg.ConfigID,
			"version":        cfg.Version,
			"last_updated":   cfg.LastUpdated,
			"environment":    cfg.Environment,
			"robot_id":       cfg.RobotID,
			"topic_count":    len(cfg.TopicMappings),
			"inbound_count":  len(inboundTopics),
			"outbound_count": len(outboundTopics),
		})
	})

	// Start server in a goroutine
	go func() {
		port := cfg.Controller.Server.Port
		logger.Infof("Starting HTTP server on port %d", port) // Use new logger
		if err := app.Listen(fmt.Sprintf(":%d", port)); err != nil {
			// Check if the error is due to server shutting down, which is expected
			if err != http.ErrServerClosed { // You might need to import "net/http"
				logger.Fatalf("Server error: %v", err) // Use new logger
			}
		}
	}()

	// Wait for termination signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	logger.Infof("Shutting down server...")

	// Stop the Message Director
	messageDirector.Stop()

	// Stop the ZeroMQ server
	zmqService.Stop()

	// Shutdown with timeout
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	if err := app.ShutdownWithContext(ctx); err != nil {
		logger.Fatalf("Server shutdown failed: %v", err) // Use new logger
	}

	logger.Infof("Server gracefully stopped") // Corrected from Info to Infof
}

// customErrorHandler handles errors in a structured way
// TODO: Consider injecting the logger here if more detailed error logging is needed
func customErrorHandler(c *fiber.Ctx, err error) error {
	// Default to 500 Internal Server Error
	code := fiber.StatusInternalServerError

	// Check if it's a Fiber error
	if e, ok := err.(*fiber.Error); ok {
		code = e.Code
	}

	// Log the error using standard logger for now, until logger is injected
	// We could potentially retrieve the logger from the Fiber context if we set it up,
	// or make the logger a global variable (less ideal).
	// For now, keeping stdlog for this specific handler.
	stdlog.Printf("HTTP Error [%d]: %v", code, err)

	// Return JSON error response
	return c.Status(code).JSON(fiber.Map{
		"error":   true,
		"message": err.Error(),
		"status":  code,
	})
}
