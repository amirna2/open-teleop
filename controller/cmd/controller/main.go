package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"path/filepath"
	"syscall"
	"time"

	"github.com/gofiber/fiber/v2"
	"github.com/gofiber/fiber/v2/middleware/logger"
	"github.com/gofiber/fiber/v2/middleware/recover"

	// Import our configuration package
	"github.com/open-teleop/controller/pkg/config"
	// Import our ZeroMQ server
	"github.com/open-teleop/controller/pkg/zeromq"
)

func main() {
	// Parse command line flags
	environment := flag.String("env", "", "Environment (development, testing, production)")
	configDir := flag.String("config-dir", "./config", "Path to configuration directory")
	flag.Parse()

	// Find executable directory for relative paths
	execPath, err := os.Executable()
	if err != nil {
		log.Fatalf("Error determining executable path: %v", err)
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
				} else {
					// Try relative to GOPATH
					gopath := os.Getenv("GOPATH")
					if gopath != "" {
						srcDir := filepath.Join(gopath, "src", "github.com", "open-teleop", "controller")
						candidateConfigDir = filepath.Join(srcDir, "..", "config")
						if _, err := os.Stat(candidateConfigDir); err == nil {
							*configDir = candidateConfigDir
						}
					}
				}
			}
		} else {
			*configDir = filepath.Join(execDir, *configDir)
		}
	}

	// Load configuration
	log.Printf("Loading configuration from %s for environment %s", *configDir, *environment)
	cfg, err := config.LoadConfigWithEnv(*configDir, *environment)
	if err != nil {
		log.Fatalf("Failed to load configuration: %v", err)
	}

	// Log the configuration
	log.Printf("Loaded configuration for environment: %s", cfg.Environment)
	log.Printf("Configuration ID: %s, Version: %s", cfg.ConfigID, cfg.Version)
	log.Printf("Robot ID: %s", cfg.RobotID)

	// Log ZeroMQ settings
	log.Printf("ZeroMQ controller address: %s", cfg.ZeroMQ.ControllerAddress)
	log.Printf("ZeroMQ gateway address: %s", cfg.ZeroMQ.GatewayAddress)

	// Log topic mappings summary
	inboundTopics := cfg.GetTopicMappingsByDirection("INBOUND")
	outboundTopics := cfg.GetTopicMappingsByDirection("OUTBOUND")
	log.Printf("Loaded %d topic mappings (%d inbound, %d outbound)",
		len(cfg.TopicMappings), len(inboundTopics), len(outboundTopics))

	// Initialize ZeroMQ service
	zmqService, err := zeromq.NewZeroMQService(cfg, log.Default())
	if err != nil {
		log.Fatalf("Failed to initialize ZeroMQ service: %v", err)
	}

	// Register configuration handlers and publisher
	configPublisher := zeromq.RegisterConfigHandlers(zmqService, cfg, log.Default())

	// Start the ZeroMQ service
	if err := zmqService.Start(); err != nil {
		log.Fatalf("Failed to start ZeroMQ service: %v", err)
	}
	log.Println("ZeroMQ service started successfully")

	// Publish initial config notification
	if err := configPublisher.PublishConfigUpdatedNotification(); err != nil {
		log.Printf("Warning: Failed to publish config notification: %v", err)
	}

	// Create a new Fiber app
	app := fiber.New(fiber.Config{
		AppName:      "Open-Teleop Controller",
		ErrorHandler: customErrorHandler,
		// Use config for server settings
		ReadTimeout: time.Duration(cfg.Controller.Server.RequestTimeout) * time.Second,
		BodyLimit:   cfg.Controller.Server.MaxRequestSize * 1024 * 1024, // Convert MB to bytes
	})

	// Add middleware
	app.Use(logger.New())
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
		log.Printf("Starting HTTP server on port %d", port)
		if err := app.Listen(fmt.Sprintf(":%d", port)); err != nil {
			log.Fatalf("Server error: %v", err)
		}
	}()

	// Wait for termination signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	log.Println("Shutting down server...")

	// Stop the ZeroMQ server
	zmqService.Stop()

	// Shutdown with timeout
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	if err := app.ShutdownWithContext(ctx); err != nil {
		log.Fatalf("Server shutdown failed: %v", err)
	}

	log.Println("Server gracefully stopped")
}

// customErrorHandler handles errors in a structured way
func customErrorHandler(c *fiber.Ctx, err error) error {
	// Default to 500 Internal Server Error
	code := fiber.StatusInternalServerError

	// Check if it's a Fiber error
	if e, ok := err.(*fiber.Error); ok {
		code = e.Code
	}

	// Log the error
	log.Printf("HTTP Error [%d]: %v", code, err)

	// Return JSON error response
	return c.Status(code).JSON(fiber.Map{
		"error":   true,
		"message": err.Error(),
		"status":  code,
	})
}
