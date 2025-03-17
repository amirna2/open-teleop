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
	// Import domain services
	// "github.com/open-teleop/controller/domain/diagnostic"
	// "github.com/open-teleop/controller/domain/teleop"
	// "github.com/open-teleop/controller/domain/video"
	// "github.com/open-teleop/controller/domain/audio"
	// "github.com/open-teleop/controller/domain/sensor"
	// "github.com/open-teleop/controller/domain/navigation"
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

	log.Printf("Loaded configuration for environment: %s", cfg.Environment)
	log.Printf("ZeroMQ receiver address: %s", cfg.ZeroMQ.ReceiverAddress)
	log.Printf("ZeroMQ publisher address: %s", cfg.ZeroMQ.PublisherAddress)

	// Create a new Fiber app
	app := fiber.New(fiber.Config{
		AppName:      "Open-Teleop Controller",
		ErrorHandler: customErrorHandler,
		// Use config for server settings
		ReadTimeout:  time.Duration(cfg.Server.RequestTimeout) * time.Second,
		BodyLimit:    cfg.Server.MaxRequestSize * 1024 * 1024, // Convert MB to bytes
	})

	// Add middleware
	app.Use(logger.New())
	app.Use(recover.New())

	// Initialize services
	// TODO: Initialize ZeroMQ server and other service components with config

	// Set up basic routes
	app.Get("/", func(c *fiber.Ctx) error {
		return c.JSON(fiber.Map{
			"status":      "online",
			"service":     "open-teleop controller",
			"environment": cfg.Environment,
			"version":     cfg.Version,
		})
	})

	// Health check endpoint
	app.Get("/health", func(c *fiber.Ctx) error {
		return c.JSON(fiber.Map{
			"status": "healthy",
			"config": map[string]interface{}{
				"environment":     cfg.Environment,
				"zmq_receiver":    cfg.ZeroMQ.ReceiverAddress,
				"zmq_publisher":   cfg.ZeroMQ.PublisherAddress,
				"high_workers":    cfg.Processing.HighPriorityWorkers,
				"standard_workers": cfg.Processing.StandardPriorityWorkers,
				"low_workers":     cfg.Processing.LowPriorityWorkers,
			},
		})
	})
	
	// Set up API routes
	// Commented out for now until we implement service routes
	// api := app.Group("/api")
	
	// TODO: Set up service routes
	// Example:
	// teleop := api.Group("/teleop")
	// teleop.Post("/command", teleopService.CommandHandler)

	// Start server in a goroutine
	serverPort := fmt.Sprintf(":%d", cfg.Server.Port)
	go func() {
		log.Printf("Server starting on port %d\n", cfg.Server.Port)
		if err := app.Listen(serverPort); err != nil {
			log.Fatalf("Failed to start server: %v\n", err)
		}
	}()

	// Set up graceful shutdown
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit
	log.Println("Shutting down server...")

	// Create context with timeout for shutdown
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	// Attempt graceful shutdown
	if err := app.ShutdownWithContext(ctx); err != nil {
		log.Fatalf("Server forced to shutdown: %v\n", err)
	}

	log.Println("Server exited properly")
}

// Custom error handler
func customErrorHandler(c *fiber.Ctx, err error) error {
	// Default 500 status code
	code := fiber.StatusInternalServerError

	// Check if it's a Fiber error
	if e, ok := err.(*fiber.Error); ok {
		code = e.Code
	}

	// Return JSON response
	return c.Status(code).JSON(fiber.Map{
		"error": err.Error(),
	})
} 