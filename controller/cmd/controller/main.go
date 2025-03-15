package main

import (
	"context"
	"log"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/gofiber/fiber/v2"
	"github.com/gofiber/fiber/v2/middleware/logger"
	"github.com/gofiber/fiber/v2/middleware/recover"
	// Import domain services
	"github.com/open-teleop/controller/domain/diagnostic"
	// "github.com/open-teleop/controller/domain/teleop"
	// "github.com/open-teleop/controller/domain/video"
	// "github.com/open-teleop/controller/domain/audio"
	// "github.com/open-teleop/controller/domain/sensor"
	// "github.com/open-teleop/controller/domain/navigation"
)

func main() {
	// Create a new Fiber app
	app := fiber.New(fiber.Config{
		AppName:      "Open-Teleop Controller",
		ErrorHandler: customErrorHandler,
	})

	// Add middleware
	app.Use(logger.New())
	app.Use(recover.New())

	// Get port from environment variable or use default
	port := os.Getenv("PORT")
	if port == "" {
		port = "8080"
	}

	// Initialize domain services
	diagnosticService := diagnostic.NewDiagnosticService()
	// Start listening for metrics from ROS2 bridge
	if err := diagnosticService.StartMetricsListener(); err != nil {
		log.Printf("Warning: Failed to start metrics listener: %v\n", err)
	}
	
	// teleopService := teleop.NewTeleopService()
	// videoService := video.NewVideoService()
	// audioService := audio.NewAudioService()
	// sensorService := sensor.NewSensorService()
	// navigationService := navigation.NewNavigationService()

	// Set up basic routes
	app.Get("/", func(c *fiber.Ctx) error {
		return c.JSON(fiber.Map{
			"status":  "online",
			"service": "open-teleop controller",
		})
	})

	// Health check endpoint
	app.Get("/health", func(c *fiber.Ctx) error {
		return c.JSON(fiber.Map{"status": "healthy"})
	})
	
	// Set up API routes
	api := app.Group("/api")
	
	// Diagnostic routes
	diagnosticRoutes := api.Group("/diagnostics")
	diagnosticRoutes.Get("/", diagnosticService.GetMetricsHandler)

	// TODO: Set up other domain service routes
	// Example:
	// teleop := api.Group("/teleop")
	// teleop.Post("/command", teleopService.CommandHandler)
	
	// video := api.Group("/video")
	// video.Get("/stream", videoService.StreamHandler)

	// Start server in a goroutine
	go func() {
		log.Printf("Server starting on port %s\n", port)
		if err := app.Listen(":" + port); err != nil {
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