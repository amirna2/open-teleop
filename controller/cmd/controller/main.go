package main

import (
	"context" // Added for WebSocket message parsing
	"errors"
	"flag"
	"fmt"
	stdlog "log"
	"net/http"
	"os"
	"os/signal"
	"path/filepath" // Added for environment normalization
	"syscall"
	"time"

	// Added for WebSocket support
	"github.com/gofiber/contrib/websocket"
	"github.com/gofiber/fiber/v2"
	fiberlogger "github.com/gofiber/fiber/v2/middleware/logger"
	"github.com/gofiber/fiber/v2/middleware/recover"

	// Import our configuration package (includes bootstrap and operational config)
	"github.com/open-teleop/controller/pkg/config"
	// Import our new logger package
	customlog "github.com/open-teleop/controller/pkg/log"
	// Import our processing components
	"github.com/open-teleop/controller/pkg/processing"
	// Import our ROS parser
	"github.com/open-teleop/controller/pkg/rosparser"
	// Import our ZeroMQ server
	"github.com/open-teleop/controller/pkg/zeromq"
	// Import our new API handlers
	"github.com/open-teleop/controller/pkg/api"
)

func main() {
	// Parse command line flags for initial setup
	configDir := flag.String("config-dir", "./config", "Path to configuration directory containing controller_config.yaml")
	flag.Parse()

	// --- Find executable directory for relative paths ---
	execPath, err := os.Executable()
	if err != nil {
		stdlog.Fatalf("Error determining executable path: %v", err)
	}
	execDir := filepath.Dir(execPath)

	// --- Resolve config directory ---
	// If config-dir is relative, make it relative to the executable or project structure
	if !filepath.IsAbs(*configDir) {
		if *configDir == "./config" {
			// Special case for default, try to find config relative to project root
			if _, err := os.Stat("./config"); os.IsNotExist(err) {
				parentDir := filepath.Dir(execDir)
				candidateConfigDir := filepath.Join(parentDir, "config")
				if _, err := os.Stat(candidateConfigDir); err == nil {
					*configDir = candidateConfigDir
				} else {
					gopath := os.Getenv("GOPATH")
					if gopath != "" {
						srcDir := filepath.Join(gopath, "src", "github.com", "open-teleop", "controller")
						candidateConfigDir = filepath.Join(srcDir, "..", "config") // Assuming config is one level up from 'controller'
						if _, err := os.Stat(candidateConfigDir); err == nil {
							*configDir = candidateConfigDir
						}
					}
				}
				if _, err := os.Stat(*configDir); os.IsNotExist(err) {
					stdlog.Fatalf("Could not find config directory. Checked ./config, %s, and GOPATH relative path. Please specify with --config-dir.", filepath.Join(filepath.Dir(execDir), "config"))
				}
			}
		} else {
			*configDir = filepath.Join(execDir, *configDir)
		}
	}
	stdlog.Printf("Using configuration directory: %s", *configDir) // Log resolved path

	// --- Load Bootstrap Configuration ---
	bootstrapCfg, err := config.LoadBootstrapConfig(*configDir)
	if err != nil {
		stdlog.Fatalf("Failed to load bootstrap configuration: %v", err)
	}

	// --- Initialize Custom Logger ---
	// Determine final log level (Bootstrap Config ONLY)
	finalLogLevel := bootstrapCfg.Logging.Level

	// Determine final log path (Bootstrap Config ONLY)
	finalLogPath := bootstrapCfg.Logging.LogPath // Get path from bootstrap config value
	if finalLogPath == "" {
		stdlog.Printf("Log path not specified in bootstrap config. Disabling file logging.")
	} else {
		stdlog.Printf("Using log directory: %s (from bootstrap config)", finalLogPath)
	}

	// Initialize logger with determined level and path
	logger, err := customlog.NewLogrusLogger(finalLogLevel, finalLogPath)
	if err != nil {
		stdlog.Fatalf("Failed to initialize logger: %v", err) // Use standard logger for this critical error
	}
	logger.Infof("Logger initialized: Level=%s, Directory='%s'", finalLogLevel, finalLogPath)

	// --- Load Operational Configuration (open_teleop_config.yaml) ---
	// Expect the operational config file to be in the same directory as the bootstrap config.
	operationalConfigPath := filepath.Join(*configDir, bootstrapCfg.Data.TeleopConfigFilename)
	logger.Infof("Attempting to load operational configuration from: %s", operationalConfigPath)

	cfg, err := config.LoadConfig(operationalConfigPath) // Use the basic loader
	if err != nil {
		logger.Fatalf("Failed to load operational configuration: %v", err)
	}

	// Log the configuration details (using operational config 'cfg')
	logger.Infof("Loaded operational configuration")
	logger.Infof("Operational Config ID: %s, Version: %s", cfg.ConfigID, cfg.Version)
	logger.Infof("Robot ID: %s", cfg.RobotID)

	// Log ZeroMQ settings (using bootstrap config for binds, operational for connect/subscribe)
	logger.Infof("ZeroMQ Request Bind Address: %s (from bootstrap config)", bootstrapCfg.ZeroMQ.RequestBindAddress)
	logger.Infof("ZeroMQ Publish Bind Address: %s (from bootstrap config)", bootstrapCfg.ZeroMQ.PublishBindAddress)
	logger.Infof("ZeroMQ Gateway Connect Address: %s (from operational config)", cfg.ZeroMQ.GatewayConnectAddress)
	logger.Infof("ZeroMQ Gateway Subscribe Address: %s (from operational config)", cfg.ZeroMQ.GatewaySubscribeAddress)

	// Log topic mappings summary (from operational config 'cfg')
	inboundTopics := cfg.GetTopicMappingsByDirection("INBOUND")
	outboundTopics := cfg.GetTopicMappingsByDirection("OUTBOUND")
	logger.Infof("Loaded %d topic mappings (%d inbound, %d outbound)",
		len(cfg.TopicMappings), len(inboundTopics), len(outboundTopics))

	// --- Initialize ZeroMQ service ---
	zmqService, err := zeromq.NewZeroMQService(
		bootstrapCfg.ZeroMQ.RequestBindAddress, // Use bootstrap config for bind address
		bootstrapCfg.ZeroMQ.PublishBindAddress, // Use bootstrap config for bind address
		cfg.ZeroMQ,                             // Pass operational ZMQ config for other settings if needed
		logger,
	)
	if err != nil {
		logger.Fatalf("Failed to initialize ZeroMQ service: %v", err)
	}

	// --- Initialize Topic Registry (uses operational config 'cfg') ---
	logger.Infof("Initializing Topic Registry")
	topicRegistry := processing.NewTopicRegistry(logger)
	topicRegistry.LoadFromConfig(cfg)

	// --- Initialize ROS Parser ---
	logger.Infof("Initializing ROS Parser")
	// Set the logger for rosparser package
	rosparser.SetLogger(logger)
	if err := rosparser.Initialize(); err != nil {
		logger.Fatalf("Failed to initialize ROS parser: %v", err)
	}
	defer rosparser.Shutdown()

	// --- Create ROS Message Processor (uses operational config 'cfg' via TopicRegistry) ---
	logger.Infof("Creating ROS Message Processor")
	rosProcessor := processing.NewRosMessageProcessor(logger, topicRegistry)

	// --- Create Result Handler ---
	logger.Infof("Creating Result Handler")
	resultHandler := processing.NewLoggingResultHandler(logger, zmqService)

	// --- Initialize and configure Message Director (uses operational config 'cfg') ---
	logger.Infof("Initializing Message Director")
	directorOptions := &processing.DirectorOptions{
		DefaultQueueSize: 1000, // TODO: Consider making this configurable (perhaps via bootstrap?)
	}
	messageDirector := processing.NewMessageDirector(cfg, logger, topicRegistry, directorOptions)
	// Initialize with worker counts from bootstrap config
	messageDirector.Initialize(
		bootstrapCfg.Processing.HighPriorityWorkers,
		bootstrapCfg.Processing.StandardPriorityWorkers,
		bootstrapCfg.Processing.LowPriorityWorkers,
	)

	// Set processor and result handler
	messageDirector.SetProcessor(rosProcessor.CreateProcessorFunc())
	messageDirector.SetResultHandler(resultHandler.CreateHandlerFunc())

	// Connect Message Director to ZeroMQ service
	zmqService.SetMessageDirector(messageDirector)

	// Start the Message Director
	messageDirector.Start()

	// --- Register configuration handlers and publisher (uses operational config 'cfg') ---
	configPublisher := zeromq.RegisterConfigHandlers(zmqService, cfg, logger)

	// --- Start the ZeroMQ service ---
	if err := zmqService.Start(); err != nil {
		logger.Fatalf("Failed to start ZeroMQ service: %v", err)
	}
	logger.Infof("ZeroMQ service started successfully")

	// Publish initial config notification
	if err := configPublisher.PublishConfigUpdatedNotification(); err != nil {
		logger.Warnf("Warning: Failed to publish config notification: %v", err)
	}

	// --- Create Fiber app (HTTP Server) ---
	app := fiber.New(fiber.Config{
		AppName:      "Open-Teleop Controller",
		ErrorHandler: customErrorHandler,
		// Use operational config for server behavior settings
		ReadTimeout: time.Duration(cfg.Controller.Server.RequestTimeout) * time.Second,
		BodyLimit:   cfg.Controller.Server.MaxRequestSize * 1024 * 1024, // Convert MB to bytes
		IdleTimeout: 7 * time.Second,                                    // Add idle timeout slightly > shutdown timeout
	})

	// Add middleware
	app.Use(fiberlogger.New())
	app.Use(recover.New())

	// --- Serve Static Files (Web UI) first ---
	// Serve index.html from ./web/static for the root path
	// Path is relative to the project root, as run_controller.sh executes from there.
	app.Static("/", "controller/web/static", fiber.Static{
		Index:         "index.html",
		CacheDuration: 1 * time.Second, // Disable caching for dev
	})

	// --- Set up basic API routes (uses operational config 'cfg') ---
	app.Get("/api/status", func(c *fiber.Ctx) error { // Changed route from / to /api/status
		return c.JSON(fiber.Map{
			"status":    "online",
			"version":   cfg.Version,
			"config_id": cfg.ConfigID,
			"robot_id":  cfg.RobotID,
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
			"robot_id":       cfg.RobotID,
			"topic_count":    len(cfg.TopicMappings),
			"inbound_count":  len(inboundTopics),
			"outbound_count": len(outboundTopics),
		})
	})

	// --- WebSocket Route for Control ---
	app.Use("/ws", func(c *fiber.Ctx) error { // Middleware to check if it's a WebSocket upgrade request
		if websocket.IsWebSocketUpgrade(c) {
			c.Locals("allowed", true)
			return c.Next()
		}
		logger.Warnf("Non-websocket request to /ws endpoint from %s", c.IP())
		return fiber.ErrUpgradeRequired
	})

	app.Get("/ws/control", websocket.New(func(conn *websocket.Conn) {
		// Call the handler from the api package, passing dependencies
		api.ControlWebSocketHandler(conn, logger, messageDirector, topicRegistry)
	}))

	// --- Start HTTP server in a goroutine ---
	go func() {
		port := bootstrapCfg.Server.HTTPPort // Use bootstrap config for server port
		logger.Infof("Starting HTTP server on port %d", port)
		if err := app.Listen(fmt.Sprintf(":%d", port)); err != nil {
			// Check if the error is due to server shutting down, which is expected
			if err != http.ErrServerClosed {
				logger.Fatalf("Server error: %v", err)
			}
		}
	}()

	// --- Wait for termination signal ---
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	logger.Infof("Shutting down server...")

	// --- Stop components ---
	messageDirector.Stop()
	zmqService.Stop()

	// --- Shutdown HTTP server with timeout ---
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	if err := app.ShutdownWithContext(ctx); err != nil {
		logger.Fatalf("Server shutdown failed: %v", err)
	}

	logger.Infof("Server gracefully stopped")
}

// customErrorHandler handles errors in a structured way
// TODO: Consider injecting the logger here if more detailed error logging is needed
func customErrorHandler(c *fiber.Ctx, err error) error {
	// Default to 500 Internal Server Error
	code := fiber.StatusInternalServerError

	// Retrieve the custom error message
	var e *fiber.Error
	if errors.As(err, &e) {
		code = e.Code
	}

	// Log the error internally
	// Consider adding more context like request ID, user agent, etc.
	stdlog.Printf("Internal Error: %v, Path: %s", err, c.Path()) // Using stdlog temporarily

	// Send generic error message to client
	return c.Status(code).JSON(fiber.Map{
		"error":   true,
		"message": "Internal Server Error", // Avoid sending detailed errors to the client
	})
}
