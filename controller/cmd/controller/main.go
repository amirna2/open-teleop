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
	// Import our new services package
	"github.com/open-teleop/controller/services"
	// Import our video service
	"github.com/open-teleop/controller/domain/video"
)

// --- Global Variables for Flags (Best Practice) ---
var (
	configDir    *string
	teleopConfig *string
)

// --- Main Application Entry Point ---

func main() {
	// --- Parse Flags First ---
	parseFlags()

	// --- Resolve Config Directory ---
	absoluteConfigDir := resolveConfigDir(*configDir)

	// --- Load Bootstrap Config ---
	bootstrapCfg, err := config.LoadBootstrapConfig(absoluteConfigDir)
	if err != nil {
		stdlog.Fatalf("Failed to load bootstrap configuration: %v", err)
	}

	// --- Initialize Logger ---
	logger, err := initLogger(bootstrapCfg)
	if err != nil {
		stdlog.Fatalf("Failed to initialize logger: %v", err) // Use standard logger for this critical error
	}
	logger.Infof("Logger initialized.")

	// --- Load Initial Operational Config ---
	// Pass the potential override from the flag
	initialCfg, operationalConfigPath, err := loadInitialConfig(absoluteConfigDir, *teleopConfig, bootstrapCfg, logger)
	if err != nil {
		logger.Fatalf("Failed to load initial operational configuration: %v", err)
	}

	// --- Setup ZeroMQ Service ---
	zmqService, err := setupZMQ(bootstrapCfg, initialCfg, logger)
	if err != nil {
		logger.Fatalf("Failed to setup ZeroMQ service: %v", err)
	}

	// --- Setup Config Service, ZMQ Handler & Publisher ---
	teleopConfigService, configPublisher, err := setupConfigServiceAndHandlers(zmqService, operationalConfigPath, logger)
	if err != nil {
		logger.Fatalf("Failed to setup configuration service and handlers: %v", err)
	}

	// --- Get Managed Config (primary source for components) ---
	cfg := teleopConfigService.GetCurrentConfig()
	if cfg == nil {
		logger.Warnf("Config from TeleopConfigService is nil after initialization, using initial load data for component setup.")
		cfg = initialCfg // Fallback
	}
	logCoreConfigInfo(logger, cfg, bootstrapCfg)

	// --- Setup Processing Components ---
	messageDirector, topicRegistry, err := setupProcessingComponents(cfg, logger, zmqService, bootstrapCfg)
	if err != nil {
		logger.Fatalf("Failed to setup processing components: %v", err)
	}

	// --- Start ZeroMQ Service & Publish Initial Notification ---
	if err := zmqService.Start(); err != nil {
		logger.Fatalf("Failed to start ZeroMQ service: %v", err)
	}
	logger.Infof("ZeroMQ service started successfully")
	if err := configPublisher.PublishConfigUpdatedNotification(); err != nil {
		logger.Warnf("Warning: Failed to publish initial config notification: %v", err)
	}

	// --- Setup and Start HTTP Server ---
	app, err := setupHTTPServer(cfg, teleopConfigService, logger, messageDirector, topicRegistry)
	if err != nil {
		logger.Fatalf("Failed to setup HTTP server: %v", err)
	}
	go startHTTPServer(app, bootstrapCfg.Server.HTTPPort, logger)

	// --- Wait for Termination Signal & Graceful Shutdown ---
	waitForShutdownSignal(logger)
	performGracefulShutdown(app, messageDirector, zmqService, logger)

	logger.Infof("Controller shut down gracefully.")
}

// --- Helper Functions for Initialization ---

// parseFlags defines and parses command line flags.
func parseFlags() {
	configDir = flag.String("config-dir", "./config", "Path to configuration directory containing controller_config.yaml")
	teleopConfig = flag.String("teleop-config", "", "Path to the teleop configuration file (overrides bootstrap config)")
	flag.Parse()
}

// resolveConfigDir determines the absolute config path for the bootstrap file.
func resolveConfigDir(configDirFlagValue string) string {
	// This function remains largely the same, but now takes the flag value as input
	execPath, err := os.Executable()
	if err != nil {
		stdlog.Fatalf("Error determining executable path: %v", err)
	}
	execDir := filepath.Dir(execPath)

	absoluteConfigDir := configDirFlagValue
	if !filepath.IsAbs(absoluteConfigDir) {
		if absoluteConfigDir == "./config" {
			// Special case for default, try relative to executable first
			candidateDir := filepath.Join(execDir, "config")
			if _, err := os.Stat(candidateDir); err == nil {
				absoluteConfigDir = candidateDir
			} else {
				// Try relative to project root (assuming executable is in controller/bin or similar)
				parentDir := filepath.Dir(execDir)
				candidateDir = filepath.Join(parentDir, "config")
				if _, err := os.Stat(candidateDir); err == nil {
					absoluteConfigDir = candidateDir
				} else {
					// Try relative to parent's parent (e.g., if in controller/cmd/controller)
					grandparentDir := filepath.Dir(parentDir)
					candidateDir = filepath.Join(grandparentDir, "config")
					if _, err := os.Stat(candidateDir); err == nil {
						absoluteConfigDir = candidateDir
					} else {
						// Fallback: check current working directory as last resort for ./config
						if _, err := os.Stat("./config"); err == nil {
							absoluteConfigDir, _ = filepath.Abs("./config") // Use absolute path of cwd/config
						}
					}
				}
			}
		} else {
			// If a relative path other than ./config is given, assume it's relative to executable
			absoluteConfigDir = filepath.Join(execDir, absoluteConfigDir)
		}
	}

	// Final check if resolved directory exists
	if _, err := os.Stat(absoluteConfigDir); os.IsNotExist(err) {
		stdlog.Fatalf("Resolved configuration directory does not exist: %s", absoluteConfigDir)
	}

	stdlog.Printf("Using configuration directory: %s", absoluteConfigDir)
	return absoluteConfigDir
}

// initLogger initializes the custom logger based on bootstrap configuration.
func initLogger(bootstrapCfg *config.BootstrapConfig) (customlog.Logger, error) {
	finalLogLevel := bootstrapCfg.Logging.Level
	finalLogPath := bootstrapCfg.Logging.LogPath
	if finalLogPath == "" {
		stdlog.Printf("Log path not specified in bootstrap config. Disabling file logging.")
	} else {
		stdlog.Printf("Using log directory: %s (from bootstrap config)", finalLogPath)
	}
	logger, err := customlog.NewLogrusLogger(finalLogLevel, finalLogPath)
	if err != nil {
		return nil, fmt.Errorf("failed to initialize logger: %w", err)
	}
	logger.Infof("Logger initialized: Level=%s, Directory='%s'", finalLogLevel, finalLogPath)
	return logger, nil
}

// loadInitialConfig loads the initial operational configuration file.
// It now accepts the teleopConfigFlag value to check for an override.
func loadInitialConfig(resolvedConfigDir string, teleopConfigFlag string, bootstrapCfg *config.BootstrapConfig, logger customlog.Logger) (*config.Config, string, error) {
	var operationalConfigPath string

	if teleopConfigFlag != "" {
		logger.Infof("Using teleop configuration override from command line: %s", teleopConfigFlag)
		if !filepath.IsAbs(teleopConfigFlag) {
			var err error
			operationalConfigPath, err = filepath.Abs(teleopConfigFlag) // Resolve relative paths based on CWD
			if err != nil {
				return nil, teleopConfigFlag, fmt.Errorf("failed to resolve absolute path for -teleop-config '%s': %w", teleopConfigFlag, err)
			}
			logger.Infof("Resolved relative teleop config path to: %s", operationalConfigPath)
		} else {
			operationalConfigPath = teleopConfigFlag
		}
	} else {
		operationalConfigPath = filepath.Join(resolvedConfigDir, bootstrapCfg.Data.TeleopConfigFilename)
		logger.Infof("Using teleop configuration path from bootstrap config: %s", operationalConfigPath)
	}

	logger.Infof("Loading initial operational configuration from: %s", operationalConfigPath)
	initialCfg, err := config.LoadConfig(operationalConfigPath)
	if err != nil {
		return nil, operationalConfigPath, fmt.Errorf("failed to load initial operational configuration from %s: %w", operationalConfigPath, err)
	}
	if initialCfg == nil {
		return nil, operationalConfigPath, fmt.Errorf("initial operational configuration loaded as nil from %s", operationalConfigPath)
	}
	logger.Infof("Initial operational configuration loaded (ID: %s) for setup.", initialCfg.ConfigID)
	return initialCfg, operationalConfigPath, nil
}

// setupZMQ initializes the ZeroMQ service.
func setupZMQ(bootstrapCfg *config.BootstrapConfig, initialCfg *config.Config, logger customlog.Logger) (*zeromq.ZeroMQService, error) {
	logger.Infof("Initializing ZeroMQ service...")
	zmqService, err := zeromq.NewZeroMQService(
		bootstrapCfg.ZeroMQ.RequestBindAddress,
		bootstrapCfg.ZeroMQ.PublishBindAddress,
		initialCfg.ZeroMQ, // Pass initial operational ZMQ config if needed by constructor
		logger,
	)
	if err != nil {
		return nil, fmt.Errorf("failed to initialize ZeroMQ service: %w", err)
	}
	logger.Infof("ZeroMQ service initialized.")
	return zmqService, nil
}

// setupConfigServiceAndHandlers initializes the config service, registers ZMQ handler, creates publisher, and links them.
func setupConfigServiceAndHandlers(zmqService *zeromq.ZeroMQService, operationalConfigPath string, logger customlog.Logger) (services.TeleopConfigService, services.ConfigPublisher, error) {
	logger.Infof("Initializing Teleop Configuration Service for: %s", operationalConfigPath)
	teleopConfigService, err := services.NewTeleopConfigService(operationalConfigPath, logger)
	if err != nil {
		// Log warning but allow to proceed, service might recover or config can be set via API
		logger.Warnf("Error initializing TeleopConfigService: %v. Operational config might be missing initially.", err)
		// Return the partially initialized service anyway, but nil publisher and no error to allow proceeding
		// Caller must handle nil config from service later.
		// Alternatively, return the error: return nil, nil, fmt.Errorf("failed to initialize TeleopConfigService: %w", err)
	}

	logger.Infof("Registering ZeroMQ Config Request Handler...")
	zeromq.RegisterZmqConfigRequestHandler(zmqService, teleopConfigService, logger)

	logger.Infof("Creating ZeroMQ Config Publisher...")
	// Publisher needs a ConfigGetter, which TeleopConfigService implements.
	configPublisher := zeromq.NewConfigPublisher(zmqService, teleopConfigService, logger)

	logger.Infof("Injecting Publisher into Config Service...")
	teleopConfigService.SetPublisher(configPublisher)

	logger.Infof("Configuration Service and ZMQ components initialized and linked.")
	return teleopConfigService, configPublisher, nil // Return nil error even if service init had warning
}

// logCoreConfigInfo logs key details from the loaded configurations.
func logCoreConfigInfo(logger customlog.Logger, cfg *config.Config, bootstrapCfg *config.BootstrapConfig) {
	logger.Infof("Using configuration for setup: ID: %s, Version: %s", cfg.ConfigID, cfg.Version)
	logger.Infof("Robot ID: %s", cfg.RobotID)
	logger.Infof("ZeroMQ Request Bind Address: %s (from bootstrap config)", bootstrapCfg.ZeroMQ.RequestBindAddress)
	logger.Infof("ZeroMQ Publish Bind Address: %s (from bootstrap config)", bootstrapCfg.ZeroMQ.PublishBindAddress)
	logger.Infof("ZeroMQ Gateway Connect Address: %s (from operational config)", cfg.ZeroMQ.GatewayConnectAddress)
	logger.Infof("ZeroMQ Gateway Subscribe Address: %s (from operational config)", cfg.ZeroMQ.GatewaySubscribeAddress)
	inboundTopics := cfg.GetTopicMappingsByDirection("INBOUND")
	outboundTopics := cfg.GetTopicMappingsByDirection("OUTBOUND")
	logger.Infof("Loaded %d topic mappings (%d inbound, %d outbound)",
		len(cfg.TopicMappings), len(inboundTopics), len(outboundTopics))
}

// setupProcessingComponents initializes ROS parser, processing chain (registry, processor, handler, director).
func setupProcessingComponents(cfg *config.Config, logger customlog.Logger, zmqService *zeromq.ZeroMQService, bootstrapCfg *config.BootstrapConfig) (*processing.MessageDirector, *processing.TopicRegistry, error) {
	logger.Infof("Initializing Processing Components...")

	logger.Infof("Initializing Topic Registry")
	topicRegistry := processing.NewTopicRegistry(logger)
	topicRegistry.LoadFromConfig(cfg) // Load with managed config
	logger.Infof("Topic Registry initialized.")

	logger.Infof("Initializing ROS Parser")
	rosparser.SetLogger(logger)
	if err := rosparser.Initialize(); err != nil {
		return nil, nil, fmt.Errorf("failed to initialize ROS parser: %w", err)
	}
	// Note: defer rosparser.Shutdown() should be called in main near the end
	logger.Infof("ROS Parser initialized.")

	logger.Infof("Creating ROS Message Processor")
	rosProcessor := processing.NewRosMessageProcessor(logger, topicRegistry)
	logger.Infof("ROS Message Processor created.")

	logger.Infof("Creating Result Handler")
	resultHandler := processing.NewLoggingResultHandler(logger, zmqService)
	logger.Infof("Result Handler created.")

	logger.Infof("Creating Video Service")
	videoService := video.NewVideoService(logger)
	logger.Infof("Video Service created.")

	logger.Infof("Initializing Message Director")
	directorOptions := &processing.DirectorOptions{
		DefaultQueueSize: 1000, // TODO: Consider making this configurable (perhaps via bootstrap?)
	}
	messageDirector := processing.NewMessageDirector(cfg, logger, topicRegistry, directorOptions)
	messageDirector.Initialize(
		bootstrapCfg.Processing.HighPriorityWorkers,
		bootstrapCfg.Processing.StandardPriorityWorkers,
		bootstrapCfg.Processing.LowPriorityWorkers,
	)
	messageDirector.SetVideoService(videoService)
	messageDirector.SetProcessor(rosProcessor.CreateProcessorFunc())
	messageDirector.SetResultHandler(resultHandler.CreateHandlerFunc())
	zmqService.SetMessageDirector(messageDirector)
	zmqService.GetDispatcher().(*zeromq.MessageDispatcher).SetVideoService(videoService)
	messageDirector.Start()
	logger.Infof("Message Director initialized and started.")

	logger.Infof("Processing Components setup complete.")
	return messageDirector, topicRegistry, nil
}

// setupHTTPServer creates the Fiber app, adds middleware, and registers routes.
func setupHTTPServer(cfg *config.Config, teleopConfigService services.TeleopConfigService, logger customlog.Logger, messageDirector *processing.MessageDirector, topicRegistry *processing.TopicRegistry) (*fiber.App, error) {
	logger.Infof("Setting up HTTP Server (Fiber)...")
	app := fiber.New(fiber.Config{
		AppName:      "Open-Teleop Controller",
		ErrorHandler: customErrorHandler,
		ReadTimeout:  time.Duration(cfg.Controller.Server.RequestTimeout) * time.Second,
		BodyLimit:    cfg.Controller.Server.MaxRequestSize * 1024 * 1024, // Convert MB to bytes
		IdleTimeout:  7 * time.Second,
	})

	logger.Infof("Adding HTTP middleware...")
	app.Use(fiberlogger.New())
	app.Use(recover.New())

	logger.Infof("Registering static file serving...")
	// TODO: Make static path configurable?
	app.Static("/", "controller/web/static", fiber.Static{
		Index:         "index.html",
		CacheDuration: 0, // Completely disable caching for dev
	})

	logger.Infof("Registering API routes...")
	// Basic Status/Health Routes (using potentially initial config via cfg)
	app.Get("/api/status", func(c *fiber.Ctx) error {
		return c.JSON(fiber.Map{
			"status":    "online",
			"version":   cfg.Version,
			"config_id": cfg.ConfigID,
			"robot_id":  cfg.RobotID,
		})
	})
	app.Get("/health", func(c *fiber.Ctx) error {
		return c.JSON(fiber.Map{"status": "healthy"})
	})
	app.Get("/config", func(c *fiber.Ctx) error { // Old summary endpoint
		inbound := cfg.GetTopicMappingsByDirection("INBOUND")
		outbound := cfg.GetTopicMappingsByDirection("OUTBOUND")
		return c.JSON(fiber.Map{
			"config_id":      cfg.ConfigID,
			"version":        cfg.Version,
			"last_updated":   cfg.LastUpdated,
			"robot_id":       cfg.RobotID,
			"topic_count":    len(cfg.TopicMappings),
			"inbound_count":  len(inbound),
			"outbound_count": len(outbound),
		})
	})

	// New Teleop Config API Routes (using service for live data)
	api.RegisterConfigRoutes(app, teleopConfigService, logger)

	// WebSocket Route (middleware for /ws)
	app.Use("/ws", func(c *fiber.Ctx) error {
		if websocket.IsWebSocketUpgrade(c) {
			c.Locals("allowed", true)
			return c.Next()
		}
		logger.Warnf("Non-websocket request to /ws endpoint from %s", c.IP())
		return fiber.ErrUpgradeRequired
	})

	// Control WebSocket endpoint
	app.Get("/ws/control", websocket.New(func(conn *websocket.Conn) {
		api.ControlWebSocketHandler(conn, logger, messageDirector, topicRegistry)
	}))

	// Video WebSocket endpoint
	app.Get("/ws/video", websocket.New(func(conn *websocket.Conn) {
		api.VideoWebSocketHandler(conn, logger, messageDirector, topicRegistry)
	}))

	logger.Infof("HTTP Server setup complete.")
	return app, nil
}

// startHTTPServer starts the Fiber app listener.
func startHTTPServer(app *fiber.App, port int, logger customlog.Logger) {
	logger.Infof("Starting HTTP server on port %d", port)
	if err := app.Listen(fmt.Sprintf(":%d", port)); err != nil {
		if !errors.Is(err, http.ErrServerClosed) {
			logger.Fatalf("HTTP Server Listen error: %v", err)
		}
	}
	logger.Infof("HTTP server listener stopped.")
}

// waitForShutdownSignal blocks until SIGINT or SIGTERM is received.
func waitForShutdownSignal(logger customlog.Logger) {
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	receivedSignal := <-quit
	logger.Infof("Received termination signal: %v. Shutting down...", receivedSignal)
}

// performGracefulShutdown handles the shutdown of components.
func performGracefulShutdown(app *fiber.App, messageDirector *processing.MessageDirector, zmqService *zeromq.ZeroMQService, logger customlog.Logger) {
	logger.Infof("Performing graceful shutdown...")
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second) // Timeout for shutdown
	defer cancel()

	// Shutdown Fiber app first
	logger.Infof("Shutting down HTTP server...")
	if err := app.ShutdownWithContext(ctx); err != nil {
		logger.Errorf("Fiber app shutdown error: %v", err)
	}

	// Stop the message director
	logger.Infof("Stopping Message Director...")
	messageDirector.Stop()

	// Stop the ZeroMQ service
	logger.Infof("Stopping ZeroMQ Service...")
	zmqService.Stop()

	// Shutdown ROS parser (deferred in main originally, called here explicitly for order)
	logger.Infof("Shutting down ROS Parser...")
	rosparser.Shutdown()

	logger.Infof("Graceful shutdown sequence complete.")
}

// customErrorHandler provides a basic JSON error response for Fiber.
func customErrorHandler(ctx *fiber.Ctx, err error) error {
	code := fiber.StatusInternalServerError
	message := "Internal Server Error"
	var e *fiber.Error
	if errors.As(err, &e) {
		code = e.Code
		message = e.Message
	}
	// Log the error internally before sending response
	// TODO: Inject logger here? For now, use stdlib but could be better.
	stdlog.Printf("[ERROR] Fiber Error: Code=%d, Message=%s, Error=%v, Path=%s", code, message, err, ctx.Path())

	ctx.Set(fiber.HeaderContentType, fiber.MIMEApplicationJSON)
	return ctx.Status(code).JSON(fiber.Map{"error": message})
}
