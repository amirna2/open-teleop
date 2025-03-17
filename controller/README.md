# Open-Teleop Controller

The controller component of the Open-Teleop platform. It serves as the central message router for robot teleoperation.

## Configuration

The controller uses a YAML-based configuration system with environment-specific settings:

### Configuration Files

- `config/controller_config.yaml`: Base configuration file
- `config/controller_config_development.yaml`: Development environment settings
- `config/controller_config_testing.yaml`: Testing environment settings
- `config/controller_config_production.yaml`: Production environment settings

### Environment Variables

All configuration values can be overridden using environment variables:

| Environment Variable | Configuration Setting |
| --- | --- |
| `TELEOP_ENVIRONMENT` | Sets the environment (development, testing, production) |
| `TELEOP_ZMQ_RECEIVER_ADDRESS` | ZeroMQ receiver address |
| `TELEOP_ZMQ_PUBLISHER_ADDRESS` | ZeroMQ publisher address |
| `TELEOP_ZMQ_QUEUE_SIZE` | ZeroMQ message queue size |
| `TELEOP_ZMQ_TIMEOUT_MS` | ZeroMQ operation timeout in milliseconds |
| `TELEOP_SERVER_PORT` | HTTP server port |
| `TELEOP_SERVER_REQUEST_TIMEOUT` | HTTP request timeout in seconds |
| `TELEOP_SERVER_MAX_REQUEST_SIZE` | Maximum HTTP request size in MB |
| `TELEOP_HIGH_PRIORITY_WORKERS` | Number of high priority worker goroutines |
| `TELEOP_STANDARD_PRIORITY_WORKERS` | Number of standard priority worker goroutines |
| `TELEOP_LOW_PRIORITY_WORKERS` | Number of low priority worker goroutines |

## Running the Controller

### Using the Run Script

The simplest way to run the controller is to use the provided script:

```bash
# Run with development environment (default)
./scripts/run_controller.sh

# Run with testing environment
./scripts/run_controller.sh -e testing

# Run with production environment
./scripts/run_controller.sh -e production
```

### Manual Execution

You can also run the controller manually:

```bash
# Build the controller
./scripts/build.sh controller

# Run with specific environment and config directory
./controller/bin/controller -env development -config-dir ./config
```

## ZeroMQ Communication

The controller uses ZeroMQ for message-based communication with gateways:

- **Receiver Address**: Binds to this address to receive messages from gateways
- **Publisher Address**: Binds to this address to publish messages to gateways

In development mode, these bind to localhost, while in testing and production they bind to all interfaces. 