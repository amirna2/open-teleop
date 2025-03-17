# Open-Teleop Controller

The controller component of the Open-Teleop platform. It serves as the central message router for robot teleoperation.

## Configuration

The controller uses a unified YAML-based configuration system that's shared with other components:

### Configuration File

- `config/open_teleop_config.yaml`: Unified configuration file for all components

### Environment Variables

All configuration values can be overridden using environment variables:

| Environment Variable | Configuration Setting |
| --- | --- |
| `TELEOP_ENVIRONMENT` | Sets the environment (development, testing, production) |
| `TELEOP_ZMQ_CONTROLLER_ADDRESS` | ZeroMQ controller address |
| `TELEOP_ZMQ_GATEWAY_ADDRESS` | ZeroMQ gateway address |
| `TELEOP_ZMQ_GATEWAY_CONNECT_ADDRESS` | Address for gateways to connect to controller |
| `TELEOP_ZMQ_GATEWAY_SUBSCRIBE_ADDRESS` | Address for gateways to subscribe to controller |
| `TELEOP_ZMQ_BUFFER_SIZE` | ZeroMQ message buffer size |
| `TELEOP_ZMQ_RECONNECT_INTERVAL_MS` | ZeroMQ reconnect interval in milliseconds |
| `TELEOP_SERVER_PORT` | HTTP server port |
| `TELEOP_SERVER_REQUEST_TIMEOUT` | HTTP request timeout in seconds |
| `TELEOP_SERVER_MAX_REQUEST_SIZE` | Maximum HTTP request size in MB |
| `TELEOP_HIGH_PRIORITY_WORKERS` | Number of high priority worker goroutines |
| `TELEOP_STANDARD_PRIORITY_WORKERS` | Number of standard priority worker goroutines |
| `TELEOP_LOW_PRIORITY_WORKERS` | Number of low priority worker goroutines |
| `TELEOP_THROTTLE_HIGH_HZ` | Throttle rate for high priority messages (0 = no throttling) |
| `TELEOP_THROTTLE_STANDARD_HZ` | Throttle rate for standard priority messages |
| `TELEOP_THROTTLE_LOW_HZ` | Throttle rate for low priority messages |

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

- **Controller Address**: Binds to this address to receive messages from gateways
- **Gateway Address**: Binds to this address to publish messages to gateways
- **Gateway Connect Address**: Address for gateways to connect to the controller
- **Gateway Subscribe Address**: Address for gateways to subscribe to the controller

In development mode, these bind to localhost, while in testing and production they bind to all interfaces. 