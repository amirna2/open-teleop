# Open-Teleop Controller

The controller component of the Open-Teleop platform. It serves as the central message router for robot teleoperation.

## Configuration

The controller uses two primary YAML configuration files located in the directory specified by the `--config-dir` flag (defaults to `./config` relative to the project root or executable location).

### Bootstrap Configuration

- **`controller_config.yaml`**: Defines essential settings needed for the controller to start up.
  - **`logging.level`**: Sets the logging verbosity (e.g., "DEBUG", "INFO", "WARN", "ERROR").
  - **`logging.log_path`**: Specifies the directory for log files. If empty or omitted, file logging is disabled.
  - **`server.http_port`**: The port for the internal HTTP server.
  - **`zeromq.request_bind_address`**: Address the controller binds to for receiving gateway requests.
  - **`zeromq.publish_bind_address`**: Address the controller binds to for publishing messages to gateways.
  - **`processing.*_workers`**: Number of worker goroutines for different priority levels.
  - **`data.directory`**: Directory where operational configuration files are stored/managed.
  - **`data.teleop_config_file`**: The filename of the main operational configuration file within the data directory.

### Operational Configuration

- **`<value of data.teleop_config_file>`** (e.g., `open_teleop_config.yaml`): Defines the teleoperation session specifics, such as topic mappings, robot ID, etc.
  - **`Environment`**: Specifies the operational environment ("development", "testing", "production"). This can be overridden by the `--env` command-line flag.
  - This configuration file has its own environment variable override mechanism. Refer to `pkg/config/config.go` and `ApplyEnvironmentOverrides` for details.

### Command-Line Flags

- **`--config-dir <path>`**: Specifies the directory containing `controller_config.yaml` and the operational config file. Defaults to `./config`.

## Running the Controller

### Using the Run Script

```bash
# Run with development environment (default behavior depends on script)
./scripts/run_controller.sh

# Run with testing environment
./scripts/run_controller.sh -e testing

# Run with production environment
./scripts/run_controller.sh -e production
```

### Manual Execution

```bash
# Build the controller
./scripts/build.sh controller

# Run with specific config directory
# (Ensure controller_config.yaml and open_teleop_config.yaml are in ./config)
./controller/bin/controller --config-dir ./config
```

## ZeroMQ Communication

The controller uses ZeroMQ for message-based communication with gateways. Addresses for binding are specified in `controller_config.yaml`, while addresses for connecting to gateways are typically defined in the operational configuration (`open_teleop_config.yaml`).

- **Controller Address**: Binds to this address to receive messages from gateways
- **Gateway Address**: Binds to this address to publish messages to gateways
- **Gateway Connect Address**: Address for gateways to connect to the controller
- **Gateway Subscribe Address**: Address for gateways to subscribe to the controller

In development mode, these bind to localhost, while in testing and production they bind to all interfaces. 