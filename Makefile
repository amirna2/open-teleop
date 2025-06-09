# Open-Teleop Build System
# Makefile front-end for the existing build.sh script

.PHONY: all clean help install
.PHONY: flatbuffers controller gateway webui av_node media_gateway
.PHONY: check-deps
.PHONY: run-controller run-gateway run-av-node run-media-gateway

# Default target
all: flatbuffers webui gateway controller media_gateway

# Check dependencies before building
check-deps:
	@./scripts/build.sh help > /dev/null 2>&1 || { echo "Error: build.sh not found or not executable"; exit 1; }

# Generate FlatBuffers interface code
flatbuffers: check-deps
	@echo "=== Generating FlatBuffers interfaces ==="
	@./scripts/build.sh fbs

# Build Svelte web UI
webui: flatbuffers
	@echo "=== Building Svelte Web UI ==="
	@./scripts/build.sh webui

# Build ROS2 gateway and related packages
gateway: flatbuffers
	@echo "=== Building ROS Gateway ==="
	@./scripts/build.sh gateway

# Build Go controller (includes web UI)
controller: flatbuffers webui
	@echo "=== Building Go Controller ==="
	@./scripts/build.sh controller

# Build A/V node
av_node: flatbuffers
	@echo "=== Building A/V Node ==="
	@./scripts/build.sh av_node

# Build media gateway
media_gateway:
	@echo "=== Building Media Gateway ==="
	@./scripts/build_media_gateway.sh

# Clean all build artifacts
clean:
	@echo "=== Cleaning build artifacts ==="
	@./scripts/build.sh clean

# Install to directory (default: ./install)
install: all
	@echo "=== Installing Open-Teleop ==="
	@./scripts/build.sh install $(INSTALL_DIR)

# Install to custom directory
install-to:
	@if [ -z "$(DIR)" ]; then \
		echo "Usage: make install-to DIR=/path/to/install"; \
		exit 1; \
	fi
	@echo "=== Installing Open-Teleop to $(DIR) ==="
	@./scripts/build.sh install $(DIR)

# Run targets - start individual components
run-controller: controller
	@echo "=== Running Go Controller ==="
	@if [ -n "$(CONFIG)" ]; then \
		echo "Using custom teleop config: $(CONFIG)"; \
		./controller/bin/controller -config-dir ./config -teleop-config $(CONFIG); \
	else \
		./scripts/run_controller.sh; \
	fi

run-gateway: gateway
	@echo "=== Running ROS Gateway ==="
	@./scripts/run_gateway.sh

run-av-node: av_node
	@echo "=== Running A/V Node ==="
	@./scripts/run_av_node.sh

run-media-gateway: media_gateway
	@echo "=== Running Media Gateway ==="
	@./scripts/run_media_gateway.sh

# Development targets for individual components
dev-controller: flatbuffers webui
	@echo "=== Quick controller rebuild ==="
	@cd controller && go build -o bin/controller ./cmd/controller

dev-gateway: flatbuffers
	@echo "=== Quick gateway rebuild ==="
	@cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select ros_gateway

# Show help
help:
	@echo "Open-Teleop Build System"
	@echo ""
	@echo "Build targets:"
	@echo "  all              Build all components (default)"
	@echo "  flatbuffers      Generate FlatBuffers interface code"
	@echo "  webui            Build Svelte web UI"
	@echo "  gateway          Build ROS2 gateway and packages"
	@echo "  controller       Build Go controller"
	@echo "  av_node          Build A/V node"
	@echo "  media_gateway    Build media gateway"
	@echo "  clean            Clean all build artifacts"
	@echo "  install          Install to ./install directory"
	@echo "  install-to DIR=path  Install to custom directory"
	@echo ""
	@echo "Run targets:"
	@echo "  run-controller   Build and run Go controller"
	@echo "  run-gateway      Build and run ROS gateway"
	@echo "  run-av-node      Build and run A/V node"
	@echo "  run-media-gateway Build and run media gateway"
	@echo ""
	@echo "Development targets:"
	@echo "  dev-controller   Quick controller rebuild (no deps)"
	@echo "  dev-gateway      Quick gateway rebuild (no deps)"
	@echo ""
	@echo "Options:"
	@echo "  INSTALL_DIR=path Custom install directory for 'install' target"
	@echo "  CONFIG=path      Custom teleop config for run-controller"
	@echo "  -j N             Build with N parallel jobs"
	@echo ""
	@echo "Examples:"
	@echo "  make all                    # Build everything"
	@echo "  make controller             # Build just controller"
	@echo "  make run-controller         # Build and run controller (default config)"
	@echo "  make run-controller CONFIG=config/test_av_config.yaml  # Custom config"
	@echo "  make run-gateway            # Build and run ROS gateway"
	@echo "  make install-to DIR=/opt/ot # Install to /opt/ot"
	@echo "  make -j4 all               # Build with 4 parallel jobs"