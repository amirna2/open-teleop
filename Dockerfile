# Base image: ROS 2 Jazzy on Ubuntu 24.04 Noble
FROM ros:jazzy-ros-base-noble

# Set non-interactive frontend for apt to avoid prompts
ENV DEBIAN_FRONTEND=noninteractive

# Define Go version and paths (Using 1.18 as minimum mentioned in dev_setup.sh)
# Update this version if a newer one is strictly required
# Or specify a newer version like 1.22.3 if preferred
ENV GO_VERSION=1.24.1
ENV GOPATH=/go
ENV PATH=$GOPATH/bin:/usr/local/go/bin:$PATH

# Install system dependencies required by open-teleop
# Base image includes git, build-essential, cmake, python3-pip
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    libzmq3-dev \
    flatbuffers-compiler \
    python3-pip \
    python3-colcon-common-extensions \
    python3-zmq \
    python3-flatbuffers \
    # ROS packages should be included in base or pulled by colcon/rosdep
    # Clean up apt cache to reduce image size
    && rm -rf /var/lib/apt/lists/*

# Install Go
RUN wget "https://golang.org/dl/go${GO_VERSION}.linux-amd64.tar.gz" -O /tmp/go.tar.gz && \
    tar -C /usr/local -xzf /tmp/go.tar.gz && \
    rm /tmp/go.tar.gz && \
    mkdir -p "$GOPATH/src" "$GOPATH/bin" && \
    chmod -R 777 "$GOPATH" # Note: Using 777 for simplicity, refine if needed

# Python dependencies are now installed via apt above

# Set working directory for subsequent commands
WORKDIR /open-teleop_ws

# Copy the entire project context from the host into the image WORKDIR
# Ensure you have a .dockerignore file to exclude large/unnecessary files (like logs/, build/, install/)
COPY . .

# Make scripts executable
RUN chmod +x ./scripts/*.sh

# Default command: Build all components and run tests
# The scripts handle sourcing ROS environment internally.
CMD ["/bin/bash", "-c", "./scripts/build.sh all && ./scripts/run_tests.sh"] 