# Base image: ROS 2 Jazzy on Ubuntu 24.04 Noble
FROM ros:jazzy-ros-base-noble

# Set non-interactive frontend for apt to avoid prompts
ENV DEBIAN_FRONTEND=noninteractive

# --- Add user/group setup ---
ARG UID=1978
ARG GID=1978
RUN groupadd --gid $GID teleop && \
    useradd --uid $UID --gid $GID --create-home --shell /bin/bash teleop
# --- End user/group setup ---

# Define Go version and paths
ENV GO_VERSION=1.24.1
ENV GOPATH=/go
ENV PATH=$GOPATH/bin:/usr/local/go/bin:$PATH

# Install system dependencies required by open-teleop
# Base image includes git, build-essential, cmake, python3-pip
# Adding python3-colcon-common-extensions, python3-zmq, python3-flatbuffers here
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    libzmq3-dev \
    flatbuffers-compiler \
    python3-pip \
    python3-colcon-common-extensions \
    python3-zmq \
    python3-flatbuffers \
    ros-jazzy-rmw-cyclonedds-cpp \
    # ROS packages should be included in base or pulled by colcon/rosdep
    # Clean up apt cache to reduce image size
    && rm -rf /var/lib/apt/lists/*

# Install Go
RUN wget "https://golang.org/dl/go${GO_VERSION}.linux-amd64.tar.gz" -O /tmp/go.tar.gz && \
    tar -C /usr/local -xzf /tmp/go.tar.gz && \
    rm /tmp/go.tar.gz && \
    # Create GOPATH and set ownership
    mkdir -p "$GOPATH/src" "$GOPATH/bin" && \
    chown -R teleop:teleop "$GOPATH"
    # --- End ownership change ---

# Set working directory for subsequent commands
WORKDIR /home/teleop/open_teleop

# Copy the entire project context
COPY . .

# Copy the CycloneDDS configuration file
COPY ros2_ws/src/ros_gateway/config/cyclonedds.xml /home/teleop/open_teleop/ros2_ws/src/ros_gateway/config/cyclonedds.xml

# Make scripts executable (including entrypoint)
# Run this before USER switch to avoid needing sudo
RUN chmod +x ./scripts/*.sh \
    && chmod +x ./scripts/entrypoint.sh

# --- Set ownership of copied files ---
RUN chown -R teleop:teleop /home/teleop/open_teleop

# --- Build the project BEFORE switching user ---
# Run the build script as root, as it might need permissions for colcon etc.
# Alternatively, grant sudo access to teleop user, but running as root is simpler here.
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && ./scripts/build.sh all"

# --- Switch to non-root user ---
USER teleop:teleop

# --- Set Entrypoint ---
ENTRYPOINT ["/home/teleop/open_teleop/scripts/entrypoint.sh"]

# CMD can be removed or left empty as ENTRYPOINT is used
# CMD []