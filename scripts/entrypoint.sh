#!/bin/bash
set -e # Exit on error

echo "--- Open-Teleop Entrypoint ---"

# Source the ROS 2 environment
echo "Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash || { echo "Failed to source ROS 2 Jazzy setup."; exit 1; }

# Source the local workspace setup
# Check if the setup file exists first
if [ -f "/open-teleop_ws/ros2_ws/install/setup.bash" ]; then
    echo "Sourcing local workspace..."
    source /open-teleop_ws/ros2_ws/install/setup.bash || { echo "Failed to source local workspace setup."; exit 1; }
else
    echo "Error: Local workspace setup file not found at /open-teleop_ws/ros2_ws/install/setup.bash"
    echo "Please ensure the workspace was built successfully."
    exit 1
fi

# Start the Go controller in the background
echo "Starting Go controller in background..."
# Note: Assumes controller reads config from /open-teleop_ws/config
# If using bootstrap config, adjust the flag: -bootstrap-config /path/to/bootstrap.yaml
/open-teleop_ws/controller/bin/controller -config-dir /open-teleop_ws/config &
CONTROLLER_PID=$!
echo "Controller PID: $CONTROLLER_PID"

# Wait a moment for the controller to potentially bind ports (optional, adjust as needed)
sleep 2

# Start the ROS gateway in the foreground
echo "Starting ROS gateway..."
# Use exec to replace the script process with the ros2 launch process
# This ensures signals (like Ctrl+C) are handled correctly by the main process
exec ros2 launch ros_gateway gateway.launch.py 