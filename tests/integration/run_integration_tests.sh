#!/bin/bash

# Open-Teleop Integration Test Runner
# This script runs integration tests for the Open-Teleop platform

set -e  # Exit on error

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Detect the absolute path to the project root
PROJECT_ROOT="$(cd "$(dirname "$(dirname "$(dirname "${BASH_SOURCE[0]}")")")" && pwd)"

# Function to print section headers
print_section() {
    echo -e "\n${GREEN}=== $1 ===${NC}"
}

# Function to print status messages
print_status() {
    echo -e "${YELLOW}$1${NC}"
}

# Function to print error messages
print_error() {
    echo -e "${RED}$1${NC}"
}

# Source development environment
source "$PROJECT_ROOT/scripts/dev_env.sh"

print_section "Open-Teleop Integration Tests"

# Check if Docker and Docker Compose are installed for containerized testing
if command -v docker &> /dev/null && command -v docker-compose &> /dev/null; then
    print_status "Docker and Docker Compose found. Can use containerized testing."
else
    print_status "Docker or Docker Compose not found. Will use local testing."
fi

# Function to run a basic connectivity test
test_connectivity() {
    print_section "Testing Bridge-Controller Connectivity"
    
    # Start the controller in the background
    print_status "Starting Go controller in background..."
    cd "$PROJECT_ROOT/controller"
    ./bin/controller &
    CONTROLLER_PID=$!
    
    # Give it a moment to start up
    sleep 2
    
    # Test if controller is running by checking the API
    print_status "Testing controller API..."
    if curl -s http://localhost:8080/api/health | grep -q "ok"; then
        echo -e "${GREEN}✓ Controller API is responding${NC}"
    else
        print_error "Controller API is not responding"
        kill $CONTROLLER_PID
        return 1
    fi
    
    # Start a basic diagnostic bridge
    print_status "Starting basic diagnostic bridge..."
    cd "$PROJECT_ROOT/ros2_ws"
    # This is a placeholder - you'll need to implement the actual launch
    ros2 launch launch/diagnostic_bridge.launch.py &
    BRIDGE_PID=$!
    
    # Give it a moment to start up
    sleep 5
    
    # Test if the controller is receiving data from the bridge
    print_status "Testing data flow from bridge to controller..."
    if curl -s http://localhost:8080/api/diagnostics | grep -q "data"; then
        echo -e "${GREEN}✓ Controller is receiving data from the bridge${NC}"
    else
        print_error "Controller is not receiving data from the bridge"
        kill $CONTROLLER_PID
        kill $BRIDGE_PID
        return 1
    fi
    
    # Clean up
    kill $CONTROLLER_PID
    kill $BRIDGE_PID
    
    echo -e "${GREEN}✓ Connectivity test passed${NC}"
    return 0
}

# Test functions should be defined above this line
# Uncomment the test functions you want to run

print_section "Running Tests"

# For now, just display a message
print_status "No integration tests are implemented yet."
print_status "This is a template for integration tests."
print_status "Add actual test code to run tests."

# Uncomment to run tests when they're implemented
# test_connectivity

print_section "Integration Tests Summary"
echo -e "${GREEN}✓ All tests completed${NC}"

exit 0 