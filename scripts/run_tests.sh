#!/bin/bash

# Open-Teleop Test Runner Script
# This script runs tests for all components of the Open-Teleop platform

set -e  # Exit on error

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Define test modes
TEST_ALL=true
TEST_ROS2=false
TEST_GO=false
TEST_INTEGRATION=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --ros2-only)
            TEST_ALL=false
            TEST_ROS2=true
            shift
            ;;
        --go-only)
            TEST_ALL=false
            TEST_GO=true
            shift
            ;;
        --integration-only)
            TEST_ALL=false
            TEST_INTEGRATION=true
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --ros2-only         Only run ROS2 bridge node tests"
            echo "  --go-only           Only run Go controller tests"
            echo "  --integration-only  Only run integration tests"
            echo "  --help              Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

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

# Check for required tools
check_dependencies() {
    print_section "Checking Dependencies"
    
    if [ "$TEST_ALL" = true ] || [ "$TEST_ROS2" = true ]; then
        if ! command_exists colcon; then
            print_error "Error: colcon build tool not found."
            print_error "Please install it with: pip install -U colcon-common-extensions"
            exit 1
        fi
    fi
    
    if [ "$TEST_ALL" = true ] || [ "$TEST_GO" = true ]; then
        if ! command_exists go; then
            print_error "Error: Go compiler not found."
            print_error "Please install Go from https://golang.org/doc/install"
            exit 1
        fi
    fi
    
    # Check if we're in the project root directory
    if [ ! -d "ros2_ws" ] || [ ! -d "controller" ]; then
        print_error "Error: Not in the project root directory."
        print_error "Please run this script from the root of the Open-Teleop project."
        exit 1
    fi
}

# Run ROS2 bridge node tests
run_ros2_tests() {
    print_section "Running ROS2 Bridge Node Tests"
    
    if [ ! -d "ros2_ws/src" ]; then
        print_error "Error: ros2_ws/src directory not found."
        exit 1
    fi
    
    # Source ROS2 setup
    print_status "Sourcing ROS2 setup..."
    source /opt/ros/jazzy/setup.bash
    
    # Run the tests
    print_status "Running ROS2 tests with colcon..."
    cd ros2_ws
    
    # Make sure the workspace is built first
    if [ ! -d "build" ] || [ ! -d "install" ]; then
        print_status "Workspace not built. Building first..."
        colcon build --symlink-install
    fi
    
    # Run the tests
    colcon test
    
    # Show test results
    print_status "Test results:"
    colcon test-result --verbose
    
    # Check if any tests failed
    if colcon test-result | grep -q "Failed:"; then
        print_error "Some ROS2 tests failed."
        TEST_SUCCESS=false
    else
        echo -e "${GREEN}✓ All ROS2 tests passed${NC}"
    fi
    
    cd ..
}

# Run Go controller tests
run_go_tests() {
    print_section "Running Go Controller Tests"
    
    if [ ! -d "controller" ]; then
        print_error "Error: controller directory not found."
        exit 1
    fi
    
    cd controller
    
    print_status "Running Go tests..."
    go test -v ./...
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ All Go tests passed${NC}"
    else
        print_error "Some Go tests failed."
        TEST_SUCCESS=false
    fi
    
    cd ..
}

# Run integration tests
run_integration_tests() {
    print_section "Running Integration Tests"
    
    if [ ! -d "tests" ]; then
        print_error "Error: tests directory not found."
        exit 1
    fi
    
    # Create integration test directory if it doesn't exist
    mkdir -p tests/integration
    
    # Check if there are any integration tests
    if [ ! -f "tests/integration/run_integration_tests.sh" ]; then
        print_status "Integration test script not found. Creating a placeholder..."
        
        # Create a placeholder integration test script
        cat > tests/integration/run_integration_tests.sh << 'EOF'
#!/bin/bash

# Open-Teleop Integration Test Runner
# This script runs integration tests for the Open-Teleop platform

echo "=== Running Integration Tests ==="
echo "No integration tests defined yet. Add your tests here."

# Return success for now
exit 0
EOF
        chmod +x tests/integration/run_integration_tests.sh
    fi
    
    # Run the integration tests
    print_status "Running integration tests..."
    tests/integration/run_integration_tests.sh
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ All integration tests passed${NC}"
    else
        print_error "Some integration tests failed."
        TEST_SUCCESS=false
    fi
}

# Execute the test process
main() {
    print_section "Open-Teleop Test Process Started"
    
    # Track overall test success
    TEST_SUCCESS=true
    
    # Check dependencies first
    check_dependencies
    
    # Run ROS2 tests if needed
    if [ "$TEST_ALL" = true ] || [ "$TEST_ROS2" = true ]; then
        run_ros2_tests
    fi
    
    # Run Go tests if needed
    if [ "$TEST_ALL" = true ] || [ "$TEST_GO" = true ]; then
        run_go_tests
    fi
    
    # Run integration tests if needed
    if [ "$TEST_ALL" = true ] || [ "$TEST_INTEGRATION" = true ]; then
        run_integration_tests
    fi
    
    print_section "Test Summary"
    if [ "$TEST_SUCCESS" = true ]; then
        echo -e "${GREEN}✓ All tests passed${NC}"
    else
        echo -e "${RED}✗ Some tests failed${NC}"
        exit 1
    fi
}

# Run the main function
main 