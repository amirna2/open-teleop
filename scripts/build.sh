#!/bin/bash

# Open-Teleop Build Script
# This script builds all components of the Open-Teleop platform

set -e  # Exit on error

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

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

# Display help information
show_help() {
    echo "Usage: $0 COMMAND"
    echo ""
    echo "Commands:"
    echo "  all         Build all components (default if no command specified)"
    echo "  gateway     Build only ROS Gateway node"
    echo "  controller  Build only Go controller"
    echo "  fbs         Generate FlatBuffers interface code only"
    echo "  clean       Clean all build artifacts"
    echo "  help        Show this help message"
    echo ""
    echo "Notes:"
    echo "  - FlatBuffers code is automatically regenerated during builds"
    echo "  - Use 'fbs' command to manually generate FlatBuffers code without building"
    echo ""
    echo "Examples:"
    echo "  $0 all        # Build everything"
    echo "  $0 clean      # Clean all build artifacts"
    echo "  $0 fbs        # Generate FlatBuffers code only"
    echo "  $0 gateway    # Build only ROS Gateway node"
}

# Check for required tools
check_dependencies() {
    print_section "Checking Dependencies"
    
    # All builds need the project directories
    if [ ! -d "ros2_ws" ] || [ ! -d "controller" ] || [ ! -d "schemas" ]; then
        print_error "Error: Not in the project root directory."
        print_error "Please run this script from the root of the Open-Teleop project."
        exit 1
    fi
    
    # Check flatc for FlatBuffers generation
    if [[ "$CMD" == "all" || "$CMD" == "fbs" || "$CMD" == "gateway" || "$CMD" == "controller" ]]; then
        if ! command_exists flatc; then
            print_error "Error: FlatBuffers compiler (flatc) not found."
            print_error "Please install it with: sudo apt-get install flatbuffers-compiler"
            exit 1
        fi
    fi
    
    # Check colcon for ROS2 builds
    if [[ "$CMD" == "all" || "$CMD" == "gateway" ]]; then
        if ! command_exists colcon; then
            print_error "Error: colcon build tool not found."
            print_error "Please install it with: pip install -U colcon-common-extensions"
            exit 1
        fi
    fi
    
    # Check Go for controller builds
    if [[ "$CMD" == "all" || "$CMD" == "controller" ]]; then
        if ! command_exists go; then
            print_error "Error: Go compiler not found."
            print_error "Please install Go from https://golang.org/doc/install"
            exit 1
        fi
    fi
}

# Clean build artifacts
clean_build() {
    print_section "Cleaning Build Artifacts"
    
    # Clean ROS2 workspace
    if [ -d "ros2_ws" ]; then
        print_status "Cleaning ROS2 workspace..."
        if [ -d "ros2_ws/build" ]; then
            rm -rf ros2_ws/build
            echo -e "${GREEN}✓ Removed ros2_ws/build directory${NC}"
        fi
        
        if [ -d "ros2_ws/install" ]; then
            rm -rf ros2_ws/install
            echo -e "${GREEN}✓ Removed ros2_ws/install directory${NC}"
        fi
        
        if [ -d "ros2_ws/log" ]; then
            rm -rf ros2_ws/log
            echo -e "${GREEN}✓ Removed ros2_ws/log directory${NC}"
        fi
    fi
    
    # Clean Go controller build artifacts
    if [ -d "controller/bin" ]; then
        print_status "Cleaning Go controller build artifacts..."
        rm -rf controller/bin
        echo -e "${GREEN}✓ Removed controller/bin directory${NC}"
    fi
    
    # Clean generated FlatBuffers code
    print_status "Cleaning generated FlatBuffers code..."
    if [ -d "controller/pkg/flatbuffers" ]; then
        rm -rf controller/pkg/flatbuffers
        echo -e "${GREEN}✓ Removed controller/pkg/flatbuffers directory${NC}"
    fi
    
    # Clean FlatBuffers code in ros_gateway
    if [ -d "ros2_ws/src/ros_gateway" ]; then
        if find "ros2_ws/src/ros_gateway" -path "*/flatbuffers/*" 2>/dev/null | grep -q .; then
            print_status "Cleaning FlatBuffers code in ros_gateway..."
            find "ros2_ws/src/ros_gateway" -path "*/flatbuffers/*" -type d -exec rm -rf {} +
            echo -e "${GREEN}✓ Removed FlatBuffers code in ros_gateway${NC}"
        fi
    fi
    
    # Create required FlatBuffers directories
    mkdir -p controller/pkg/flatbuffers
    if [ -d "ros2_ws/src/ros_gateway" ]; then
        mkdir -p "ros2_ws/src/ros_gateway/ros_gateway/flatbuffers"
    fi
    
    # Clean build-related directories in the project root
    print_status "Cleaning project root directories..."
    
    # Check and remove install directory in project root
    if [ -d "install" ]; then
        rm -rf install
        echo -e "${GREEN}✓ Removed install directory from project root${NC}"
    fi
    
    # Check and remove log directory in project root
    if [ -d "log" ]; then
        rm -rf log
        echo -e "${GREEN}✓ Removed log directory from project root${NC}"
    fi
    
    # Check and remove build directory in project root (if any)
    if [ -d "build" ]; then
        rm -rf build
        echo -e "${GREEN}✓ Removed build directory from project root${NC}"
    fi
    
    # Check for any __pycache__ directories and remove them
    print_status "Cleaning Python cache files..."
    find . -type d -name "__pycache__" -exec rm -rf {} +  2>/dev/null || true
    find . -name "*.pyc" -delete 2>/dev/null || true
    echo -e "${GREEN}✓ Removed Python cache files${NC}"
    
    echo -e "${GREEN}✓ Cleanup complete${NC}"
}

# Generate interface code from FlatBuffers schemas
generate_interfaces() {
    print_section "Generating Interface Code"
    
    if [ ! -d "schemas" ]; then
        print_error "Error: schemas directory not found."
        exit 1
    fi
    
    # Create output directories if they don't exist
    mkdir -p controller/pkg/flatbuffers
    if [ -d "ros2_ws/src/ros_gateway" ]; then
        mkdir -p "ros2_ws/src/ros_gateway/ros_gateway/flatbuffers"
    fi
    
    print_status "Running generate_interfaces.sh script..."
    ./scripts/generate_interfaces.sh
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Interface code generation complete${NC}"
    else
        print_error "Error generating interface code"
        exit 1
    fi
}

# Build ROS2 gateway node
build_ros2() {
    print_section "Building ROS Gateway Node"
    
    if [ ! -d "ros2_ws/src" ]; then
        print_error "Error: ros2_ws/src directory not found."
        exit 1
    fi
    
    if [ ! -d "ros2_ws/src/ros_gateway" ]; then
        print_error "Error: ros2_ws/src/ros_gateway directory not found."
        print_error "Please make sure the ROS Gateway package is in the ros2_ws/src directory."
        exit 1
    fi
    
    # Generate FlatBuffers code first if we're not building all
    if [ "$CMD" != "all" ]; then
        generate_interfaces
    fi
    
    # Source ROS2 setup
    print_status "Sourcing ROS2 setup..."
    source /opt/ros/jazzy/setup.bash
    
    # Build the workspace
    print_status "Building workspace with colcon..."
    cd ros2_ws
    
    # First, build the open_teleop_logger package that the gateway depends on
    print_status "Building open_teleop_logger dependency..."
    colcon build --symlink-install --packages-select open_teleop_logger
    
    if [ $? -eq 0 ]; then
        print_status "Source the updated setup to make open_teleop_logger available..."
        source install/setup.bash
        
        print_status "Building ros_gateway package..."
        colcon build --symlink-install --packages-select ros_gateway
        
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}✓ ROS Gateway node build complete${NC}"
            # Source the setup script
            source install/setup.bash
        else
            print_error "Error building ROS Gateway node"
            exit 1
        fi
    else
        print_error "Error building open_teleop_logger dependency"
        exit 1
    fi
    
    cd ..
}

# Build Go controller
build_go() {
    print_section "Building Go Controller"
    
    if [ ! -d "controller" ]; then
        print_error "Error: controller directory not found."
        exit 1
    fi
    
    # Generate FlatBuffers code first if we're not building all
    if [ "$CMD" != "all" ]; then
        generate_interfaces
    fi
    
    cd controller
    
    print_status "Downloading Go dependencies..."
    go mod download
    
    print_status "Building controller..."
    # Create bin directory if it doesn't exist
    mkdir -p bin
    go build -o bin/controller ./cmd/controller
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Go controller build complete${NC}"
    else
        print_error "Error building Go controller"
        exit 1
    fi
    
    cd ..
}

# Execute the build process
main() {
    print_section "Open-Teleop Build Process Started"
    
    # Check dependencies first
    check_dependencies
    
    case "$CMD" in
        all)
            generate_interfaces
            build_ros2
            build_go
            
            print_section "Build Complete!"
            echo -e "${GREEN}✓ All components built successfully${NC}"
            echo ""
            echo "To run the platform:"
            echo "1. In one terminal:"
            echo "   cd ros2_ws"
            echo "   source install/setup.bash"
            echo "   ros2 launch ros_gateway gateway.launch.py"
            echo ""
            echo "2. In another terminal:"
            echo "   cd controller"
            echo "   ./bin/controller"
            echo ""
            echo "Or use Docker Compose:"
            echo "   cd infra"
            echo "   docker-compose up"
            ;;
        
        gateway)
            print_status "Building ROS Gateway node only"
            build_ros2
            print_section "Gateway Build Complete!"
            ;;
        
        controller)
            print_status "Building Go controller only"
            build_go
            print_section "Controller Build Complete!"
            ;;
        
        fbs)
            print_status "Generating FlatBuffers interface code only"
            generate_interfaces
            print_section "FlatBuffers Generation Complete!"
            ;;
        
        clean)
            print_status "Cleaning all build artifacts"
            clean_build
            print_section "Cleanup Complete!"
            ;;
        
        *)
            show_help
            exit 1
            ;;
    esac
}

# Parse command (default to "all" if not specified)
CMD="${1:-all}"

# If command is "help", show help and exit
if [[ "$CMD" == "help" ]]; then
    show_help
    exit 0
fi

# Run the main function
main 