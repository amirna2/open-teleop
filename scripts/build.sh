#!/bin/bash

# Open-Teleop Build Script
# This script builds all components of the Open-Teleop platform

set -e  # Exit on error

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Default version if not specified
DEFAULT_VERSION="0.1.0"

# Default installation directory (relative path for local development)
DEFAULT_INSTALL_DIR="./install"

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
    echo "Usage: $0 COMMAND [OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  all             Build all components (default if no command specified)"
    echo "  gateway         Build only ROS Gateway node"
    echo "  controller      Build only Go controller"
    echo "  fbs             Generate FlatBuffers interface code only"
    echo "  install [DIR]   Install built artifacts to specified directory (default: ./install)"
    echo "  clean           Clean all build artifacts"
    echo "  help            Show this help message"
    echo ""
    echo "Options:"
    echo "  --version VER   Specify version for packaging/installing"
    echo ""
    echo "Examples:"
    echo "  $0 all                     # Build everything"
    echo "  $0 install ~/open-teleop   # Install to custom directory"
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
    
    # Clean generated open_teleop_msgs code (if exists)
    if [ -d "ros2_ws/src/open_teleop_msgs" ]; then
         if find "ros2_ws/src/open_teleop_msgs" -name "*.py" -o -name "*.h" -o -name "*.hpp" -o -name "*.c" -o -name "*.cpp" | grep -q .; then
            print_status "Cleaning generated code in open_teleop_msgs..."
            # Be careful here, might need refinement based on actual generated structure
            find ros2_ws/src/open_teleop_msgs \( -path "*/rosidl_adapter" -o -path "*/rosidl_generator_c" -o -path "*/rosidl_generator_cpp" -o -path "*/rosidl_generator_py" -o -path "*/rosidl_typesupport_c" -o -path "*/rosidl_typesupport_cpp" -o -path "*/rosidl_typesupport_fastrtps_c" -o -path "*/rosidl_typesupport_fastrtps_cpp" -o -path "*/rosidl_typesupport_interface" -o -path "*/rosidl_typesupport_introspection_c" -o -path "*/rosidl_typesupport_introspection_cpp" \) -prune -o -name "*.py" -print -delete
            echo -e "${GREEN}✓ Cleaned generated code in open_teleop_msgs${NC}"
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
    
    # Clean Media Gateway build artifacts
    if [ -d "media-gateway" ]; then
        print_status "Cleaning Media Gateway..."
        ./scripts/build_media_gateway.sh clean
    fi
    
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

        print_status "Building open_teleop_msgs dependency..."
        colcon build --symlink-install --packages-select open_teleop_msgs

        if [ $? -eq 0 ]; then
            print_status "Source the updated setup to make open_teleop_msgs available..."
            source install/setup.bash
        
            print_status "Building ros_gateway package..."
            colcon build --symlink-install --packages-select ros_gateway
            
            if [ $? -eq 0 ]; then
                print_status "Building open_teleop_av_node package..."
                colcon build --symlink-install --packages-select open_teleop_av_node

                if [ $? -eq 0 ]; then
                     echo -e "${GREEN}✓ ROS packages build complete${NC}"
                     # Source the setup script
                     source install/setup.bash
                 else
                     print_error "Error building open_teleop_av_node package"
                     exit 1
                fi
            else
                print_error "Error building ROS Gateway node"
                exit 1
            fi
        else
            print_error "Error building open_teleop_msgs package"
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
    
    # Build ROS parser C++ library
    print_status "Building ROS parser C++ library..."
    if [ -d "controller/pkg/rosparser/cpp" ]; then
        mkdir -p lib
        cd controller/pkg/rosparser/cpp
        mkdir -p build
        cd build
        cmake .. && make
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}✓ ROS parser C++ library built successfully${NC}"
            # Copy the library to the project lib directory
            cp lib/libros_parser.so ../../../../../lib/
        else
            print_error "Error building ROS parser C++ library"
            cd ../../../../..
            exit 1
        fi
        cd ../../../../..  # Return to project root
    else
        print_error "Error: ROS parser C++ directory not found at controller/pkg/rosparser/cpp"
        exit 1
    fi
    
    # Build Go controller
    cd controller
    
    print_status "Downloading Go dependencies..."
    go mod download
    
    print_status "Building controller..."
    # Create bin directory if it doesn't exist
    mkdir -p bin
    # Set LD_LIBRARY_PATH to include the built ROS parser library
    export LD_LIBRARY_PATH="$(pwd)/../lib:$LD_LIBRARY_PATH"
    go build -o bin/controller ./cmd/controller
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Go controller build complete${NC}"
    else
        print_error "Error building Go controller"
        exit 1
    fi
    
    cd ..  # Return to project root
}

# Install built components to target directory
install_runtime() {
    print_section "Installing Open-Teleop"
    
    # Ensure everything is built first if needed
    if [ ! -d "controller/bin" ] || [ ! -d "ros2_ws/install" ]; then
        print_status "Building all components before installing..."
        generate_interfaces
        build_ros2
        build_go
    fi
    
    # Default to current directory's 'install' folder if no path provided
    INSTALL_DIR="${1:-./install}"
    
    print_status "Installing to $INSTALL_DIR..."
    
    # Clean previous installation if it exists
    if [ -d "$INSTALL_DIR" ]; then
        rm -rf "$INSTALL_DIR"
    fi
    
    # Create installation directory structure
    mkdir -p $INSTALL_DIR/bin
    mkdir -p $INSTALL_DIR/config
    mkdir -p $INSTALL_DIR/ros2_overlay
    mkdir -p $INSTALL_DIR/scripts
    mkdir -p $INSTALL_DIR/lib
    
    # Copy Go controller
    print_status "Installing Go controller..."
    cp controller/bin/controller $INSTALL_DIR/bin/
    
    # Copy ROS parser library
    print_status "Installing ROS parser library..."
    if [ -f "lib/libros_parser.so" ]; then
        cp lib/libros_parser.so $INSTALL_DIR/lib/
    else
        print_error "Warning: ROS parser library not found at lib/libros_parser.so"
        print_error "The controller may not work correctly without it."
    fi
    
    # Copy configuration files
    print_status "Installing configuration files..."
    cp -r config/* $INSTALL_DIR/config/
    
    # Handle ROS2 workspace - need to regenerate flatbuffers code first
    print_status "Regenerating interface code..."
    generate_interfaces
    
    print_status "Installing ROS2 overlay with resolved symlinks..."
    
    # Clean and rebuild without symlinks
    cd ros2_ws
    source /opt/ros/jazzy/setup.bash
    rm -rf build install log
    # Build dependencies first, then the main nodes
    colcon build --packages-select open_teleop_logger open_teleop_msgs
    source install/setup.bash
    colcon build --packages-select ros_gateway open_teleop_av_node
    cd ..
    
    # Copy with resolved symlinks
    cp -rL ros2_ws/install/* $INSTALL_DIR/ros2_overlay/
    
    # Make sure flatbuffers directory exists in the site-packages for ros_gateway
    print_status "Ensuring flatbuffers files are properly installed..."
    SITE_PACKAGES_DIR="$INSTALL_DIR/ros2_overlay/ros_gateway/lib/python3.12/site-packages/ros_gateway"
    mkdir -p "$SITE_PACKAGES_DIR/flatbuffers/open_teleop"
    
    # Copy the generated flatbuffers files
    cp -r ros2_ws/src/ros_gateway/ros_gateway/flatbuffers/* "$SITE_PACKAGES_DIR/flatbuffers/"

    # Also install open_teleop_av_node site-packages
    AV_NODE_SITE_PACKAGES_DIR="$INSTALL_DIR/ros2_overlay/open_teleop_av_node/lib/python3.12/site-packages/open_teleop_av_node"
    if [ -d "$AV_NODE_SITE_PACKAGES_DIR" ]; then
        print_status "Ensuring open_teleop_av_node files are properly installed..."
        # No extra files to copy for now, just ensuring structure exists if built
    fi
    
    # Copy CycloneDDS config to the right location
    print_status "Installing CycloneDDS config..."
    mkdir -p $INSTALL_DIR/ros2_overlay/share/ros_gateway/config
    cp ros2_ws/src/ros_gateway/config/cyclonedds.xml $INSTALL_DIR/ros2_overlay/share/ros_gateway/config/
    
    # Create workspace setup script
    print_status "Creating workspace setup script..."
    cat > $INSTALL_DIR/scripts/workspace.bash << 'EOF'
#!/bin/bash
# Open-Teleop workspace setup script

# Find the workspace root based on script location
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

# Source ROS2 environment first
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "WARNING: ROS2 Jazzy setup.bash not found!"
fi

# Source the ROS2 overlay
if [ -f "$WORKSPACE_ROOT/ros2_overlay/setup.bash" ]; then
    source "$WORKSPACE_ROOT/ros2_overlay/setup.bash"
else
    echo "ERROR: ROS2 overlay setup.bash not found!"
    return 1
fi

# Add controller binary directory to PATH (only if not already there)
BIN_DIR="$WORKSPACE_ROOT/bin"
if [[ ":$PATH:" != *":$BIN_DIR:"* ]]; then
    export PATH="$BIN_DIR:$PATH"
fi

# Add library directory to LD_LIBRARY_PATH (only if not already there)
LIB_DIR="$WORKSPACE_ROOT/lib"
if [[ ":$LD_LIBRARY_PATH:" != *":$LIB_DIR:"* ]]; then
    export LD_LIBRARY_PATH="$LIB_DIR:$LD_LIBRARY_PATH"
fi

# Set RMW Implementation to CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Configure logging with microsecond precision
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time:yyyy-MM-dd HH:mm:ss.uuuuuu}] [{name}]: {message}"

# Configure CycloneDDS
CONFIG_FILE="$WORKSPACE_ROOT/ros2_overlay/share/ros_gateway/config/cyclonedds.xml"
if [ -f "$CONFIG_FILE" ]; then
    export CYCLONEDDS_URI="$CONFIG_FILE"
    echo "CycloneDDS configured with: $CONFIG_FILE"
else
    echo "WARNING: CycloneDDS config file not found at $CONFIG_FILE!"
fi

# Set configuration directory for controller
export OPEN_TELEOP_CONFIG_DIR="$WORKSPACE_ROOT/config"

echo "Open-Teleop workspace environment configured. You can now run:"
echo "  - controller (Go controller)"
echo "  - ros2 launch ros_gateway gateway.launch.py (ROS Gateway)"
EOF
    chmod +x $INSTALL_DIR/scripts/workspace.bash
    
    # Create a simple start script
    print_status "Creating start script..."
    cat > $INSTALL_DIR/scripts/start.sh << 'EOF'
#!/bin/bash
# Open-Teleop startup script

# Find the workspace root based on script location
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

# Source the workspace environment
source "$SCRIPT_DIR/workspace.bash"

# Start the Go controller in the background
"$WORKSPACE_ROOT/bin/controller" -config-dir "$WORKSPACE_ROOT/config" &
CONTROLLER_PID=$!
echo "Controller started with PID: $CONTROLLER_PID"

# Wait a moment for controller to initialize
sleep 2

# Start the ROS Gateway
ros2 launch ros_gateway gateway.launch.py

# When ROS Gateway exits, also kill the controller
kill $CONTROLLER_PID 2>/dev/null || true
EOF
    chmod +x $INSTALL_DIR/scripts/start.sh
    
    echo -e "${GREEN}✓ Installation complete in $INSTALL_DIR${NC}"
    echo "To use the installed software:"
    echo "  source $INSTALL_DIR/scripts/workspace.bash"
}

# Main script execution
main() {
    # Set defaults
    CMD=${1:-all}
    
    # Parse additional arguments
    shift || true
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --version)
                VERSION="$2"
                shift 2
                ;;
            *)
                INSTALL_DIR="$1"
                shift
                ;;
        esac
    done
    
    case $CMD in
        all)
            check_dependencies
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
            echo "3. (Optional) In a third terminal:"
            echo "   cd ros2_ws"
            echo "   source install/setup.bash"
            echo "   ros2 launch open_teleop_av_node av_node.launch.py"
            ;;
        gateway)
            check_dependencies
            build_ros2
            ;;
        controller)
            check_dependencies
            build_go
            ;;
        fbs)
            check_dependencies
            generate_interfaces
            ;;
        install)
            check_dependencies
            install_runtime "$INSTALL_DIR"
            ;;
        clean)
            clean_build
            # Also clean install directory if it exists
            if [ -d "install" ]; then
                print_status "Cleaning install directory..."
                rm -rf install
                echo -e "${GREEN}✓ Removed install directory${NC}"
            fi
            ;;
        help)
            show_help
            ;;
        av_node)
            check_dependencies
            # Assuming generate_interfaces is not needed directly for av_node
            # Assuming open_teleop_logger and open_teleop_msgs are built first
            # This might need adjustment when msgs package exists
            print_status "Building open_teleop_av_node only..."
            cd ros2_ws
            source /opt/ros/jazzy/setup.bash
            # Ensure dependencies are built/available
            print_status "Ensuring dependencies (logger, msgs) are built..."
            colcon build --symlink-install --packages-up-to open_teleop_msgs
            print_status "Sourcing potentially updated install space..."
            source install/setup.bash # Source existing install space for dependencies
            print_status "Building open_teleop_av_node..."
            colcon build --symlink-install --packages-select open_teleop_av_node
            if [ $? -eq 0 ]; then
                echo -e "${GREEN}✓ open_teleop_av_node build complete${NC}"
            else
                print_error "Error building open_teleop_av_node"
                exit 1
            fi
            cd ..
            ;;
        *)
            print_error "Error: Unknown command '$CMD'"
            show_help
            exit 1
            ;;
    esac
}

# Entry point
main "$@" 