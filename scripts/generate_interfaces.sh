#!/bin/bash
# Script to generate code from FlatBuffers schemas

set -e  # Exit on error

# Path configuration
SCHEMAS_DIR="$(dirname "$(dirname "$0")")/schemas"
ROS2_OUTPUT_DIR="$(dirname "$(dirname "$0")")/ros2_ws/src/bridge_nodes"
GO_OUTPUT_DIR="$(dirname "$(dirname "$0")")/controller/pkg/flatbuffers"
ROS_GATEWAY_DIR="$(dirname "$(dirname "$0")")/ros2_ws/src/ros_gateway"

# Check FlatBuffers compiler
if ! command -v flatc &> /dev/null; then
    echo "Error: FlatBuffers compiler (flatc) not found."
    echo "Please install it with: sudo apt-get install flatbuffers-compiler"
    exit 1
fi

# Ensure output directories exist
mkdir -p "$GO_OUTPUT_DIR"

# First, process the OTT message schema which is common to all bridges
OTT_SCHEMA="$SCHEMAS_DIR/ott_message.fbs"
if [ -f "$OTT_SCHEMA" ]; then
    echo "Processing common schema: ott_message.fbs"
    
    # Generate Go code for controller
    echo "Generating Go code for OTT message..."
    flatc --go -o "$GO_OUTPUT_DIR" "$OTT_SCHEMA"
    
    # Generate Python code for ROS gateway
    if [ -d "$ROS_GATEWAY_DIR" ]; then
        echo "Generating Python code for ros_gateway..."
        GATEWAY_MODULE_DIR="$ROS_GATEWAY_DIR/ros_gateway"
        if [ -d "$GATEWAY_MODULE_DIR" ]; then
            # Create flatbuffers directory
            mkdir -p "$GATEWAY_MODULE_DIR/flatbuffers"
            flatc --python -o "$GATEWAY_MODULE_DIR/flatbuffers" "$OTT_SCHEMA"
            echo "Generated FlatBuffer code for ros_gateway"
        else
            echo "Warning: ros_gateway module directory not found at $GATEWAY_MODULE_DIR"
        fi
    else
        echo "Warning: ros_gateway package directory not found, skipping"
    fi
    
    # Generate Python code for bridge packages only
    echo "Generating Python code for bridge packages..."
    # List all packages in the bridge_nodes directory
    for pkg_dir in "$ROS2_OUTPUT_DIR"/*; do
        if [ -d "$pkg_dir" ]; then
            pkg_name=$(basename "$pkg_dir")
            
            # Only generate for actual bridge packages
            if [[ "$pkg_name" == *bridge* ]]; then
                echo "Found bridge package: $pkg_name"
                
                # Find the Python module directory (same name as the package)
                module_dir="$pkg_dir/$pkg_name"
                if [ -d "$module_dir" ]; then
                    # Create flatbuffers directory
                    mkdir -p "$module_dir/flatbuffers"
                    echo "Generating Python code for $pkg_name..."
                    flatc --python -o "$module_dir/flatbuffers" "$OTT_SCHEMA"
                else
                    echo "Warning: Python module directory not found in $pkg_dir"
                fi
            else
                echo "Skipping non-bridge package: $pkg_name"
            fi
        fi
    done
fi

# Process remaining schema files (if any)
for schema in "$SCHEMAS_DIR"/*.fbs; do
    # Skip the OTT message schema as we've already processed it
    if [ -f "$schema" ] && [ "$(basename "$schema")" != "ott_message.fbs" ]; then
        schema_name=$(basename "$schema")
        echo "Processing schema: $schema_name"
        
        # Extract package name (for ROS2 output)
        package_name=""
        case "$schema_name" in
            teleop.fbs)
                package_name="teleop_command_bridge"
                ;;
            # Add more mappings as needed
            *)
                echo "Warning: No package mapping for $schema_name, skipping ROS2 code generation"
                ;;
        esac
        
        # Generate Go code
        echo "Generating Go code..."
        flatc --go -o "$GO_OUTPUT_DIR" "$schema"
        
        # Generate Python code if package mapping exists
        if [ -n "$package_name" ]; then
            ros2_pkg_dir="$ROS2_OUTPUT_DIR/$package_name/$package_name"
            if [ -d "$ros2_pkg_dir" ]; then
                echo "Generating Python code for $package_name..."
                flatc --python -o "$ros2_pkg_dir" "$schema"
            else
                echo "Warning: ROS2 package directory not found: $ros2_pkg_dir"
            fi
        fi
    fi
done

echo "Code generation complete!" 