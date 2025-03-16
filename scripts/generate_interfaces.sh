#!/bin/bash
# Script to generate code from FlatBuffers schemas

set -e  # Exit on error

# Path configuration
SCHEMAS_DIR="$(dirname "$(dirname "$0")")/schemas"
ROS2_OUTPUT_DIR="$(dirname "$(dirname "$0")")/ros2_ws/src/bridge_nodes"
GO_OUTPUT_DIR="$(dirname "$(dirname "$0")")/controller/pkg/flatbuffers"

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
    
    # Generate Python code for all bridge packages
    echo "Generating Python code for all bridge packages..."
    for pkg_dir in "$ROS2_OUTPUT_DIR"/*bridge*/*; do
        if [ -d "$pkg_dir" ]; then
            # Create flatbuffers directory if it doesn't exist
            mkdir -p "$pkg_dir/flatbuffers"
            echo "Generating Python code for $(basename "$(dirname "$pkg_dir")")..."
            flatc --python -o "$pkg_dir/flatbuffers" "$OTT_SCHEMA"
        fi
    done
fi

# Process remaining schema files with specific mappings
for schema in "$SCHEMAS_DIR"/*.fbs; do
    # Skip the OTT message schema as we've already processed it
    if [ -f "$schema" ] && [ "$(basename "$schema")" != "ott_message.fbs" ]; then
        schema_name=$(basename "$schema")
        echo "Processing schema: $schema_name"
        
        # Extract package name (for ROS2 output)
        # If diagnostics.fbs maps to system_diagnostic, teleop.fbs to teleop_command_bridge, etc.
        package_name=""
        case "$schema_name" in
            diagnostics.fbs)
                package_name="system_diagnostic"
                ;;
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