#!/bin/bash

# Ensure the script stops on error
set -e

# The root of the repository
PROJECT_ROOT=$(git rev-parse --show-toplevel)

# Build the media-gateway component first
"$PROJECT_ROOT/scripts/build_media_gateway.sh"

# Navigate to the component directory
cd "$PROJECT_ROOT/media-gateway"

# Activate the python virtual environment
source .venv/bin/activate

# Run the gateway
python3 -m media_gateway.main "$@" 