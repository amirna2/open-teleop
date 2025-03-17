#!/bin/bash

# Open-Teleop Controller Run Script
# This script runs the controller with the specified environment configuration

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Default environment
ENV="development"

# Parse command line arguments
while getopts "e:h" opt; do
  case $opt in
    e) ENV="$OPTARG" ;;
    h) 
      echo "Usage: $0 [-e environment] [-h]"
      echo ""
      echo "Options:"
      echo "  -e environment   Specify environment (development, testing, production)"
      echo "                   Default: development"
      echo "  -h               Show this help message"
      exit 0
      ;;
    \?) 
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

# Validate environment
case $ENV in
  development|testing|production)
    echo -e "${GREEN}Starting controller in ${ENV} environment${NC}"
    ;;
  *)
    echo -e "${RED}Invalid environment: ${ENV}${NC}"
    echo "Valid environments: development, testing, production"
    exit 1
    ;;
esac

# Get the absolute path to the project root
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_DIR="${PROJECT_ROOT}/config"
CONTROLLER_BIN="${PROJECT_ROOT}/controller/bin/controller"

# Check if controller binary exists
if [ ! -f "$CONTROLLER_BIN" ]; then
  echo -e "${YELLOW}Controller binary not found. Building now...${NC}"
  cd "$PROJECT_ROOT" && ./scripts/build.sh controller
  
  # Check if build succeeded
  if [ ! -f "$CONTROLLER_BIN" ]; then
    echo -e "${RED}Failed to build controller${NC}"
    exit 1
  fi
fi

# Run the controller with the specified environment
echo -e "${GREEN}Running controller with config from: ${CONFIG_DIR}${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
echo ""

# Set environment variables that can be used by the controller
export TELEOP_ENVIRONMENT="$ENV"

# Run the controller with the specified environment
"$CONTROLLER_BIN" -env "$ENV" -config-dir "$CONFIG_DIR" 