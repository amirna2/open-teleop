#!/bin/bash
# test_config_exchange.sh - Test the configuration exchange between controller and gateway

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}===== Testing Open-Teleop Configuration Exchange =====${NC}"

# Get project root
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT"

# Build the controller
echo -e "${YELLOW}Building controller...${NC}"
./scripts/build.sh controller

# Run the go test client
echo -e "${YELLOW}Running Go test client to verify controller's ZeroMQ service...${NC}"
echo -e "${YELLOW}Make sure controller is running in another terminal!${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop the test${NC}"
echo ""

cd "$PROJECT_ROOT/controller"
go test -v ./pkg/zeromq/test/req_client_test.go -run TestRequestClient

echo ""
echo -e "${GREEN}Configuration request test complete!${NC}"
echo ""
echo -e "${YELLOW}Running subscriber test...${NC}"
echo -e "${YELLOW}This will listen for configuration updates indefinitely.${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop the test${NC}"

go test -v ./pkg/zeromq/test/req_client_test.go -run TestSubscriber 