#!/bin/bash
# test_gateway_outbound.sh - Test the ROS Gateway's outbound data flow

# Set up colorful output
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== ROS Gateway Outbound Test ===${NC}"
echo -e "${YELLOW}This test publishes battery state messages to test the gateway's outbound data flow.${NC}"
echo -e "${YELLOW}Make sure the gateway is running in a separate terminal before running this test.${NC}"
echo ""

# Source ROS2 setup
source ~/projects/open-teleop/ros2_ws/install/setup.bash

# Run the publisher script to publish battery state messages at 1 Hz
echo -e "${GREEN}Publishing battery state messages to /battery_state...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop.${NC}"
echo ""

python3 ~/projects/open-teleop/tests/ros2-scripts/pub.py --topics batterystate --rate 1 