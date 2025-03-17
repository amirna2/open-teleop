#!/bin/bash
# test_gateway_inbound.sh - Test the ROS Gateway's inbound data flow

# Set up colorful output
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== ROS Gateway Inbound Test ===${NC}"
echo -e "${YELLOW}This test subscribes to the /cmd_vel topic to test the gateway's inbound data flow.${NC}"
echo -e "${YELLOW}Make sure the gateway is running in a separate terminal before running this test.${NC}"
echo ""

# Source ROS2 setup
source ~/projects/open-teleop/ros2_ws/install/setup.bash

# Run the subscriber script to subscribe to cmd_vel messages
echo -e "${GREEN}Subscribing to /cmd_vel...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop.${NC}"
echo ""

# Note: The publisher script names topics with 'pub' suffix, but we need to use the actual ROS topic name for the subscriber
python3 ~/projects/open-teleop/tests/ros2-scripts/sub.py /cmd_vel 