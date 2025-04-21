#!/bin/bash

./scripts/build.sh gateway
cd ros2_ws
source install/setup.bash
ros2 launch ros_gateway gateway.launch.py

