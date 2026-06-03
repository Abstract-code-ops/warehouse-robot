#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# Load environment variables from .env if present
if [ -f "/ros2_ws/.env" ]; then
    export $(grep -v '^#' /ros2_ws/.env | xargs)
fi

exec "$@"
