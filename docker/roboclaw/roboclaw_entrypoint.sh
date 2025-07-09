#!/bin/bash
# entrypoint.sh for the roboclaw_container

# Exit immediately if a command exits with a non-zero status.
set -e

# Create ros2_ws and src directories
mkdir -p /home/roboclaw/ros2_ws
mkdir -p /home/roboclaw/ros2_ws/src

# Execute the command passed to the docker run command
exec "/bin/bash" 