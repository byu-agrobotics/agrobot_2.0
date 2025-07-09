# Created by Brighton Anderson

#!/bin/bash

# This script intelligently starts the Roboclaw container and ROS node.
# It keeps the container running in the background for a persistent environment.

# --- Configuration ---
# The name of the service defined in your docker-compose.yml
SERVICE_NAME="roboclaw"
CONTAINER_NAME="roboclaw-ct"
# The project name is typically the name of the current directory
PROJECT_NAME=$(basename "$PWD")
# Docker Compose creates an image with the name <project>-<service>
IMAGE_NAME="${PROJECT_NAME}-${SERVICE_NAME}"

# --- Graceful shutdown function ---
# This function is called when you press Ctrl+C
cleanup() {
    echo -e "\nROS node stopped. The container '${CONTAINER_NAME}' remains running in the background."
    exit 0
}

# Trap Ctrl+C (the SIGINT signal) and call the cleanup function.
# This ensures that when you exit the script, the container is NOT stopped.
trap cleanup SIGINT

# --- Main Logic ---
# Check if the container is already running.
if [ $(docker ps -q -f name=^/${CONTAINER_NAME}$) ]; then
    echo "Container '${CONTAINER_NAME}' is already running."
else
    echo "Container not running. Starting it now..."
    # If the container isn't running, check if the image exists and build if needed.
    if [[ -z "$(docker images -q ${IMAGE_NAME} 2> /dev/null)" ]]; then
      echo "Image not found. Building and starting container..."
      # --build creates the image. This image IS persistent.
      # You will only see this message on the very first run.
      docker compose up -d --build
    else
      echo "Image found. Starting container from existing image..."
      # This starts a container from your existing, persistent image.
      docker compose up -d
    fi

    # Check if the container started successfully before proceeding
    if [ ! $(docker ps -q -f name=^/${CONTAINER_NAME}$) ]; then
        echo "Error: Container '${CONTAINER_NAME}' failed to start."
        exit 1
    fi
fi

echo "--------------------------------------------------------"
echo "Attaching to container and starting ROS node..."
echo "Press Ctrl+C to stop the node and detach from the container."
echo "--------------------------------------------------------"


# Execute the command inside the running container.
# The '-it' flags attach your terminal to the command, so you can see the
# output and stop it with Ctrl+C.
docker exec -it ${CONTAINER_NAME} /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && ros2 run agrobot_navigation roboclaw_wrapper"

# After the docker exec command is stopped (by you with Ctrl+C),
# perform a final cleanup.
cleanup

