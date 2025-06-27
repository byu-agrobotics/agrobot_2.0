#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Launches tasks on the robot over SSH using the 'robot_launch' tmux session

script_dir=$(dirname "$(readlink -f "$0")")
source $script_dir/base_common.sh

# Check for a "-u <username>" argument
while getopts ":u:" opt; do
  case $opt in
    u)
      ROBOT_USERNAME=$OPTARG
      ;;
  esac
done

# Check for an SSH connection to the robot
checkConnection # defined in base_common.sh

# Start the Docker container if not already running
if [ ! $(ssh $ROBOT_USERNAME@$ROBOT_IP_ADDRESS "docker ps" | grep -q "agrobot-ct") ]; then
    printWarning "Starting the agrobot-ct container on the robot..."
    ssh -t $ROBOT_USERNAME@$ROBOT_IP_ADDRESS "cd ~/agrobot_2.0/docker && docker compose up -d"
fi

# This envsubst allows for the use of environment variables in the tmuxp config
envsubst < tmuxp/robot_launch.yaml > tmuxp/tmp/robot_launch.yaml
scp tmuxp/tmp/robot_launch.yaml $ROBOT_USERNAME@$ROBOT_IP_ADDRESS:~/agrobot_2.0/base_scripts/tmuxp/tmp/
ssh $ROBOT_USERNAME@$ROBOT_IP_ADDRESS \
  "docker exec agrobot-ct tmuxp load -d /home/agrobot-docker/.tmuxp/robot_launch.yaml"

# Attach to the 'robot_launch' tmux session (with mosh)
# https://github.com/mobile-shell/mosh
mosh $ROBOT_USERNAME@$ROBOT_IP_ADDRESS -- docker exec -it agrobot-ct tmux attach -t robot_launch

# Kill the tmux session on exit
ssh $ROBOT_USERNAME@$ROBOT_IP_ADDRESS "docker exec agrobot-ct tmux kill-session -t robot_launch"
