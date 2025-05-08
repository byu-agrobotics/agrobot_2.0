#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Launches tasks on the base station using the 'base_launch' tmux session

script_dir=$(dirname "$(readlink -f "$0")")
source $script_dir/tools/base_common.sh

# Check for a "-t <task>" argument
while getopts ":t:" opt; do
  case $opt in
    t)
      task=$OPTARG
      ;;
  esac
done

# Start the Docker containers if not already running
if [ $(docker ps | grep agrobot-ct | wc -l) -eq 0 ]; then
		printWarning "Starting the agrobot-ct container..."
		cd ~/agrobot_2.0/docker && docker compose up -d
fi
if [ $(docker ps | grep mapproxy | wc -l) -eq 0 ]; then
    printWarning "Starting the mapproxy container..."
    docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
fi

# Launch the specified task configuration over SSH
case $task in
    "autonomy")
        printInfo "Setting up the autonomy task..."
        # This envsubst allows for the use of environment variables in the tmuxp config
        envsubst < tmuxp/autonomy/base_launch.yaml > tmuxp/tmp/base_launch.yaml
        docker exec agrobot-ct tmuxp load -d /home/agrobot-docker/.tmuxp/base_launch.yaml
        ;;
    "servicing")
        printWarning "Not implemented yet"
        exit
        ;;
    "retrieval")
        printWarning "Not implemented yet"
        exit
        ;;
    "science")
        printWarning "Not implemented yet"
        exit
        ;;
    *)
        printError "No task specified"
        echo "Specify a task using 'bash base_launch.sh -t <task>' (ex. 'bash base_launch.sh -t autonomy')"
        exit 1
        ;;
esac

# Attach to the 'base_launch' tmux session
docker exec -it agrobot-ct tmux attach -t base_launch

# Kill the tmux session on exit
docker exec agrobot-ct tmux kill-session -t base_launch
