#!/bin/bash
# Created by Braden Meyers & Nelson Durrant, Mar 2025
#
# Set up a SSH key for passwordless access to the robot

script_dir=$(dirname "$(readlink -f "$0")")
source $script_dir/base_common.sh

# Check for "-u <username>" or "-p <password>" arguments
while getopts ":u:p:" opt; do
  case $opt in
    u)
      ROBOT_USERNAME=$OPTARG
      ;;
    p)
      ROBOT_PASSWORD=$OPTARG
      ;;
  esac
done

# Function to remove an old SSH key from known_hosts
# This prevents SSH errors if the host's key has changed.
remove_old_ssh_key() {
    local host=$1
    local port=$2

    if [[ "$port" == "22" ]]; then
        # Remove without specifying port notation for default SSH port
        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "$host" &> /dev/null
    else
        # Remove using port notation for non-default ports
        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "[$host]:$port" &> /dev/null
    fi
}

# Check if ssh pass is installed
if ! command -v sshpass &> /dev/null; then
    printWarning "Please install 'sshpass' using (sudo apt install sshpass)"
    ROBOT_CONNECT="ssh"
else
    ROBOT_CONNECT="sshpass -p $ROBOT_PASSWORD ssh"
fi

# Remove old SSH key (if it exists) to prevent key mismatch issues
remove_old_ssh_key "$ROBOT_IP_ADDRESS" "22"

# Test the SSH connection to accept the new key if prompted
$ROBOT_CONNECT -o StrictHostKeyChecking=accept-new $ROBOT_USERNAME@$ROBOT_IP_ADDRESS "echo" &> /dev/null

# Copy the SSH key to the robot
ssh-copy-id $ROBOT_USERNAME@$ROBOT_IP_ADDRESS
