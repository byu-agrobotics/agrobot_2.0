#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Syncs custom messages between the ROS2 and micro-ROS workspaces

# Set base path (optional, for reuse)
BASE_DIR=~/agrobot_2.0

# Sync message interfaces from ROS2 workspace to micro-ROS firmware
rsync -avc --delete "$BASE_DIR/agrobot_ws/src/agrobot_interfaces" "$BASE_DIR/firmware/teensy_pio/extra_packages"

# Navigate to firmware directory
cd "$BASE_DIR/firmware/teensy_pio"

pio run --target clean_microros
pio pkg install
pio run
