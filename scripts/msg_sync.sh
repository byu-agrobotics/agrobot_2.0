#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Syncs custom messages between the ROS2 and micro-ROS workspaces

# Set base path (optional, for reuse)
BASE_DIR=~/agrobot_2.0

rsync -avc --delete ~/agrobot_ws/src/agrobot_interfaces ~/firmware/teensy_pio/extra_packages

cd ~/firmware/teensy_pio
pio run --target clean_microros
pio pkg install
# pio run
pio run -j1

