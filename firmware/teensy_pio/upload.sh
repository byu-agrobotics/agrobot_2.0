#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Builds and uploads the latest micro-ROS code to the Teensy board

# cd ~/firmware/teensy_pio
# pio run

# cd ~/firmware/teensy_pio/.pio/build/teensy41
# tycmd upload firmware.hex

cd ~/firmware/teensy_pio
pio run --target upload