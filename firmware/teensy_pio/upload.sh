#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Builds and uploads the latest micro-ROS code to the Teensy board

sudo chmod 666 /dev/ttyACM0

# cd ~/firmware/teensy_pio
# pio run

# cd ~/firmware/teensy_pio/.pio/build/teensy41
# tycmd upload firmware.hex


# Exit immediately if a command exits with a non-zero status.
set -e

echo "Building firmware..."
cd ~/firmware/teensy_pio
echo ""
echo "--------------------------------------------------------------------"
echo "--- UPLOAD ATTEMPT 1: Forcing board into bootloader mode."
echo "--- This attempt is expected to fail, which is normal."
echo "--------------------------------------------------------------------"

# Run the upload command but add '|| true' so the script doesn't exit if it fails.
pio run --target upload || true

echo ""
echo "Pausing for 2 seconds to allow the USB device to settle..."
sleep 2

echo ""
echo "--------------------------------------------------------------------"
echo "--- UPLOAD ATTEMPT 2: Performing the actual firmware upload."
echo "--------------------------------------------------------------------"

# The Teensy is now stable in bootloader mode. This second run will succeed.
pio run --target upload

echo ""
echo "✅ ✅ ✅ Build and Upload complete! ✅ ✅ ✅"

