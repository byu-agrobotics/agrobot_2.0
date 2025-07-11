#!/bin/bash

set -e

# Function to clean up background processes and modules
cleanup() {
    echo "[CLEANUP] Stopping video stream..."

    # Kill the background streaming processes
    sudo pkill -f libcamera-vid || true
    sudo pkill -f ffmpeg || true

    # Wait a moment for processes to fully shut down
    sleep 1

    # Check and unload v4l2loopback if loaded
    if lsmod | grep -q v4l2loopback; then
        echo "[CLEANUP] Removing v4l2loopback module..."
        sudo modprobe -r v4l2loopback || true
    fi

    echo "[CLEANUP] Cleanup done. Exiting."
    exit 0
}

# Trap Ctrl+C (SIGINT) and SIGTERM to clean up properly
trap cleanup SIGINT SIGTERM

echo "[INIT] Killing any previous processes..."
sudo pkill -f libcamera-vid || true
sudo pkill -f ffmpeg || true

# Sleep briefly to ensure processes are fully terminated
sleep 1

echo "[INIT] Unloading any existing v4l2loopback..."
sudo modprobe -r v4l2loopback || true
sleep 1

echo "[INIT] Loading v4l2loopback module..."
sudo modprobe v4l2loopback devices=1 video_nr=8 card_label="EggCam" exclusive_caps=1 max_buffers=2

echo "[START] Starting libcamera-vid stream to ffmpeg -> /dev/video8..."
libcamera-vid -t 0 --inline --width 640 --height 480 --framerate 30 --codec yuv420 --output - | \
ffmpeg -f rawvideo -pix_fmt yuv420p -s 640x480 -i - -f v4l2 -pix_fmt rgb24 /dev/video8 > ~/video_stream.log 2>&1 &

echo "[RUNNING] Video stream started in background. Logs: ~/video_stream.log"
echo "Press Ctrl+C to stop the stream."

# Wait indefinitely
wait
