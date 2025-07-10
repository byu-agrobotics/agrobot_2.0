#!/bin/bash

# Function to clean up background processes
cleanup() {
    echo "Stopping video stream..."
    # Kill the ffmpeg process, which will also stop libcamera-vid
    sudo pkill ffmpeg
    sudo pkill libcamera-vid
    echo "Video stream stopped."
    exit 0
}

# Trap Ctrl+C (SIGINT) and call the cleanup function
trap cleanup SIGINT

set -e

echo "Stopping any running libcamera-vid or ffmpeg processes..."
sudo pkill libcamera-vid || true
sudo pkill ffmpeg || true

echo "Checking if v4l2loopback module is loaded..."
if lsmod | grep -q v4l2loopback; then
    echo "v4l2loopback module is loaded, removing it..."
    sudo modprobe -r v4l2loopback
else
    echo "v4l2loopback module not loaded."
fi

echo "Loading v4l2loopback module with parameters..."
sudo modprobe v4l2loopback devices=1 video_nr=8 card_label="EggCam" exclusive_caps=1 max_buffers=2

echo "Starting video stream (libcamera-vid -> ffmpeg -> /dev/video8)..."
libcamera-vid -t 0 --inline --width 640 --height 480 --framerate 30 --codec yuv420 --output - | \
ffmpeg -f rawvideo -pix_fmt yuv420p -s 640x480 -i - -f v4l2 -pix_fmt rgb24 /dev/video8 > ~/video_stream.log 2>&1 &

echo "Video stream started in background. Logs: ~/video_stream.log"
echo "Press Ctrl+C to stop the stream."

# Wait indefinitely to keep the script running
# so it can be interrupted by Ctrl+C
wait