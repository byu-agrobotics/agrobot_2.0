#!/bin/bash
# Sync custom messages between ROS 2 and micro-ROS workspaces

set -e

echo "[msg_sync] Syncing message package..."

# Step 1: Sync source .msg and package files
rsync -avc --delete ~/agrobot_ws/src/agrobot_interfaces ~/firmware/teensy_pio/extra_packages

# Step 2: Ensure the micro_ros config points to extra packages
MICROROS_CONFIG=~/firmware/teensy_pio/.micro_ros.config.yml
if [ ! -f "$MICROROS_CONFIG" ]; then
  echo "[msg_sync] Creating .micro_ros.config.yml"
  cat <<EOF > "$MICROROS_CONFIG"
build:
  packages:
    - extra_packages/agrobot_interfaces
EOF
fi

# Step 3: Rebuild
cd ~/firmware/teensy_pio
pio run --target clean_microros
pio pkg install
pio run





# #!/bin/bash
# # Created by Nelson Durrant, Feb 2025
# #
# # Builds and uploads the latest micro-ROS code to the Teensy board

# cd ~/firmware/teensy_pio
# pio run

# cd ~/firmware/teensy_pio/.pio/build/teensy41
# tycmd upload firmware.hex


# # cd ~/firmware/teensy_pio
# # pio run

# # echo "Please press the RESET button on the Teensy NOW to enter bootloader mode..."
# # read -p "Press Enter after pressing reset..."

# # cd ~/firmware/teensy_pio/.pio/build/teensy41
# # tycmd upload firmware.hex

# # cd ~/firmware/teensy_pio
# # pio run --target upload
