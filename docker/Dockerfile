# Created by Nelson Durrant, Oct 2024
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH

# Define a username and password for the new user
ARG NAME=agrobot-docker
ARG PASS=agrobot

# Update and upgrade
RUN apt update && apt upgrade -y

# Add tytools (as root)
RUN mkdir -p -m0755 /etc/apt/keyrings
RUN apt update && apt install -y curl \
    wget \
    unzip \
    python3-pip
RUN curl https://download.koromix.dev/debian/koromix-archive-keyring.gpg -o /etc/apt/keyrings/koromix-archive-keyring.gpg
RUN echo "deb [signed-by=/etc/apt/keyrings/koromix-archive-keyring.gpg] https://download.koromix.dev/debian stable main" > /etc/apt/sources.list.d/koromix.dev-stable.list
RUN apt update && apt install -y tytools

# Set up a new user
RUN useradd -ms /bin/bash $NAME
RUN usermod -aG sudo $NAME
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN usermod -aG video $NAME
RUN usermod -aG dialout $NAME
RUN groupadd docker
RUN usermod -aG docker $NAME
RUN echo "$NAME:$PASS" | chpasswd
USER $NAME
WORKDIR /home/$NAME

# Install general dependencies
USER root
RUN apt update && apt install -y \
    curl \
    wget \
    rsync \
    git \
    vim \
    nano \
    tmux \
    python3-pip
USER $NAME

# Install graphical interface dependencies
USER root
RUN apt update && apt install -y \
    x11-apps \
    x11-utils \
    x11-xserver-utils \
    xauth 
USER $NAME

# Install PlatformIO
USER root
RUN apt install -y python3-venv
USER $NAME

RUN curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
RUN python3 get-platformio.py
RUN rm get-platformio.py

# Set up PlatformIO shell commands
RUN mkdir -p /usr/local/bin

USER root
RUN ln -s /home/$NAME/.platformio/penv/bin/platformio /usr/local/bin/platformio
RUN ln -s /home/$NAME/.platformio/penv/bin/pio /usr/local/bin/pio
RUN ln -s /home/$NAME/.platformio/penv/bin/piodebuggdb /usr/local/bin/piodebuggdb
USER $NAME

# Install the micro-ROS agent
RUN mkdir microros_ws

WORKDIR /home/$NAME/microros_ws
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
# The below is a quick fix for segfaulting builds - Nelson Durrant, May 2025
# It looks like someone broke the micro-ROS setup script, so we need to use a specific commit
WORKDIR /home/$NAME/microros_ws/src/micro_ros_setup
RUN git reset --hard 91a554f
WORKDIR /home/$NAME/microros_ws
RUN rosdep update

USER root
RUN rosdep install --from-paths src --ignore-src -y
USER $NAME

SHELL ["/bin/bash", "-c"] 
RUN source /opt/ros/humble/setup.bash && colcon build
RUN source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run micro_ros_setup create_agent_ws.sh
RUN source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run micro_ros_setup build_agent.sh
SHELL ["/bin/sh", "-c"]

WORKDIR /home/$NAME

# Dry run PlatformIO to install the necessary dependencies
RUN git clone https://github.com/byu-agrobotics/agrobot_2.0.git
# '|| true' is used to ignore errors on a pio command failure
WORKDIR /home/$NAME/agrobot_2.0/firmware/teensy_pio
RUN pio run -t upload || true
WORKDIR /home/$NAME
RUN rm -rf /home/$NAME/agrobot_2.0

# Set up automatic ROS 2 sourcing and colorized output
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/$NAME/microros_ws/install/setup.bash" >> /home/$NAME/.bashrc
RUN echo "source /home/$NAME/agrobot_ws/install/setup.bash" >> /home/$NAME/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
RUN touch ~/.hushlogin

# Set up tmuxp for scripting purposes
RUN pip3 install tmuxp

# Add local python binaries to the PATH
ENV PATH="$PATH:/home/$NAME/.local/bin"

###############################################################
# IMPORTANT! When adding dependencies or Dockerfile commands
# below, please document your changes following this format:
#     
#     # What do these commands do? - Full Name, Month Year
#     # Why do we need them? Which ROS 2 nodes require them?
#     < Add Dockerfile commands here >
#
# https://docs.docker.com/reference/dockerfile/
#
###############################################################

# Install graphical interface packages - Nelson Durrant, Feb 2025
# We use RViz2, RQT, and Python3-tk for GUIs
USER root
RUN apt update && apt install -y \
    ros-humble-rviz2 \
    ros-humble-rqt* \
    python3-tk
USER $NAME

# NOTE: For some reason, this is broken as of Jun 2025
# Install plotjuggler - Nelson Durrant, Feb 2025
# Useful for plotting and visualizing data from ROS 2 bags and real-time tests
# USER root
# RUN apt update && apt install -y ros-humble-plotjuggler-ros
# USER $NAME

# Install mobility packages - Nelson Durrant, Feb 2025
# Used for controlling the robot's movement through a joystick or keyboard
USER root
RUN apt update && apt install -y \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-teleop-twist-keyboard \
    ros-humble-topic-tools
USER $NAME

# Install turtlesim tf2 packages - Nelson Durrant, Feb 2025
# Used in the ROS 2 and tf2 tutorials
USER root
RUN apt update && apt install -y \
    ros-humble-turtle-tf2-py \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-turtlesim
USER $NAME

# Install OpenCV packages - Ashton Palacios, Feb 2025
# Used for egg computer vision and image processing
USER root
RUN apt update && apt install -y \
    libopencv-dev \
    python3-opencv \
    v4l-utils
USER $NAME

# Install the serial and pyserial libraries - Nelson Durrant, May 2025
# Used by the roboclaw library for serial communication

RUN pip3 install serial pyserial 

# Used by the drive_controller node for PID control
RUN pip3 install simple-pid

