#!/bin/bash
set -e

# Load environment variables
if [ -f .env ]; then
    export $(grep -v '^#' .env | xargs)
fi

# Import repositories from ros2.repos
vcs import < src/ros2.repos src

# ROS repository setup is already done in Dockerfile, but add if needed
# (ROS repositories and basic packages are already installed in the container)
sudo apt update

# Install target-specific dependencies
if [ "$TARGET" = "robot" ]; then
    echo "Installing robot (SBC) specific dependencies..."
    # Robot-specific packages for Raspberry Pi SBC
    sudo apt-get -y install --no-install-recommends \
        python3-venv \
        python3-pip \
        python3-colcon-common-extensions \
        rosdep \
        build-essential \
        python3-libgpiod \
        python3-pyudev \
        i2c-tools \
        micro-ros-agent \
        ros-humble-micro-ros-msgs

    # Enable hardware interfaces (I2C, SPI, camera)
    # Note: These may require raspi-config on actual Pi, but install tools
    sudo apt-get -y install raspi-config 2>/dev/null || true  # May not be available in all distros

    # Install Pico SDK if path is set
    if [ -n "$PICO_SDK_PATH" ] && [ ! -d "$PICO_SDK_PATH" ]; then
        echo "Installing Pico SDK to $PICO_SDK_PATH..."
        sudo apt-get -y install git cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
        git clone https://github.com/raspberrypi/pico-sdk.git "$PICO_SDK_PATH"
        cd "$PICO_SDK_PATH"
        git submodule update --init
        cd -
    fi

elif [ "$TARGET" = "remote_pc" ]; then
    echo "Installing remote PC (development) specific dependencies..."
    # Development and simulation packages
    sudo apt-get -y install --no-install-recommends \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
        ros-humble-joint-state-publisher-gui \
        ros-humble-rqt \
        ros-humble-rqt-image-view \
        ros-humble-rosbridge-suite \
        ros-humble-moveit \
        ros-humble-moveit-msgs \
        ros-humble-moveit-ros-planning \
        ros-humble-moveit-ros-planning-interface \
        ros-humble-moveit-servo \
        joystick \
        jstest-gtk \
        evtest

else
    echo "Warning: TARGET not set or unrecognized. Installing minimal ROS dependencies."
fi

# Update rosdep and install project-specific dependencies
rosdep update --rosdistro=$ROS_DISTRO
rosdep install --from-paths src --ignore-src -y --rosdistro=$ROS_DISTRO
