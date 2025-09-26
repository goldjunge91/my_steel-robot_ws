#!/bin/bash
set -e

# Load environment variables (support files with CRLF)
if [ -f .env ]; then
    echo "Loading environment variables from .env file..."
    # Use sed to strip CRLF if present, then export non-comment lines
    eval "$(sed 's/\r$//' .env | grep -v '^#' | xargs)"
fi

# Provide sane defaults if not set in environment or .env
TARGET=${TARGET:-robot}
ROS_DISTRO=${ROS_DISTRO:-humble}

# Helper: ensure ROS apt repository is present (required to install ros-humble-* packages)
ensure_ros_apt() {
    if ! grep -Rq "packages.ros.org" /etc/apt/sources.list.d 2>/dev/null; then
        echo "Adding ROS 2 apt repository..."
        sudo apt-get install -y --no-install-recommends curl gnupg2 lsb-release
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
        sudo sh -c "echo \"deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main\" > /etc/apt/sources.list.d/ros2.list"
        sudo apt update
    fi
}

# Import repositories from ros2.repos (guarded)
# ROS repository setup is already done in Dockerfile, but add if needed
# (ROS repositories and basic packages are already installed in the container)

# Make sure ROS apt repository is available when installing ros-humble packages
ensure_ros_apt

# Install target-specific dependencies
if [ "$TARGET" = "robot" ]; then
    echo "Hello world"
    echo "Installing robot (SBC) specific dependencies..."
    export ROS_DISTRO="humble" 
    # sudo apt-get update -y
    # sudo apt-get upgrade -y
    # Robot-specific packages for Raspberry Pi SBC (only those available on ARM)
    sudo apt-get -y install --no-install-recommends \
        python3-venv \
        python3-pip \
        python3-colcon-common-extensions \
        dos2unix \
        ros-humble-ros-core \
        python3-rosdep \
        build-essential \
        python3-libgpiod \
        python3-pyudev \
        i2c-tools \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-xacro \
        ros-humble-urdf \
        ros-humble-urdf-launch \
        ros-humble-urdf-tutorial \
        ros-humble-twist-mux \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-rmw-fastrtps-shared-cpp \
        joystick \
        jstest-gtk \
        evtest \
        # micro-ros-agent

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

# Check if repo directory exists
if [ ! -d src/ ]; then
    vcs import < src/ros2.repos src
fi

# Update rosdep and install project-specific dependencies
# Ensure rosdep is available; try to install python3-rosdep if missing
if ! command -v rosdep >/dev/null 2>&1; then
    echo "rosdep not found — attempting to install python3-rosdep"
    sudo apt-get update || true
    sudo apt-get -y install python3-rosdep || echo "python3-rosdep not available via apt, please install rosdep manually"
fi
sudo rosdep init 2>/dev/null || true
rosdep update --rosdistro=$ROS_DISTRO || echo "rosdep update failed — check network and ROS distro"
rosdep install --from-paths src --ignore-src -y --rosdistro=$ROS_DISTRO || echo "rosdep install failed or some dependencies unavailable"
