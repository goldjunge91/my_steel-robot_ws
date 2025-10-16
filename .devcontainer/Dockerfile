FROM althack/ros2:humble-dev

# Build arguments
ARG TARGET=remote_pc
ARG INSTALL_PICO_SDK=false
ARG WORKSPACE=/workspace

# Environment variables
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

# Install pytest plugins required by setup.cfg and robot_control_gui/requirements.txt
RUN python3 -m pip install --no-cache-dir \
    pytest-timeout \
    pytest-cov

# Install vcstool to ensure it's available in PATH
RUN python3 -m pip install --no-cache-dir vcstool

# Verify base image has required ROS2 packages
# The althack/ros2:humble-dev image includes:
# - ros-humble-desktop (ROS2 core, rviz2, rqt, etc.)
# - python3-colcon-common-extensions
# - python3-rosdep
# - build-essential, cmake, git
# Additional packages will be installed via rosdep in setup.sh

# Configure git safe.directory to avoid permission errors in container
RUN git config --global --add safe.directory '*'

# Set up auto-source of workspace for ros user
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# Optional: Install Pico SDK if requested (disabled by default for CI)
RUN if [ "$INSTALL_PICO_SDK" = "true" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            gcc-arm-none-eabi \
            binutils-arm-none-eabi \
            gdb-multiarch && \
        git clone --depth 1 https://github.com/raspberrypi/pico-sdk.git /opt/pico-sdk && \
        cd /opt/pico-sdk && git submodule update --init && \
        rm -rf /var/lib/apt/lists/*; \
    fi

ENV DEBIAN_FRONTEND=dialog

USER ros
WORKDIR /home/ros