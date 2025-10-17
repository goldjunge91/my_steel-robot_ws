FROM osrf/ros:humble-desktop-full

# Build arguments
ARG TARGET=remote_pc
ARG INSTALL_PICO_SDK=false
ARG WORKSPACE=/workspace

# Environment variables
ENV ROS_DISTRO=humble
# Set up dev packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get -y install --no-install-recommends \
    bash-completion \
    gdb \
    gettext-base \
    git-core \
    openssh-client \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcs2l \
    sudo \
    # For GUI
    libx11-xcb1 libxcb1 libxcb-render0 libxcb-shm0 libxcb-xfixes0 libxcb-shape0 \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 libxcb-util1 \
    libxcb-xinerama0 libxcb-xkb1 libxkbcommon0 libxkbcommon-x11-0 \
    libglu1-mesa libxi6 libsm6 libxrender1 libfontconfig1 \
    # Clean up
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

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


# Set up auto-source of workspace for ros 
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Find and replace any user with matching UID
RUN set -eux; \
    existing_user=$(getent passwd "$USER_UID" | cut -d: -f1 || true); \
    if [ -n "$existing_user" ]; then \
    usermod -l $USERNAME -d /home/$USERNAME -m $existing_user && \
    groupmod -n $USERNAME $existing_user && \
    echo "Renamed $existing_user to $USERNAME"; \
    else \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "Created new user $USERNAME"; \
    fi
# Ensure necessary directories and permissions
RUN mkdir -p /home/$USERNAME /run/user/$USER_UID && \
    chown -R $USER_UID:$USER_GID /home/$USERNAME /run/user/$USER_UID

# Add sudo support for the non-root user
RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && touch /home/$USERNAME/.sudo_as_admin_successful

# Set up autocompletion for user
RUN  echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /usr/share/vcs2l-completion/vcs.bash ]; then source /usr/share/vcs2l-completion/vcs.bash; fi" >> /home/$USERNAME/.bashrc \
    # Set up auto-source of workspace for user
    && echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Set up python symlinks for vscode paths
RUN PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info[0]}.{sys.version_info[1]}')") \
    && tgt="/opt/ros/${ROS_DISTRO}/lib/python${PYTHON_VERSION}/site-packages" \
    && [ -d "$tgt" ] && ln -s "$tgt" "/opt/ros/${ROS_DISTRO}/lib/ros_site_packages"

ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1