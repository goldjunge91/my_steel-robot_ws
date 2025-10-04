FROM althack/ros2:humble-dev

# Use root for package installation steps
USER root
ENV DEBIAN_FRONTEND=noninteractive

# Allow skipping Doppler install (useful for debugging/build isolation)
ARG INSTALL_DOPPLER=true

# Target platform argument
ARG TARGET=remote_pc

# Install minimal packages needed by installer script (git, build tools, cmake, libusb)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    cmake \
    pkg-config \
    libusb-1.0-0-dev \
    libudev-dev \
    ninja-build \
    ccache \
    && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# Copy and run picotool / pico-sdk installer (run as root during image build)
COPY install_picotool_and_sdk.sh /tmp/install_picotool_and_sdk.sh
RUN chmod +x /tmp/install_picotool_and_sdk.sh \
    && (set -x; /tmp/install_picotool_and_sdk.sh /opt/pico-sdk) \
    && rm -f /tmp/install_picotool_and_sdk.sh

# Tell SDK/CMake to prefer ninja
ENV PICO_SDK_PATH=/opt/pico-sdk
ENV CMAKE_GENERATOR=Ninja
ENV PICOTOOL_FETCH_FROM_GIT_PATH=/opt/picotool-cache
ENV CMAKE_GENERATOR=Ninja

# Install essential ROS2 packages and useful tools
RUN apt-get update \
    && apt-get -y install --no-install-recommends \
    cmake \
    gcc-arm-none-eabi \
    binutils-arm-none-eabi \
    gdb-multiarch \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-urdf \
    ros-humble-urdf-launch \
    ros-humble-urdf-tutorial \
    ros-humble-twist-mux \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rqt-image-view \
    ros-humble-rosbridge-suite \
    ros-humble-rqt \
    ros-humble-moveit \
    ros-humble-moveit-msgs \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-servo \
    ros-humble-micro-ros-msgs \
    python3-colcon-common-extensions \
    software-properties-common \
    curl \
    apt-transport-https \
    ca-certificates \
    gnupg ;

# Install target-specific packages
RUN if [ "$TARGET" = "remote_pc" ]; then \
    echo "Installing remote PC packages..." && \
    apt-get -y install --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rqt-image-view \
    ros-humble-rosbridge-suite \
    ros-humble-rqt \
    ros-humble-moveit \
    ros-humble-moveit-msgs \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-servo \
    ros-humble-micro-ros-msgs \
    python3-colcon-common-extensions \
    joystick jstest-gtk evtest \
    software-properties-common \
    curl \
    apt-transport-https \
    ca-certificates \
    gnupg ; \
    elif [ "$TARGET" = "robot" ]; then \
    echo "Installing robot (SBC) packages..." && \
    apt-get -y install --no-install-recommends \
    python3-venv \
    python3-pip \
    build-essential \
    python3-libgpiod \
    python3-pyudev \
    i2c-tools \
    ros-humble-micro-ros-msgs; \
    else \
    echo "Warning: TARGET not set or unrecognized. Installing minimal ROS dependencies."; \
    fi \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*


# Install Doppler CLI (add key and repo, then install)
RUN if [ "${INSTALL_DOPPLER}" = "true" ]; then \
    set -eux; \
    curl -sLf --retry 3 --tlsv1.2 --proto "=https" 'https://packages.doppler.com/public/cli/gpg.DE2A7741A397C129.key' \
    | gpg --batch --yes --dearmor -o /usr/share/keyrings/doppler-archive-keyring.gpg; \
    echo "deb [signed-by=/usr/share/keyrings/doppler-archive-keyring.gpg] https://packages.doppler.com/public/cli/deb/debian any-version main" \
    > /etc/apt/sources.list.d/doppler-cli.list; \
    apt-get update && apt-get -y install doppler && apt-get autoremove -y && apt-get clean -y; \
    rm -rf /var/lib/apt/lists/*; \
    else \
    echo "Skipping Doppler install (INSTALL_DOPPLER=${INSTALL_DOPPLER})"; \
    fi

RUN apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
ENV ROS_DISTRO=humble
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# Create minimal git config for ros user (environment variables will be expanded at runtime)
RUN mkdir -p /home/ros && \
    printf '%s\n' '[safe]' '    directory = *' '[credential]' '    helper = store' > /home/ros/.gitconfig && \
    chown ros:ros /home/ros/.gitconfig && chmod 644 /home/ros/.gitconfig

# Add git identity setup to bashrc (so it runs when container starts)
RUN echo '' >> /home/ros/.bashrc && \
    echo '# Set git identity from environment variables' >> /home/ros/.bashrc && \
    echo 'if [ -n "$GIT_AUTHOR_NAME" ]; then' >> /home/ros/.bashrc && \
    echo '    git config --global user.name "$GIT_AUTHOR_NAME"' >> /home/ros/.bashrc && \
    echo 'fi' >> /home/ros/.bashrc && \
    echo 'if [ -n "$GIT_AUTHOR_EMAIL" ]; then' >> /home/ros/.bashrc && \
    echo '    git config --global user.email "$GIT_AUTHOR_EMAIL"' >> /home/ros/.bashrc && \
    echo 'fi' >> /home/ros/.bashrc && \
    echo 'if [ -n "$GIT_COMMITTER_NAME" ]; then' >> /home/ros/.bashrc && \
    echo '    git config --global committer.name "$GIT_COMMITTER_NAME"' >> /home/ros/.bashrc && \
    echo 'fi' >> /home/ros/.bashrc && \
    echo 'if [ -n "$GIT_COMMITTER_EMAIL" ]; then' >> /home/ros/.bashrc && \
    echo '    git config --global committer.email "$GIT_COMMITTER_EMAIL"' >> /home/ros/.bashrc && \
    echo 'fi' >> /home/ros/.bashrc && \
    chown ros:ros /home/ros/.bashrc

USER ros
