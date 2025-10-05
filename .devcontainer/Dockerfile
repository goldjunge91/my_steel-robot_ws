FROM althack/ros2:humble-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install --no-install-recommends <your-package-list-here> \
#    #
#    # Clean up
#    && apt-get autoremove -y \
#    && apt-get clean -y \
#    && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
RUN sudo usermod -aG docker ros
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc


# ##########################
# FROM althack/ros2:humble-dev
# # FROM ubuntu:22.04

# # CI-optimized image for my_steel-robot_ws
# # - Uses BuildKit cache mounts for apt/pip/ccache
# # - Provides build args to skip heavy groups (Gazebo/MoveIt) for fast iteration
# # - Installs essential tools for build/test/lint workflows used by .github/actions

# ARG DEBIAN_FRONTEND=noninteractive
# ARG INSTALL_GAZEBO=true
# ARG INSTALL_MOVEIT=true
# ARG INSTALL_LINT_TOOLS=true
# ARG INSTALL_PICO_SDK=false

# ENV ROS_DISTRO=humble

# RUN apt-get update && apt-get install -y --no-install-recommends \
#         curl gnupg lsb-release ca-certificates software-properties-common \
#         && rm -rf /var/lib/apt/lists/*

# # Setup ROS2 apt repository (Ubuntu 22.04 -> jammy)
# RUN set -eux; \
#         curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg; \
#         echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2.list

# # Prepare buildkit-friendly apt installs (cache mounts used at build time)
# RUN apt-get update && apt-get install -y --no-install-recommends \
#         build-essential ca-certificates git cmake pkg-config gnupg2 python3 python3-pip python3-venv \
#         python3-colcon-common-extensions python3-rosdep python3-vcstool python3-setuptools python3-wheel \
#         ninja-build ccache cmake ninja-build \
#         gcc-arm-none-eabi binutils-arm-none-eabi gdb-multiarch \
#         && apt-get clean && rm -rf /var/lib/apt/lists/*

# # Add a non-root 'ros' user and set up home
# RUN useradd -m -s /bin/bash ros || true
# RUN mkdir -p /home/ros/.local/bin && chown -R ros:ros /home/ros

# USER ros
# WORKDIR /home/ros

# # Ensure pip cache dir exists for cache mounts
# RUN python3 -m pip install --user -U pip setuptools wheel

# USER root

# # Install optional heavy groups using BuildKit cache mounts when enabled
# RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt/lists \
#         sh -c 'set -eux; apt-get -o Acquire::Check-Valid-Until=false update; \
#         PKGS="python3-colcon-common-extensions python3-rosdep git cmake build-essential ninja-build"; \
#         if [ "${INSTALL_GAZEBO}" = "true" ]; then PKGS="$PKGS ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control"; fi; \
#         if [ "${INSTALL_MOVEIT}" = "true" ]; then PKGS="$PKGS ros-humble-moveit ros-humble-moveit-msgs ros-humble-moveit-ros-planning"; fi; \
#         if [ "${INSTALL_LINT_TOOLS}" = "true" ]; then PKGS="$PKGS cppcheck clang-format python3-flake8 python3-pep257 xmllint"; fi; \
#         apt-get -y install --no-install-recommends $PKGS || true; \
#         apt-get autoremove -y; apt-get clean; rm -rf /var/lib/apt/lists/*'

# # Optionally install pico SDK/tools at build time (disabled by default)
# ARG INSTALL_PICO_SDK=false
# RUN if [ "$INSTALL_PICO_SDK" = "true" ]; then \
#         set -eux; apt-get update && apt-get install -y --no-install-recommends git cmake ninja-build pkg-config build-essential gcc-arm-none-eabi binutils-arm-none-eabi; \
#         git clone --depth 1 https://github.com/raspberrypi/pico-sdk.git /opt/pico-sdk || true; \
#         rm -rf /var/lib/apt/lists/*; \
#         fi

# # Initialize rosdep (as root) — this is relatively fast; keeps image ready for rosdep usage
# RUN set -eux; rosdep update || true

# # Final cleanup
# RUN apt-get autoremove -y || true; apt-get clean || true; rm -rf /var/lib/apt/lists/* /tmp/*

# ENV PATH=/home/ros/.local/bin:$PATH
# ENV DEBIAN_FRONTEND=dialog

# USER ros
# CMD ["bash"]

# FROM althack/ros2:humble-dev

# # CI-optimized Dockerfile
# # Features:
# # - BuildKit cache mounts for apt, pip, and ccache
# # - Build args to optionally enable heavy packages (Gazebo, MoveIt, extra tools)
# # - Non-interactive, combined apt runs and cleanup

# USER root
# ARG INSTALL_GAZEBO=false
# ARG INSTALL_MOVEIT=false
# ARG INSTALL_EXTRA_TOOLS=false
# ARG INSTALL_PICO_SDK=false
# ARG INSTALL_DOPPLER=false
# ARG WORKSPACE=/workspace
# ENV DEBIAN_FRONTEND=noninteractive

# # Install core packages. Use BuildKit cache mounts. This speeds up repeated builds in CI and locally.
# RUN --mount=type=cache,target=/var/cache/apt \
#         --mount=type=cache,target=/var/lib/apt/lists \
#         --mount=type=cache,target=/root/.cache/pip \
#         --mount=type=cache,target=/root/.ccache \
#         sh -c '\
#         set -eux; \
#         export APT_OPTS="-o Acquire::Check-Valid-Until=false"; \
#         apt-get $APT_OPTS update; \
#         apt-get $APT_OPTS install -y --no-install-recommends \
#         ca-certificates curl gnupg lsb-release software-properties-common \
#         build-essential cmake pkg-config ninja-build ccache python3-pip python3-venv \
#         gcc-arm-none-eabi binutils-arm-none-eabi gdb-multiarch \
#         python3-colcon-common-extensions; \
#         rm -rf /var/lib/apt/lists/*'

# # Optionally install heavy ROS packages (Gazebo/MoveIt/extra) based on build args
# RUN --mount=type=cache,target=/var/cache/apt \
#         --mount=type=cache,target=/var/lib/apt/lists \
#         sh -c '\
#         set -eux; \
#         export APT_OPTS="-o Acquire::Check-Valid-Until=false"; \
#         apt-get $APT_OPTS update; \
#         PKGS=""; \
#         if [ "${INSTALL_GAZEBO}" = "true" ]; then \
#         PKGS="$PKGS ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control"; \
#         fi; \
#         if [ "${INSTALL_MOVEIT}" = "true" ]; then \
#         PKGS="$PKGS ros-humble-moveit ros-humble-moveit-msgs ros-humble-moveit-ros-planning \
#         ros-humble-moveit-ros-planning-interface ros-humble-moveit-servo"; \
#         fi; \
#         if [ "${INSTALL_EXTRA_TOOLS}" = "true" ]; then \
#         PKGS="$PKGS ros-humble-joint-state-publisher-gui ros-humble-rqt-image-view \
#         ros-humble-rosbridge-suite ros-humble-rqt ros-humble-micro-ros-msgs"; \
#         fi; \
#         if [ -n "$PKGS" ]; then \
#         apt-get $APT_OPTS install -y --no-install-recommends $PKGS; \
#         fi; \
#         apt-get autoremove -y; \
#         rm -rf /var/lib/apt/lists/*'

# # Optional: install Doppler CLI if requested
# RUN --mount=type=cache,target=/var/cache/apt \
#         sh -c '\
#         if [ "${INSTALL_DOPPLER}" = "true" ]; then \
#         set -eux; \
#         curl -sLf --retry 3 --tlsv1.2 --proto "=https" "https://packages.doppler.com/public/cli/gpg.DE2A7741A397C129.key" \
#         | gpg --batch --yes --dearmor -o /usr/share/keyrings/doppler-archive-keyring.gpg; \
#         echo "deb [signed-by=/usr/share/keyrings/doppler-archive-keyring.gpg] https://packages.doppler.com/public/cli/deb/debian any-version main" \
#         > /etc/apt/sources.list.d/doppler-cli.list; \
#         apt-get -o Acquire::Check-Valid-Until=false update; \
#         apt-get -o Acquire::Check-Valid-Until=false install -y doppler; \
#         rm -rf /var/lib/apt/lists/*; \
#         fi'

# # Optional: build Pico SDK / picotool during image build (can be slow). Disabled by default.
# COPY install_picotool_and_sdk.sh /tmp/install_picotool_and_sdk.sh
# RUN if [ "${INSTALL_PICO_SDK}" = "true" ]; then \
#         chmod +x /tmp/install_picotool_and_sdk.sh && /tmp/install_picotool_and_sdk.sh /opt/pico-sdk; \
#         fi || true

# # Configure ros user environment to auto-source workspace if present
# ARG WORKSPACE
# RUN mkdir -p /home/ros && echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc || true

# ENV DEBIAN_FRONTEND=dialog

# USER ros
# WORKDIR /home/ros