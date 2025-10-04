ARG BASE_IMAGE=ghcr.io/openai/codex-universal:latest
ARG ROS_INSTALL=true
ARG DEBIAN_FRONTEND=noninteractive

FROM ${BASE_IMAGE} as base

# Metadata
LABEL maintainer="maintainer@example.com"
LABEL description="Workspace image based on ghcr.io/openai/codex-universal:latest with optional ROS Humble components"

# Allow passing a platform at docker run/build time, e.g. --platform=linux/amd64

# Set environment defaults
ENV LANG=en_US.UTF-8 \
        LC_ALL=en_US.UTF-8 \
        DEBIAN_FRONTEND=${DEBIAN_FRONTEND}

USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
        locales \
        ca-certificates \
        curl \
        ca-certificates \
        tzdata \
        && rm -rf /var/lib/apt/lists/*

# Configure locale
RUN locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

## Optionally install ROS Humble if build arg ROS_INSTALL is set to true
RUN if [ "${ROS_INSTALL}" = "true" ]; then \
        apt-get update && apt-get install -y --no-install-recommends gnupg2 lsb-release software-properties-common curl && \
        # add ROS apt source (uses ros-infrastructure package)
        ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
        curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
        dpkg -i /tmp/ros2-apt-source.deb || true && apt-get update && apt-get install -y --no-install-recommends ros-humble-desktop && \
        apt-get install -y --no-install-recommends ros-dev-tools && \
        rm -rf /var/lib/apt/lists/* /tmp/ros2-apt-source.deb ; fi

# Create a non-root user matching typical host UID/GID to make bind mounts writable
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} \
        && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -s /bin/bash \
        && mkdir -p /workspace

WORKDIR /workspace

USER ${USERNAME}

# Default entrypoint opens a shell; the user can mount the project workspace into /workspace
ENTRYPOINT ["/bin/bash"]

# Notes for use are included as a comment below
# To run interactively with the same defaults used by the codex example:
# docker run --rm -it \
#   -e CODEX_ENV_PYTHON_VERSION=3.12 \
#   -e CODEX_ENV_NODE_VERSION=20 \
#   -e CODEX_ENV_RUST_VERSION=1.87.0 \
#   -e CODEX_ENV_GO_VERSION=1.23.8 \
#   -e CODEX_ENV_SWIFT_VERSION=6.1 \
#   -e CODEX_ENV_RUBY_VERSION=3.4.4 \
#   -e CODEX_ENV_PHP_VERSION=8.4 \
#   -v $(pwd):/workspace/$(basename $(pwd)) -w /workspace/$(basename $(pwd)) \
#   --platform linux/amd64 \
#   ${BASE_IMAGE}
