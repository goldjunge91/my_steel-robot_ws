#!/bin/bash
set -e

# Import repositories from ros2.repos
vcs import < src/ros2.repos src

# ROS repository setup is already done in Dockerfile, but add if needed
# (ROS repositories and basic packages are already installed in the container)
sudo apt update
# Update rosdep and install project-specific dependencies
rosdep update --rosdistro=$ROS_DISTRO
rosdep install --from-paths src --ignore-src -y --rosdistro=$ROS_DISTRO
