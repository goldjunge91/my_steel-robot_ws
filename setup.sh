#!/bin/bash
set -e

vcs import < src/ros2.repos src
sudo apt-get update
sudo apt-get install -y --no-install-recommends libboost1.74-dev || true

rosdep update --rosdistro=$ROS_DISTRO
rosdep install --from-paths src --ignore-src -y --rosdistro=$ROS_DISTRO
