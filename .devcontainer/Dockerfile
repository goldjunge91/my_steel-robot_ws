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
# Install essential ROS2 packages at build time to avoid reinstalling after each container rebuild
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
        && apt-get -y install --no-install-recommends \
        # Core simulation and control packages
        ros-humble-gazebo-ros-pkgs \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-gazebo-ros2-control \
        ros-humble-xacro \
        ros-humble-joint-state-publisher-gui \
        ros-humble-urdf \
        ros-humble-urdf-launch \
        ros-humble-urdf-tutorial \
        ros-humble-twist-mux \
        ros-humble-rqt-image-view \
        ros-humble-rosbridge-suite \
        ros-humble-rqt \
        # Useful tools and extensions
        python3-colcon-common-extensions \
        joystick jstest-gtk evtest \
        software-properties-common \
        curl \
        #
        # Clean up
        && apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog


# Set up auto-source of workspace for ros user
ARG WORKSPACE
ENV ROS_DISTRO=humble
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
