shell>$ ros2 run micro_ros_setup create_firmware_ws.sh zephyr  rpi_pico
Package 'micro_ros_setup' not found
shell>$ source /opt/ros/$ROS_DISTRO/setup.bash
shell>$ ls
shell>$ git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
        <!-- output -->
        Cloning into 'src/micro_ros_setup'...
        remote: Enumerating objects: 4090, done.
        remote: Counting objects: 100% (405/405), done.
        remote: Compressing objects: 100% (190/190), done.
        remote: Total 4090 (delta 351), reused 215 (delta 215), pack-reused 3685 (from 2)
        Receiving objects: 100% (4090/4090), 941.03 KiB | 1.31 MiB/s, done.
        Resolving deltas: 100% (2846/2846), done.
        Updating files: 100% (157/157), done.
        <!-- output end -->
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
reading in sources list data from /etc/ros/rosdep/sources.list.d
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml
Query rosdistro index https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml
Skip end-of-life distro "ardent"
Skip end-of-life distro "bouncy"
Skip end-of-life distro "crystal"
Skip end-of-life distro "dashing"
Skip end-of-life distro "eloquent"
Skip end-of-life distro "foxy"
Skip end-of-life distro "galactic"
Skip end-of-life distro "groovy"
Add distro "humble"
Skip end-of-life distro "hydro"
Skip end-of-life distro "indigo"
Skip end-of-life distro "iron"
Skip end-of-life distro "jade"
Add distro "jazzy"
Add distro "kilted"
Skip end-of-life distro "kinetic"
Skip end-of-life distro "lunar"
Skip end-of-life distro "melodic"
Skip end-of-life distro "noetic"
Add distro "rolling"
updated cache in /home/ros/.ros/rosdep/sources.cache
#All required rosdeps installed successfully
Starting >>> micro_ros_setup
[Processing: micro_ros_setup]                         
[Processing: micro_ros_setup]                                 
Finished <<< micro_ros_setup [1min 19s]                           

Summary: 1 package finished [1min 20s]
ros@docker-desktop:/workspaces/my_steel-robot_ws/uros_ws$ ros2 run micro_ros_setup create_firmware_ws.sh zephyr rpi_pico