# Aliases for my_steel ROS 2 workspace
# Usage: source this file in your shell, e.g.:
#   source docs/aliases.zsh

# Build helpers
alias cb='echo "Building workspace..." && colcon build'
alias cbs='echo "Building workspace with symlinks..." && colcon build --symlink-install'

# Source (simulation/hardware)
alias sbs='echo "Sourcing setup.zsh... (simulation)" && export ros_model=robot_xl && export ROBOT_ROS_BUILD_TYPE=simulation && source install/setup.zsh'
alias sbr='echo "Sourcing setup.zsh... (hardware)" && export ros_model=robot_xl && export ROBOT_ROS_BUILD_TYPE=hardware && source install/setup.zsh'

# Gazebo simulation
alias sim='echo "Launching simulation..." && ROBOT_MODEL_NAME=robot_xl ros2 launch robot_gazebo simulation.launch.py'
alias sim_man='ROBOT_MODEL_NAME=robot_xl ros2 launch robot_gazebo simulation.launch.py configuration:=manipulation'

# Controller bringup (uses our new bringup)
alias bringup='echo "Launching bringup..." && ros2 launch robot_bringup bringup.launch.py robot_model:=robot_xl'
alias bringup_nerf='ros2 launch robot_bringup bringup.launch.py robot_model:=robot_xl mecanum:=True use_sim:=True include_nerf_launcher:=True'

# ros2_control in SIM + spawn robot in Gazebo
alias ctrl_sim_nerf='ros2 launch robot_controller controller.launch.py robot_model:=robot_xl mecanum:=True use_sim:=True include_nerf_launcher:=True'
alias spawn_robot='ros2 run ros_gz_sim create -name robot -topic robot_description'

# URDF quick-load with mock joints (incl. Nerf)
alias urdf_sim_nerf='ros2 launch robot_description load_urdf.launch.py robot_model:=robot_xl use_sim:=True include_nerf_launcher:=True mock_joints:=True'

# Nerf launcher node + quick test commands
alias nerf_node='ros2 launch robot_nerf_launcher nerf_launcher.launch.py'
alias nerf_fire='ros2 topic pub -1 /nerf_launcher/cmd/fire std_msgs/Bool "{data: true}"'
alias nerf_aim='ros2 topic pub -1 /nerf_launcher/cmd/aim geometry_msgs/Vector3 "{x: 0.0, y: 0.0, z: 0.0}"'
alias nerf_fly100='ros2 topic pub -1 /nerf_launcher/cmd/flywheel std_msgs/Float32 "{data: 1.0}"'

# RViz and teleop
alias rviz_launch='echo "Launching RViz..." && ros2 launch robot_description rviz.launch.py'
alias teleop='echo "Starting teleop..." && ros2 run teleop_twist_keyboard teleop_twist_keyboard'

