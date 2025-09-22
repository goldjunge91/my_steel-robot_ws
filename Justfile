set shell := ["bash", "-lc"]

ros_setup := 'source /opt/ros/humble/setup.bash && if [ -f install/setup.bash ]; then source install/setup.bash; fi'

# Show available recipes
default:
    @just --list

# Bootstrap dev container dependencies
setup-dev:
    ./setup.sh

# Build the entire workspace with merge-install layout
build:
    {{ros_setup}} && colcon build --merge-install --symlink-install --event-handlers console_direct+

# Build only hardware + robot packages and their dependencies
build-hardware:
    {{ros_setup}} && colcon build --packages-up-to robot_hardware robot --merge-install --symlink-install --event-handlers console_direct+

# Run workspace tests for core robot + hardware packages
run-tests:
    {{ros_setup}} && colcon test --packages-select robot_hardware robot

# Remove build artifacts (clean rebuild helper)
clean:
    rm -rf build install log

# Launch Gazebo simulation with mecanum controller
start-gazebo-sim:
    {{ros_setup}} && ros2 launch robot launch_sim.launch.py use_sim_time:=true

# Launch tmux-based simulation helper (multi pane workflow)
start-sim-tmux:
    ./start_sim_tmux.sh

# Run keyboard teleoperation against mecanum controller topic
run-teleop:
    {{ros_setup}} && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/mecanum_cont/cmd_vel_unstamped

# Open a shell with ROS and workspace overlays sourced
shell:
    {{ros_setup}} && exec bash
