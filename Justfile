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
    {{ros_setup}} && colcon build --symlink-install --event-handlers console_direct+
    # {{ros_setup}} && colcon build --merge-install --symlink-install --event-handlers console_direct+

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

# Check current target configuration
check-target:
    @if [ -f .env ]; then export $(grep -v '^#' .env | xargs); fi && \
    echo "Current TARGET: $TARGET" && \
    if [ "$TARGET" = "robot" ]; then \
        echo "Configured for robot (SBC) deployment"; \
        echo "Packages: ROS2 core, hardware interfaces, micro-ROS, Pico SDK"; \
    elif [ "$TARGET" = "remote_pc" ]; then \
        echo "Configured for remote PC development"; \
        echo "Packages: ROS2 core, Gazebo, MoveIt, RQT, simulation tools"; \
    else \
        echo "TARGET not set. Please set TARGET=robot or TARGET=remote_pc in .env"; \
    fi

# Build firmware for Raspberry Pi Pico
build-firmware:
    cd firmware && make build

# Build firmware in release mode
build-firmware-release:
    cd firmware && make build_release

# Flash firmware to Pico (requires BOOTSEL mode or picotool)
flash-firmware:
    cd firmware && make flash

# Flash release firmware to Pico
flash-firmware-release:
    cd firmware && make flash-release

# Monitor firmware debug output and micro-ROS connection
monitor-firmware:
    ./firmware/monitor_firmware.sh

# Test firmware and micro-ROS connection
test-firmware:
    ./scripts/test_firmware_connection.sh

# Start micro-ROS agent with auto-detection
start-microros:
    {{ros_setup}} && python3 scripts/launch_microros_agent.py

# Start micro-ROS agent with specific device
start-microros-dev device="/dev/ttyACM0":
    {{ros_setup}} && ros2 run micro_ros_agent micro_ros_agent serial --dev {{device}} -b 115200 -v
