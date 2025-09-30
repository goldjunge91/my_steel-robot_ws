# robot_controller Package Analysis

## Function

The `robot_controller` package provides a ROS 2 node that acts as an interface between a joystick (or other teleoperation input) and the robot's differential drive controller. It translates joystick commands into `geometry_msgs/msg/Twist` messages, which are then published to control the robot's linear and angular velocities.

## Purpose

The primary purpose of this package is to enable intuitive teleoperation of a differential drive robot using a joystick. It simplifies the control process by:

*   **Mapping Joystick Input:** Converting raw joystick button and axis data into meaningful robot movement commands.
*   **Providing a Standard Interface:** Publishing `Twist` messages, which are a standard way to command mobile robots in ROS 2, making it compatible with various differential drive controllers.
*   **Configurability:** Allowing users to customize joystick button and axis mappings, as well as maximum linear and angular velocities, through ROS parameters.

## Usage

The `robot_controller` package is typically used by launching its node, which then subscribes to joystick input and publishes velocity commands.

1.  **Dependencies:** It depends on `rclcpp` for ROS 2 communication, `sensor_msgs` for joystick input (`sensor_msgs/msg/Joy`), and `geometry_msgs` for publishing velocity commands (`geometry_msgs/msg/Twist`).
2.  **Executable:** The package builds an executable named `robot_controller_node`. This node is responsible for:
    *   Subscribing to the `/joy` topic (or a remapped topic) to receive `sensor_msgs/msg/Joy` messages.
    *   Processing the joystick data based on configured button and axis mappings.
    *   Calculating the desired linear (x) and angular (z) velocities for the robot.
    *   Publishing these velocities as `geometry_msgs/msg/Twist` messages to the `/cmd_vel` topic (or a remapped topic, typically `/diff_cont/cmd_vel_unstamped` for a differential drive controller).
3.  **Configuration:** The `config/` directory contains `robot_controller.yaml`, which defines ROS parameters for:
    *   `axis_linear_x`: Joystick axis for linear X velocity.
    *   `axis_angular_z`: Joystick axis for angular Z velocity.
    *   `button_turbo`: Button to enable turbo mode (increased max velocities).
    *   `max_linear_velocity`: Maximum linear velocity.
    *   `max_angular_velocity`: Maximum angular velocity.
    *   `turbo_linear_multiplier`: Multiplier for linear velocity in turbo mode.
    *   `turbo_angular_multiplier`: Multiplier for angular velocity in turbo mode.
4.  **Launch Files:** A typical usage would involve a launch file that:
    *   Starts the `joy_node` (from the `joy` package) to publish joystick input.
    *   Launches the `robot_controller_node` from this package, loading its parameters from `robot_controller.yaml`.
    *   Remaps the `/cmd_vel` output topic to the appropriate input topic of the robot's differential drive controller.

## Key Components and Files:

*   **`package.xml`:** Defines package metadata, build tool (`ament_cmake`), and dependencies (`rclcpp`, `sensor_msgs`, `geometry_msgs`).
*   **`CMakeLists.txt`:** Configures the build process, finds necessary ROS 2 packages, defines and builds the `robot_controller_node` executable from `src/robot_controller_node.cpp`, and installs the `config` and `launch` directories.
*   **`src/robot_controller_node.cpp`:** The main C++ source file containing the ROS 2 node logic for subscribing to joystick input, processing it, and publishing `Twist` commands.
*   **`config/robot_controller.yaml`:** Configuration file for joystick mappings and velocity parameters.
*   **`launch/robot_controller.launch.py`:** Example Python launch file for starting the `robot_controller_node` with its configuration.
*   **`README.md`:** Provides an overview, usage instructions, and configuration details.
