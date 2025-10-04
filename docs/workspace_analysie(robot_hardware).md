# robot_hardware Directory Analysis

## Function

The `robot_hardware` directory implements the host-side (SBC - Single Board Computer) `ros2_control` `SystemInterface` or a bridge node. Its core function is to establish a communication link between the ROS 2 `controller_manager` and the low-level firmware running on a microcontroller (Pico). Specifically, it handles:

*   **`read()` operation:** Reads encoder data from the Pico and translates it into joint states for ROS 2.
*   **`write()` operation:** Sends wheel commands (velocities) from ROS 2 controllers to the Pico firmware.

## Purpose

The primary purpose of this package is to provide the necessary hardware abstraction layer for the robot. It enables ROS 2 controllers to command the robot's physical actuators and receive feedback from its sensors without needing to directly interact with the low-level microcontroller. This serves to:

*   **Decouple Hardware from Control:** Allows for flexible changes in either the hardware or the high-level control logic without affecting the other.
*   **Standardize Interface:** Provides a standard `ros2_control` interface, making the robot compatible with existing ROS 2 controllers and tools.
*   **Facilitate Development:** Simplifies the development of robot applications by abstracting away the complexities of low-level communication protocols and hardware specifics.

## Usage

The `robot_hardware` package is used as a plugin within the `ros2_control` framework.

1.  **Configuration:**
    *   It requires configuration parameters, typically defined in a YAML file like `config/ros2_control_params.yaml`. These parameters include `serial_port`, `baudrate`, `drive_type` (e.g., `mecanum`, `diff`), and a list of `joint names` for the wheels.
    *   This configuration is passed to the `ros2_control_node` via the robot's URDF.
2.  **Integration with `ros2_control`:**
    *   The package exports its `SystemInterface` implementation via `pluginlib` (as indicated by `multi_drive_hardware_plugins.xml`). This allows the `controller_manager` to dynamically load and use this hardware interface.
3.  **Development and Testing:**
    *   The `README.md` suggests developing with "Fake-Hardware" initially, where the `read()` function provides deterministic encoder values for testing.
    *   It mentions implementing serial communication using libraries like `boost::asio` or `serialib`.
    *   Unit tests would involve simulating serial input to verify state updates.

## Key Components and Files (as described in README.md):

*   **`src/` (directory):** Contains the C++ source code for the `hardware_interface::SystemInterface` implementation or a Python bridge node.
*   **`include/` (directory):** Contains header files for the C++ implementation.
*   **`config/ros2_control_params.yaml`:** An example YAML file for configuring the hardware interface parameters.
*   **`multi_drive_hardware_plugins.xml`:** An XML file used by `pluginlib` to export the `SystemInterface` plugin, making it discoverable by `ros2_control`.

**Note:** At the time of analysis, the `src/robot_hardware` directory does not contain `package.xml` or `CMakeLists.txt` directly in its root, suggesting it might be a sub-package or its contents are managed differently within the overall project structure. The analysis is based on the provided `README.md`.
