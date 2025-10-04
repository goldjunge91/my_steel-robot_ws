# mecabridge_hardware Package Analysis

## Function

The `mecabridge_hardware` package serves as a ROS 2 `SystemInterface` implementation, bridging high-level ROS 2 control systems with low-level Raspberry Pi Pico firmware for mobile robot platforms. It provides a standardized interface for controlling various robot hardware components and receiving sensor feedback.

## Purpose

The primary purpose of this package is to:

*   **Abstract Hardware:** Offer a consistent ROS 2 interface for robot hardware, decoupling control logic from low-level implementation details.
*   **Support Multiple Drive Configurations:** Accommodate differential, mecanum, and four-wheel independent drive systems, enhancing versatility.
*   **Ensure Safety:** Implement a robust watchdog system and CRC-16/CCITT-FALSE validated serial protocol to guarantee deterministic and safe operation, including automatic failsafes upon communication loss.
*   **Enable Configurability:** Allow comprehensive customization of robot parameters (e.g., wheel geometry, encoder settings, serial communication) through a single YAML configuration file.
*   **Facilitate Extensibility:** Be designed with modular components and compile-time feature toggles to support future hardware integrations and software enhancements.

## Usage

The `mecabridge_hardware` package is integrated and used within a ROS 2 environment through the following steps:

1.  **Building:** The package is built using `colcon build --packages-select mecabridge_hardware` within a ROS 2 workspace.
2.  **Configuration:**
    *   **Hardware Parameters:** A YAML file (e.g., `mecabridge_hardware_params.yaml`) defines robot-specific parameters such as `drive_type`, `has_encoders`, `device` (serial port), `baud_rate`, `timeout`, `max_lin_vel`, `max_ang_vel`, wheel names, `enc_counts_per_rev`, `wheel_radius`, `wheel_base`, `track_width`, and `mix_factor`.
    *   **URDF/Xacro Integration:** The `mecabridge_hardware/MecaBridgeHardware` plugin is declared within the `<ros2_control>` tag of the robot's URDF/Xacro description, referencing the hardware configuration YAML.
    *   **Controller Configuration:** Separate YAML files (e.g., `mecabridge_differential_controller.yaml`) configure standard `ros2_controllers` (like `diff_drive_controller`, `mecanum_drive_controller`, `forward_command_controller`, `joint_state_broadcaster`) and map them to the interfaces exposed by `mecabridge_hardware`.
3.  **Launching:** Python launch files (e.g., `mecabridge_differential.launch.py`, `mecabridge_mecanum.launch.py`, `mecabridge_four_wheel.launch.py`) are used to start the entire robot system. These typically:
    *   Load the robot description.
    *   Start the `ros2_control_node`.
    *   Spawn the necessary controllers (e.g., `joint_state_broadcaster`, drive controllers).
    *   Pass configuration parameters to the nodes.
4.  **Operation:**
    *   High-level commands (e.g., `geometry_msgs/Twist` messages on `/cmd_vel`) are published to the appropriate ROS 2 topics, which are then processed by the spawned controllers.
    *   The `MecaBridgeHardware` interface translates these commands into a custom binary protocol and transmits them over a serial link to the Raspberry Pi Pico firmware.
    *   The Pico firmware interprets the commands, controls motors, servos, and ESCs, and reads sensor data from encoders.
    *   The Pico sends back state frames containing encoder counts, servo positions, ESC states, and various status flags.
    *   The `MecaBridgeHardware` interface parses these incoming state frames and updates the `ros2_control` state interfaces, which are subsequently published as `sensor_msgs/JointState` messages.
5.  **Safety Features:** A watchdog system, active on both the host (Raspberry Pi 4B) and the Pico firmware, monitors communication. If no valid command or heartbeat frame is received within a configurable `SAFE_CUTOFF_MS` (e.g., 150ms), all actuators are commanded to a safe, neutral (zero) state.
6.  **Testing:** The package includes comprehensive unit and integration tests. These tests cover protocol validation (CRC, frame parsing), configuration parsing, and the overall functionality of the hardware interface. Mock serial backends are utilized to enable testing without physical hardware.

## Key Components and Files:

*   **`MecaBridgeHardware` (`include/mecabridge_hardware/mecabridge_hardware.hpp`, `src/mecabridge_hardware/mecabridge_hardware.cpp`):** The core `ros2_control::SystemInterface` implementation. It manages the hardware lifecycle, exports state and command interfaces, and orchestrates `read()` and `write()` operations with the serial backend.
*   **`MecaBridgeSerialProtocol` (`include/mecabridge_hardware/mecabridge_serial_protocol.h`, `src/mecabridge_hardware/mecabridge_serial_protocol.cpp`):** Manages all serial communication with the Raspberry Pi Pico. This includes auto-detection, connection/reconnection logic, and the low-level transmission/reception of protocol messages.
*   **`MecaBridgeDriveConfig` (`include/mecabridge_hardware/mecabridge_drive_config.h`):** A C++ struct defining the configurable parameters for the hardware interface, such as drive type, encoder presence, and physical robot dimensions.
*   **`mecabridge_utils` (directory `src/mecabridge_utils`):** A collection of utility libraries for:
    *   **`protocol`:** Handles the custom binary communication protocol, including CRC-16/CCITT-FALSE calculation, frame encoding, and parsing.
    *   **`config`:** Provides functionality for parsing and validating the YAML configuration files.
    *   **`safety`:** Implements the watchdog logic.
    *   **`latency`:** Tracks communication latency.
    *   **`serial`:** Defines generic serial backend interfaces and provides mock/loopback implementations for testing.
*   **`firmware/mecabridge_pico` (directory):** Contains the Raspberry Pi Pico firmware source code:
    *   **`main.cpp`:** The main application loop on the Pico, responsible for processing serial data, managing the watchdog, controlling actuators, and sending state feedback.
    *   **`actuators.cpp/.hpp`:** Implements the low-level control for motors, servos, and ESCs, including PWM generation and command clamping.
    *   **`frame_parser.cpp/.hpp`:** Handles the binary protocol on the Pico side, including CRC validation and frame serialization/deserialization.
    *   **`enc.cpp/.hpp`:** Manages quadrature encoder input.
    *   **`esc_servo.cpp/.hpp`:** Provides functions for controlling ESCs and servos via PWM.
*   **`config` (directory `src/mecabridge_hardware/config`):** Stores example and default YAML configuration files for various drive types and hardware settings.
*   **`launch` (directory `src/mecabridge_hardware/launch`):** Contains Python launch files for orchestrating the startup of the `mecabridge_hardware` interface and associated controllers.
*   **`test` (directory `src/mecabridge_hardware/test`):** Houses GTest-based unit and integration tests for the host-side ROS 2 package, ensuring the correctness and robustness of the implementation.
