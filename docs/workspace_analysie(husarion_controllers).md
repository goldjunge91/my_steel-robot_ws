The `src/husarion_controllers` package contains robotic controllers designed to work with `ros2_control` specifically for Husarion robots. Currently, it primarily houses the `mecanum_drive_controller`.

### Package: `husarion_controllers`

*   **Function:** Provides specialized `ros2_control` compatible controllers for Husarion robots.
*   **Purpose:** To enable precise and robust control of mobile robot platforms, particularly those with mecanum drive systems, by integrating with the ROS 2 control framework.
*   **Usage:** This package is intended to be built within a ROS 2 workspace and its controllers are then loaded and managed by the `controller_manager` node.

#### Sub-package: `mecanum_drive_controller`

*   **Function:** This is a `ros2_control` controller for mobile robots equipped with a mecanum drive system. It translates high-level velocity commands (linear x, linear y, and angular z) into individual wheel velocity commands for a four-wheeled mecanum base. It also computes and publishes odometry based on wheel feedback.
*   **Purpose:** To provide a standard and real-time safe way to control mecanum-wheeled robots within the ROS 2 ecosystem, handling the complex kinematics of mecanum wheels and providing essential odometry information.
*   **Usage:**
    1.  **Build:** It is built using `ament_cmake` and depends on standard ROS 2 packages like `controller_interface`, `geometry_msgs`, `nav_msgs`, `rclcpp`, `rclcpp_lifecycle`, `realtime_tools`, and `tf2`.
    2.  **Configuration:** The controller's behavior is configured via a YAML file (e.g., `mecanum_drive_controller_parameter.yaml`). Key parameters include:
        *   `front_left_wheel_name`, `front_right_wheel_name`, `rear_left_wheel_name`, `rear_right_wheel_name`: Names of the wheel joints as defined in the robot's URDF.
        *   `wheel_separation_x`, `wheel_separation_y`, `wheel_radius`: Physical dimensions of the robot's wheel layout.
        *   `wheel_separation_x_multiplier`, `wheel_separation_y_multiplier`, `wheel_radius_multiplier`: Calibration factors for fine-tuning kinematics.
        *   `odom_frame_id`, `base_frame_id`: Frame IDs for odometry publishing.
        *   `open_loop`: Boolean to enable/disable open-loop odometry calculation.
        *   `position_feedback`: Boolean indicating if position feedback is available from hardware.
        *   `enable_odom_tf`: Boolean to enable/disable publishing of the `odom` to `base_link` TF transform.
        *   `cmd_vel_timeout`: Timeout for incoming velocity commands, after which the robot will halt.
        *   `publish_rate`: Rate at which odometry and TF messages are published.
        *   `linear.x`, `linear.y`, `angular.z`: Parameters for velocity, acceleration, and jerk limits for each translational and rotational degree of freedom.
    3.  **Operation:**
        *   It subscribes to velocity commands (typically `geometry_msgs/msg/Twist` or `geometry_msgs/msg/TwistStamped`) on topics like `~/cmd_vel` or `~/cmd_vel_unstamped`.
        *   It publishes odometry messages (`nav_msgs/msg/Odometry`) on `~/odom` and optionally TF transforms (`tf2_msgs/msg/TFMessage`) on `/tf`.
        *   It provides a real-time safe implementation, odometry publishing, and supports task-space velocity, acceleration, and jerk limits, as well as automatic stopping after command timeouts.
        *   The controller works with wheel joints through a velocity interface, meaning it expects to receive and send velocity commands to the hardware interfaces for each wheel.

In essence, `husarion_controllers` provides the `mecanum_drive_controller` as a robust and configurable solution for controlling mecanum-wheeled robots within the ROS 2 framework, handling kinematics, odometry, and safety features like command timeouts and speed limiting.