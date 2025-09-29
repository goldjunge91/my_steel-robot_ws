# ROS 2 Steel Robot Workspace

This workspace contains the ROS 2 packages for a mobile robot.

## Architecture

The main packages for running the robot are:

-   `robot_description`: Contains the URDF and mesh files for the robot.
-   `robot_controller`: Contains the launch files and controller configurations for the robot.
-   `rosbot_hardware_interfaces`: Contains the `ros2_control` hardware interface for the robot.

The intended architecture is to use the `robot_description` package to load the robot's URDF and the `robot_controller` package to launch the controllers. The `rosbot_hardware_interfaces` package provides the hardware interface for the `ros2_control` framework.

### Obsolete Packages

The following packages are obsolete and should not be used:

-   `robot`: This package is a monolithic predecessor to the `robot_description` and `robot_controller` packages. Its launch files and configs are outdated and conflict with the modern setup.
-   `mecabridge_hardware`: This package contains a parallel, unused hardware interface implementation.
-   `robot_autonomy`, `robot_gazebo`, `robot_hardware`, `robot_localization`, `robot_nerf_launcher`, `robot_vision`: These packages appear to be empty scaffolds and can be removed.

## How to Run the Robot

### Simulation

To run the robot in simulation, you can use the `launch_sim.launch.py` file from the `robot` package. However, this launch file is outdated and uses the obsolete `robot` package. It is recommended to create a new launch file in the `robot_controller` package for running the simulation.

A new launch file should be created in `robot_controller/launch` that uses the URDF from `robot_description` and the controller configurations from `robot_controller`.

### Hardware

To run the robot on hardware, you can use the `controller.launch.py` file from the `robot_controller` package. This launch file loads the correct URDF from `robot_description` and the `rosbot_hardware_interfaces` hardware interface.

By default, the `controller.launch.py` launch file uses the `diff_drive_controller`. To use the `mecanum_drive_controller`, you can pass the `controller` argument to the launch file:

```bash
ros2 launch robot_controller controller.launch.py controller:=mecanum_drive_controller
```

## Inconsistencies and Recommendations

### Controller Inconsistency

The main simulation launch file (`robot/launch/launch_sim.launch.py`) is configured to use `mecanum_drive_controller`, while the main hardware launch file (`robot_controller/launch/controller.launch.py`) defaults to `diff_drive_controller` for the "robot" model. This needs to be unified for consistent behavior between sim and reality.

It is recommended to create a new simulation launch file in the `robot_controller` package that uses the same controller configurations as the hardware launch file.

### Obsolete Packages

As mentioned above, several packages are obsolete and should be removed to avoid confusion and conflicts.

It is recommended to remove the following packages:

-   `robot`
-   `mecabridge_hardware`
-   `robot_autonomy`
-   `robot_gazebo`
-   `robot_hardware`
-   `robot_localization`
-   `robot_nerf_launcher`
-   `robot_vision`

By cleaning up the workspace and unifying the launch files, you will have a more consistent and maintainable ROS 2 project.
