# robot_autonomy Directory Analysis

## Function

The `robot_autonomy` directory serves as an orchestration layer for deploying and managing the navigation stack (Nav2) and Simultaneous Localization and Mapping (SLAM) functionalities for the `my_steel` robot. It leverages Docker for containerization and `just` (a modern alternative to `make`) for defining common development and deployment recipes.

## Purpose

The primary purpose of this directory is to streamline the setup, deployment, and operation of the robot's autonomous capabilities. This includes:

*   **Simplified Deployment:** Using Docker to containerize Nav2 and SLAM components, ensuring consistent environments and easier deployment across different machines (local, remote, Raspberry Pi).
*   **Automated Workflows:** Providing `justfile` recipes to automate common tasks like starting the navigation stack, running simulations, and managing Docker containers.
*   **Modular Configuration:** Organizing Nav2 launch and parameter sets in a dedicated `nav2/` directory, allowing for clear separation of concerns and easier customization for different robot setups (simulation vs. real hardware).
*   **Integration with Robot:** Ensuring that the navigation stack correctly consumes sensor data (`/odom`, `/scan`, `/tf`) published by the robot's bringup system.

## Usage

The `robot_autonomy` directory is used to manage and launch the robot's autonomous navigation and mapping features.

1.  **Deployment with Docker:**
    *   The `docker/` directory contains container definitions for Nav2, `slam_toolbox`, and `foxglove` (a visualization tool).
    *   Users would typically use `just` commands (e.g., `just start-nav`) to build and run these Docker containers, bringing up the entire navigation stack.
2.  **Local Testing in Simulation:**
    *   The `README.md` suggests testing locally in simulation using `ros2 launch robot_sim simulation.launch.py`. This implies there might be a `robot_sim` package elsewhere that provides simulation launch files, or this command is a placeholder for a more general simulation launch.
3.  **Nav2 Configuration:**
    *   The `nav2/` directory contains launch files and parameter sets specifically for configuring the Nav2 stack. These would define navigation behaviors, planner settings, controller parameters, and localization methods.
4.  **`justfile` / `Makefile`:**
    *   The `justfile` (or `Makefile`) provides a collection of commands (`just start-nav`) to orchestrate the Docker containers and launch ROS 2 nodes, simplifying complex multi-component setups.
5.  **Prerequisites:** It emphasizes the importance of `/odom`, `/scan`, and `/tf` topics being published by the robot's bringup system before starting the navigation planners. It also mentions using conserved parameter sets for simulation vs. real robot, likely managed within the `nav2/` configurations.

## Key Components and Files:

*   **`README.md`:** Provides an overview, content structure, quickstart guide, and important notes regarding topic publishing and parameter sets.
*   **`docker/` (directory):** Contains Dockerfile(s) and potentially `docker-compose.yaml` files for containerizing Nav2, `slam_toolbox`, and `foxglove`.
*   **`nav2/` (directory):** Contains ROS 2 launch files and YAML parameter files for configuring the Nav2 stack (e.g., `amcl`, `map_server`, `local_costmap`, `global_costmap`, planners, controllers).
*   **`justfile` / `Makefile`:** Defines custom commands for building, running, and managing the autonomous stack, often interacting with Docker.
