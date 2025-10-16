# Requirements Document

## Introduction

This specification defines the requirements for creating a production-ready Docker image and container orchestration system for deploying the my_steel robot software stack on a Raspberry Pi 4B. The solution addresses current deployment challenges including missing dependencies, environment configuration issues, and service management complexity. The Docker image will encapsulate the complete ROS2 Humble workspace with all necessary dependencies, ensuring consistent and reliable deployment on the Raspberry Pi hardware platform.

## Glossary

- **Docker_Image**: A lightweight, standalone, executable package that includes everything needed to run the robot software including code, runtime, system tools, libraries, and settings
- **Container**: A running instance of a Docker_Image
- **ROS2_Workspace**: The colcon workspace containing all ROS2 packages for the robot
- **Raspberry_Pi**: The Raspberry Pi 4B single-board computer serving as the robot's main controller
- **micro_ROS_Agent**: The bridge service that enables communication between the Raspberry Pi Pico microcontroller and ROS2
- **Hardware_Interface**: The ros2_control plugin that interfaces with the Pico firmware via micro-ROS
- **Bringup_System**: The complete robot launch system including controllers, state publishers, and sensor interfaces
- **Host_System**: The Raspberry Pi's base operating system (Ubuntu 22.04)
- **DDS_Middleware**: Data Distribution Service middleware for ROS2 communication (FastRTPS or CycloneDDS)
- **USB_Device**: Serial devices connected via USB including the Raspberry Pi Pico (/dev/ttyACM0)
- **Network_Mode**: Docker networking configuration determining how containers communicate with the host and external networks
- **Volume_Mount**: A mechanism to share directories between the host system and container
- **Environment_Variables**: Configuration parameters passed to the container at runtime
- **Multi_Stage_Build**: A Docker build technique that uses multiple FROM statements to optimize image size
- **Docker_Compose**: A tool for defining and running multi-container Docker applications using YAML configuration
- **Tailscale**: A VPN service that creates a secure mesh network between devices for remote access and communication
- **Tailscale_Network**: The virtual private network created by Tailscale connecting the Raspberry_Pi and remote development machines
- **Remote_PC**: A development workstation that connects to the robot over the Tailscale_Network for monitoring and control

## Requirements

### Requirement 1

**User Story:** As a robot operator, I want a Docker image that contains all necessary ROS2 dependencies and packages, so that I can deploy the robot software without manual dependency installation.

#### Acceptance Criteria

1. WHEN THE Docker_Image is built, THE Docker_Image SHALL include ROS2 Humble base installation with all required packages
2. WHEN THE Docker_Image is built, THE Docker_Image SHALL include micro_ros_agent package version compatible with the Pico firmware
3. WHEN THE Docker_Image is built, THE Docker_Image SHALL include all custom ROS2 packages from the ROS2_Workspace
4. WHEN THE Docker_Image is built, THE Docker_Image SHALL include all Python dependencies specified in requirements files
5. WHEN THE Docker_Image is built, THE Docker_Image SHALL resolve all rosdep dependencies for the robot packages

### Requirement 2

**User Story:** As a robot operator, I want the Docker container to access USB devices and network interfaces, so that the robot can communicate with the Pico microcontroller and external systems.

#### Acceptance Criteria

1. WHEN THE Container is started, THE Container SHALL have access to USB_Device at /dev/ttyACM0 for micro-ROS communication
2. WHEN THE Container is started, THE Container SHALL use host network mode to enable ROS2 DDS communication
3. WHEN THE Container is started, THE Container SHALL have access to USB camera devices for vision capabilities
4. WHEN THE Container is started, THE Container SHALL preserve device permissions for serial and USB access
5. WHEN THE Container is started, THE Container SHALL support hot-plugging of USB devices without restart

### Requirement 3

**User Story:** As a robot operator, I want proper DDS middleware configuration in the container, so that ROS2 communication is reliable and does not encounter buffer or discovery issues.

#### Acceptance Criteria

1. WHEN THE Container is started, THE Container SHALL set RMW_IMPLEMENTATION to rmw_fastrtps_cpp by default
2. WHEN THE Container is started, THE Container SHALL set ROS_DOMAIN_ID to 0 for network isolation
3. WHEN THE Container is started, THE Container SHALL unset CYCLONEDDS_URI to prevent configuration conflicts
4. WHEN THE Container is started, THE Container SHALL configure FastRTPS with appropriate buffer sizes for robot telemetry
5. WHERE CycloneDDS is required, THE Container SHALL support alternative DDS configuration via environment override

### Requirement 4

**User Story:** As a robot operator, I want the container to automatically start the robot bringup system, so that the robot becomes operational without manual intervention.

#### Acceptance Criteria

1. WHEN THE Container is started, THE Container SHALL source the ROS2 environment before launching services
2. WHEN THE Container is started, THE Container SHALL source the ROS2_Workspace install directory
3. WHEN THE Container is started, THE Container SHALL launch the Bringup_System with robot_model parameter set to robot_xl
4. WHEN THE Container is started, THE Container SHALL launch the Bringup_System with mecanum drive configuration
5. WHEN THE Container is started, THE Container SHALL wait for micro_ROS_Agent connection before starting Hardware_Interface

### Requirement 5

**User Story:** As a robot operator, I want a Docker Compose configuration for easy container management, so that I can start, stop, and configure the robot with simple commands.

#### Acceptance Criteria

1. WHEN THE Docker_Compose file is used, THE Docker_Compose file SHALL define service configuration for the robot container
2. WHEN THE Docker_Compose file is used, THE Docker_Compose file SHALL define service configuration for the micro_ROS_Agent container
3. WHEN THE Docker_Compose file is used, THE Docker_Compose file SHALL configure automatic restart policy for service resilience
4. WHEN THE Docker_Compose file is used, THE Docker_Compose file SHALL expose environment variables for runtime configuration
5. WHEN THE Docker_Compose file is used, THE Docker_Compose file SHALL define volume mounts for persistent configuration and logs

### Requirement 6

**User Story:** As a robot operator, I want the Docker image to be optimized for ARM64 architecture, so that it runs efficiently on the Raspberry Pi with minimal resource overhead.

#### Acceptance Criteria

1. WHEN THE Docker_Image is built, THE Docker_Image SHALL target ARM64 architecture for Raspberry Pi 4B
2. WHEN THE Docker_Image is built, THE Docker_Image SHALL use Multi_Stage_Build to minimize final image size
3. WHEN THE Docker_Image is built, THE Docker_Image SHALL remove build dependencies and cache files after compilation
4. WHEN THE Docker_Image is built, THE Docker_Image SHALL use release build configuration for ROS2 packages
5. WHEN THE Docker_Image is built, THE Docker_Image SHALL be smaller than 2GB in compressed size

### Requirement 7

**User Story:** As a robot operator, I want the container to support both standalone and multi-service deployment modes, so that I can run micro-ROS agent separately or integrated based on my needs.

#### Acceptance Criteria

1. WHEN THE Container is configured for standalone mode, THE Container SHALL run only the Bringup_System without micro_ROS_Agent
2. WHEN THE Container is configured for integrated mode, THE Container SHALL run both micro_ROS_Agent and Bringup_System
3. WHEN THE Container is configured for agent-only mode, THE Container SHALL run only the micro_ROS_Agent service
4. WHERE multiple containers are used, THE Container SHALL coordinate startup order via health checks
5. WHERE multiple containers are used, THE Container SHALL share network namespace for ROS2 communication

### Requirement 8

**User Story:** As a robot operator, I want the container to persist logs and configuration, so that I can debug issues and maintain settings across container restarts.

#### Acceptance Criteria

1. WHEN THE Container writes logs, THE Container SHALL write ROS2 logs to a Volume_Mount on the Host_System
2. WHEN THE Container writes logs, THE Container SHALL write micro-ROS agent logs to a Volume_Mount on the Host_System
3. WHEN THE Container reads configuration, THE Container SHALL read robot configuration from a Volume_Mount on the Host_System
4. WHEN THE Container is restarted, THE Container SHALL preserve log history from previous runs
5. WHEN THE Container is restarted, THE Container SHALL apply updated configuration from mounted volumes

### Requirement 9

**User Story:** As a developer, I want clear build and deployment documentation, so that I can build custom images and deploy to new Raspberry Pi units efficiently.

#### Acceptance Criteria

1. THE documentation SHALL provide step-by-step instructions for building the Docker_Image on ARM64 systems
2. THE documentation SHALL provide step-by-step instructions for building the Docker_Image on x86_64 systems using buildx
3. THE documentation SHALL provide instructions for pushing images to Docker Hub or private registry
4. THE documentation SHALL provide instructions for deploying containers on fresh Raspberry Pi installations
5. THE documentation SHALL provide troubleshooting guidance for common deployment issues

### Requirement 10

**User Story:** As a robot operator, I want the container to support health checks and graceful shutdown, so that the system can recover from failures and shut down safely.

#### Acceptance Criteria

1. WHEN THE Container is running, THE Container SHALL expose a health check endpoint for monitoring
2. WHEN THE Container health check fails, THE Container SHALL restart automatically after 3 consecutive failures
3. WHEN THE Container receives SIGTERM signal, THE Container SHALL gracefully shutdown ROS2 nodes
4. WHEN THE Container receives SIGTERM signal, THE Container SHALL wait up to 30 seconds for clean shutdown
5. WHEN THE Container is unhealthy, THE Container SHALL log diagnostic information for troubleshooting

### Requirement 11

**User Story:** As a robot operator, I want Tailscale VPN integrated into the container, so that the robot and remote PC can communicate securely over the internet without complex network configuration.

#### Acceptance Criteria

1. WHEN THE Container is started, THE Container SHALL initialize Tailscale client if authentication key is provided
2. WHEN THE Container is started with Tailscale enabled, THE Container SHALL connect to the Tailscale_Network automatically
3. WHEN THE Container is connected to Tailscale_Network, THE Container SHALL advertise ROS2 DDS traffic over the VPN interface
4. WHEN THE Remote_PC is connected to Tailscale_Network, THE Remote_PC SHALL discover and communicate with robot ROS2 nodes
5. WHERE Tailscale is not configured, THE Container SHALL operate normally using local network interfaces

### Requirement 12

**User Story:** As a developer, I want Tailscale configuration to be manageable via environment variables and volumes, so that I can deploy to multiple robots without rebuilding the image.

#### Acceptance Criteria

1. WHEN THE Container is configured for Tailscale, THE Container SHALL accept Tailscale authentication key via environment variable
2. WHEN THE Container is configured for Tailscale, THE Container SHALL persist Tailscale state to a Volume_Mount for reconnection
3. WHEN THE Container is configured for Tailscale, THE Container SHALL accept custom Tailscale hostname via environment variable
4. WHEN THE Container is configured for Tailscale, THE Container SHALL support Tailscale subnet routing for multi-robot networks
5. WHEN THE Container is restarted, THE Container SHALL reconnect to Tailscale_Network without re-authentication
