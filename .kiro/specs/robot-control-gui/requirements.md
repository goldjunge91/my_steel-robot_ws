# Requirements Document
## Path
'/home/marco/ros2_steel_ws/my_steel-robot_ws/robot_control_gui'
## Introduction

This document defines the requirements for a graphical user interface (GUI) application that simplifies the launching and management of various ROS2 robot operations. The GUI will provide an intuitive interface to replace manual script execution for common robot tasks such as teleoperation, robot startup, simulation, and micro-ROS agent management. The application targets both local development (remote PC) and remote robot control scenarios.

## Requirements

### Requirement 0: CODING
NEVER MAKE CODE FILES WITH MORE THEN 300 LINES OF CODE

### Requirement 1: Teleoperation Control

**User Story:** As a robot operator, I want to start and stop teleoperation from the GUI, so that I can control the robot without manually running shell scripts.

#### Acceptance Criteria

1. WHEN the user clicks "Start Teleoperation" THEN the system SHALL launch the joy node and teleop_twist_joy node
2. WHEN teleoperation is starting THEN the system SHALL check for connected joystick devices and display an error if none are found
3. WHEN teleoperation is running THEN the system SHALL display the process status (running/stopped) and PID information
4. WHEN the user clicks "Stop Teleoperation" THEN the system SHALL terminate all teleoperation processes gracefully
5. WHEN teleoperation is active THEN the system SHALL display controller instructions (button mappings) in the GUI
6. IF the xbox_teleop.yaml config file exists THEN the system SHALL use it when launching teleop nodes

### Requirement 2: Robot System Management

**User Story:** As a robot operator, I want to start and stop the robot system remotely from my PC, so that I can manage the robot without SSH access to the Raspberry Pi.

#### Acceptance Criteria

1. WHEN the user clicks "Start Robot" THEN the system SHALL execute the robot startup sequence (micro-ROS agent, robot bringup, Foxglove bridge)
2. WHEN starting the robot remotely THEN the system SHALL connect to the robot via SSH using configured credentials
3. WHEN the robot is starting THEN the system SHALL display real-time status updates for each component (micro-ROS agent, bringup, Foxglove)
4. WHEN the robot system is running THEN the system SHALL display the Foxglove WebSocket URL for connection
5. WHEN the user clicks "Stop Robot" THEN the system SHALL terminate all robot processes on the remote system
6. IF the robot connection fails THEN the system SHALL display a clear error message with troubleshooting suggestions
7. WHEN robot processes are running THEN the system SHALL display log output in a scrollable text area

### Requirement 3: Simulation Management

**User Story:** As a developer, I want to start and stop Gazebo simulation from the GUI, so that I can test robot behavior without hardware.

#### Acceptance Criteria

1. WHEN the user clicks "Start Simulation" THEN the system SHALL launch Gazebo with the robot model and configured world
2. WHEN simulation is starting THEN the system SHALL display loading progress and status
3. WHEN simulation is running THEN the system SHALL display the simulation status and allow control commands
4. WHEN the user clicks "Stop Simulation" THEN the system SHALL terminate Gazebo and all related ROS2 nodes
5. WHEN simulation is active THEN the system SHALL provide options to launch RViz or other visualization tools
6. IF the user selects tmux mode THEN the system SHALL launch simulation in separate tmux windows with control and monitoring terminals

### Requirement 4: Micro-ROS Agent Management

**User Story:** As a robot operator, I want to start and stop the micro-ROS agent from the GUI, so that I can manage firmware communication easily.

#### Acceptance Criteria

1. WHEN the user clicks "Start micro-ROS Agent" THEN the system SHALL auto-detect the Pico device and launch the agent
2. WHEN the device is not auto-detected THEN the system SHALL allow manual device selection from a dropdown
3. WHEN the agent is starting THEN the system SHALL display connection status and device information
4. WHEN the agent is running THEN the system SHALL show real-time message traffic statistics
5. WHEN the user clicks "Stop Agent" THEN the system SHALL terminate the micro-ROS agent process
6. IF the device connection fails THEN the system SHALL display diagnostic information and retry options

### Requirement 5: Process Monitoring and Logging

**User Story:** As a robot operator, I want to see real-time logs and process status, so that I can diagnose issues quickly.

#### Acceptance Criteria

1. WHEN any process is running THEN the system SHALL display its stdout/stderr output in a dedicated log viewer
2. WHEN logs are displayed THEN the system SHALL support auto-scrolling and manual scrolling
3. WHEN the user selects a process THEN the system SHALL show detailed information (PID, uptime, resource usage)
4. WHEN processes generate errors THEN the system SHALL highlight error messages in red
5. WHEN the user clicks "Clear Logs" THEN the system SHALL clear the log viewer for the selected process
6. WHEN the user clicks "Save Logs" THEN the system SHALL save logs to a timestamped file

### Requirement 6: Configuration Management

**User Story:** As a robot operator, I want to configure connection settings and preferences, so that I can adapt the GUI to my setup.

#### Acceptance Criteria

1. WHEN the user opens settings THEN the system SHALL display configuration options for robot hostname, SSH credentials, and ROS_DOMAIN_ID
2. WHEN the user modifies settings THEN the system SHALL validate inputs before saving
3. WHEN settings are saved THEN the system SHALL persist them to a configuration file
4. WHEN the application starts THEN the system SHALL load saved settings automatically
5. IF no configuration exists THEN the system SHALL use sensible defaults and prompt for required settings
6. WHEN the user clicks "Test Connection" THEN the system SHALL verify SSH connectivity to the robot

### Requirement 7: Quick Actions and Status Dashboard

**User Story:** As a robot operator, I want to see the overall system status at a glance, so that I can quickly understand what is running.

#### Acceptance Criteria

1. WHEN the application starts THEN the system SHALL display a dashboard showing status of all major components
2. WHEN any component status changes THEN the system SHALL update the dashboard in real-time
3. WHEN the user hovers over a status indicator THEN the system SHALL show detailed tooltip information
4. WHEN the dashboard is displayed THEN the system SHALL show indicators for: teleoperation, robot system, simulation, micro-ROS agent
5. WHEN a component is running THEN the system SHALL display a green indicator; when stopped, a gray indicator; when error, a red indicator
6. WHEN the user clicks a status indicator THEN the system SHALL navigate to the detailed view for that component

### Requirement 8: Firmware Management

**User Story:** As a developer, I want to build and flash firmware from the GUI, so that I can update the Pico without using command-line tools.

#### Acceptance Criteria

1. WHEN the user clicks "Build Firmware" THEN the system SHALL execute the firmware build process and display progress
2. WHEN the user selects "Debug" or "Release" mode THEN the system SHALL build the appropriate firmware variant
3. WHEN firmware build completes THEN the system SHALL display build status and any errors
4. WHEN the user clicks "Flash Firmware" THEN the system SHALL detect the Pico in BOOTSEL mode and flash the firmware
5. IF the Pico is not in BOOTSEL mode THEN the system SHALL display instructions for entering BOOTSEL mode
6. WHEN flashing completes THEN the system SHALL display success confirmation and suggest monitoring firmware output

### Requirement 9: ROS2 Topic and Node Monitoring

**User Story:** As a developer, I want to view active ROS2 topics and nodes, so that I can verify system connectivity and data flow.

#### Acceptance Criteria

1. WHEN the user opens the monitoring view THEN the system SHALL list all active ROS2 nodes
2. WHEN the user opens the monitoring view THEN the system SHALL list all active ROS2 topics with message types
3. WHEN the user selects a topic THEN the system SHALL display real-time message data in a formatted view
4. WHEN the user clicks "Refresh" THEN the system SHALL update the node and topic lists
5. WHEN the user selects a node THEN the system SHALL display node information (publishers, subscribers, services)
6. WHEN ROS2 is not running THEN the system SHALL display a message indicating no ROS2 environment detected

### Requirement 10: Multi-Platform Support

**User Story:** As a developer, I want the GUI to work on Linux, so that I can use it on my development machine and robot SBC.

#### Acceptance Criteria

1. WHEN the application is launched on Linux THEN the system SHALL run without platform-specific errors
2. WHEN the application uses system commands THEN the system SHALL use Linux-compatible commands (bash, SSH, etc.)
3. WHEN the application accesses file paths THEN the system SHALL use POSIX-style paths
4. WHEN the application is packaged THEN the system SHALL provide installation instructions for Ubuntu 22.04
5. IF the application requires dependencies THEN the system SHALL document all required packages and installation steps
