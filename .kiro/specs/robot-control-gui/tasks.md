# Implementation Plan

- [x] 1. Set up project structure and core infrastructure
  - Create a git repo with gh cli (`gh repo create`) and clone it (`git clone`) name it robot_control_gui
  - Create directory structure: `robot_control_gui/` with subdirectories for `ui/`, `core/`, `utils/`
  - Create `setup.py` for package installation
  - Create `__init__.py` files for Python package structure
  - Create `requirements.txt` with dependencies: PyQt6, paramiko (rclpy not needed - using subprocess for ROS2 commands)
  - Create main entry point script `robot_control_gui.py`
  - Create `.gitignore` for Python projects
  - _Requirements: 10.1, 10.3, 10.5_
  - _Status: Completed - Repository created at https://github.com/goldjunge91/robot_control_gui_
  - _Path: '/home/marco/ros2_steel_ws/my_steel-robot_ws/robot_control_gui'

- [x] 2. Implement configuration management system
  - Create `ConfigManager` class in `core/config_manager.py`
  - Implement JSON-based config loading and saving
  - Implement dot-notation config access (get/set methods)
  - Implement config validation with error reporting
  - Create default configuration schema with all required fields
  - Implement config file creation in `~/.config/robot_control_gui/`
  - _Requirements: 6.1, 6.2, 6.3, 6.4, 6.5_

- [ ] 3. Implement process management system
  - Create `ProcessManager` class in `core/process_manager.py`
  - Implement `start_process()` method with subprocess.Popen for async execution
  - Implement real-time stdout/stderr capture using threads
  - Implement `stop_process()` method with graceful termination (SIGTERM then SIGKILL)
  - Implement process status tracking (PID, uptime, exit code)
  - Implement `cleanup_all()` method for application shutdown
  - Create `ProcessInfo` dataclass with uptime calculation properties
  - _Requirements: 1.1, 1.4, 2.5, 3.4, 4.5, 5.1_

- [ ] 4. Implement SSH management for remote robot control
  - Create `SSHManager` class in `core/ssh_manager.py`
  - Implement SSH connection using paramiko with key-based authentication
  - Implement `execute_command()` method with real-time output streaming
  - Implement `execute_script()` method for running shell scripts remotely
  - Implement connection testing and error handling
  - Implement automatic reconnection on connection loss
  - _Requirements: 2.2, 2.3, 2.5, 2.6, 6.6_

- [ ] 5. Implement ROS2 interface for monitoring
  - Create `ROS2Interface` class in `core/ros2_interface.py`
  - Implement ROS2 context initialization with rclpy
  - Implement `get_node_list()` using subprocess to call `ros2 node list`
  - Implement `get_topic_list()` using subprocess to call `ros2 topic list`
  - Implement `echo_topic()` for real-time topic message display
  - Implement ROS2 environment detection
  - _Requirements: 9.1, 9.2, 9.3, 9.4, 9.5, 9.6_

- [ ] 6. Implement firmware builder
  - Create `FirmwareBuilder` class in `core/firmware_builder.py`
  - Implement `build()` method to execute make commands in firmware directory
  - Implement build mode selection (debug/release)
  - Implement `flash()` method to copy .uf2 file to Pico mount point
  - Implement Pico BOOTSEL mode detection by checking for RPI-RP2 mount
  - Implement build progress tracking and output streaming
  - _Requirements: 8.1, 8.2, 8.3, 8.4, 8.5, 8.6_

- [ ] 7. Implement log management system
  - Create `LogManager` class in `core/log_manager.py`
  - Implement timestamped log file creation for each process
  - Implement log writing with automatic flushing
  - Implement log rotation based on file size
  - Implement log directory management in `~/.local/share/robot_control_gui/logs/`
  - _Requirements: 5.6_

- [ ] 8. Create main window and application structure
  - Create `MainWindow` class in `ui/main_window.py`
  - Implement QMainWindow with menu bar (File, Edit, View, Tools, Help)
  - Implement status bar with message display and timestamp
  - Implement QTabWidget for hosting tab widgets
  - Implement config loading on startup
  - Implement cleanup on closeEvent to stop all processes
  - Create application icon and window title
  - _Requirements: 7.1, 7.2, 7.3_

- [ ] 9. Implement dashboard tab
  - Create `DashboardWidget` class in `ui/dashboard_widget.py`
  - Create status indicator widgets with colored icons (green/red/gray)
  - Create status indicators for: teleoperation, robot system, simulation, micro-ROS agent
  - Create quick action buttons for common operations
  - Create recent activity log viewer (QTextEdit, read-only)
  - Implement signal connections to trigger actions in other tabs
  - Implement real-time status updates from other components
  - _Requirements: 7.1, 7.2, 7.3, 7.4, 7.5, 7.6_

- [ ] 10. Implement teleoperation tab
  - Create `TeleopWidget` class in `ui/teleop_widget.py`
  - Create Start/Stop teleoperation buttons
  - Create joystick status indicator with device path display
  - Create controller button mapping display panel
  - Create log viewer for teleoperation output
  - Implement joystick detection using `/dev/input/js*` check
  - Implement process start with joy node and teleop_twist_joy node
  - Implement process stop with cleanup
  - Connect to ProcessManager for process lifecycle management
  - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5, 1.6_

- [ ] 11. Implement robot control tab
  - Create `RobotWidget` class in `ui/robot_widget.py`
  - Create connection settings inputs (hostname, username, SSH key path)
  - Create Start/Stop Robot buttons
  - Create component status indicators (micro-ROS, bringup, Foxglove)
  - Create Foxglove URL display with copy-to-clipboard button
  - Create tabbed log viewer for each component
  - Implement SSH connection and remote script execution
  - Implement Test Connection button functionality
  - Connect to SSHManager for remote operations
  - _Requirements: 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 6.6_

- [ ] 12. Implement simulation tab
  - Create `SimulationWidget` class in `ui/simulation_widget.py`
  - Create Start/Stop Simulation buttons
  - Create world selection dropdown populated from `src/robot/worlds/`
  - Create tmux mode checkbox
  - Create Launch RViz button
  - Create simulation status panel
  - Create log viewer for simulation output
  - Implement simulation launch with world parameter
  - Implement tmux mode launch using start_sim_tmux.sh
  - Connect to ProcessManager for process lifecycle management
  - _Requirements: 3.1, 3.2, 3.3, 3.4, 3.5, 3.6_

- [ ] 13. Implement firmware tab
  - Create `FirmwareWidget` class in `ui/firmware_widget.py`
  - Create build mode selector (Debug/Release dropdown)
  - Create Build Firmware button
  - Create Flash Firmware button
  - Create Monitor Firmware button
  - Create build progress bar
  - Create build/flash log viewer
  - Implement firmware build with mode selection
  - Implement firmware flash with Pico detection
  - Implement firmware monitoring launch
  - Connect to FirmwareBuilder for build/flash operations
  - _Requirements: 8.1, 8.2, 8.3, 8.4, 8.5, 8.6_

- [ ] 14. Implement monitoring tab
  - Create `MonitoringWidget` class in `ui/monitoring_widget.py`
  - Create node list widget (QListWidget)
  - Create topic list widget (QListWidget)
  - Create topic message viewer (QTextEdit)
  - Create Refresh button
  - Create Start/Stop Echo buttons
  - Implement node list refresh
  - Implement topic list refresh with message types
  - Implement topic echo with message display
  - Connect to ROS2Interface for ROS2 queries
  - _Requirements: 9.1, 9.2, 9.3, 9.4, 9.5, 9.6_

- [ ] 15. Implement settings dialog
  - Create `SettingsDialog` class in `ui/settings_dialog.py`
  - Create tabbed settings interface (Robot, Teleoperation, Simulation, Firmware, UI, Paths)
  - Create input fields for all configuration options
  - Implement input validation for each field
  - Implement Save/Cancel buttons
  - Implement Test Connection button in Robot settings
  - Connect to ConfigManager for loading/saving settings
  - _Requirements: 6.1, 6.2, 6.3, 6.4, 6.5, 6.6_

- [ ] 16. Implement error handling and user notifications
  - Create error message dictionary with titles, messages, and suggestions
  - Implement error dialog display using QMessageBox
  - Implement status bar notifications for background errors
  - Implement error logging to file
  - Add error handling to all process start/stop operations
  - Add error handling to SSH connection operations
  - Add error handling to ROS2 operations
  - _Requirements: 1.2, 2.6, 4.6, 9.6_

- [ ] 17. Implement logging and output streaming
  - Create output streaming mechanism using QThread for non-blocking reads
  - Implement log viewer widget with auto-scroll functionality
  - Implement log line limit (1000 lines) with circular buffer
  - Implement log highlighting (errors in red, warnings in yellow)
  - Implement Clear Logs button functionality
  - Implement Save Logs button functionality
  - Connect log viewers to ProcessManager output callbacks
  - _Requirements: 5.1, 5.2, 5.3, 5.4, 5.5, 5.6_

- [ ] 18. Implement UI styling and theming
  - Create color scheme constants for status indicators
  - Implement system theme detection (light/dark mode)
  - Create consistent spacing and layout using QGroupBox
  - Implement minimum window size (1024x768)
  - Create application icon
  - Implement keyboard shortcuts (Ctrl+T, Ctrl+R, Ctrl+S)
  - Implement proper tab order for accessibility
  - _Requirements: 10.1, 10.2, 10.3, 10.4_

- [ ] 19. Finalize application packaging and documentation
  - Update main entry point in `robot_control_gui.py` to integrate MainWindow (already has argument parsing and ROS2 check)
  - Pin dependency versions in requirements.txt
  - Create README.md with installation and usage instructions
  - Create .desktop file for Linux desktop integration
  - _Requirements: 10.1, 10.4, 10.5_
  - _Note: setup.py, requirements.txt, and basic entry point already created in task 1_

- [ ] 20. Integration and end-to-end testing
  - Test teleoperation start/stop with joystick connected and disconnected
  - Test robot control via SSH with valid and invalid credentials
  - Test simulation launch in normal and tmux modes
  - Test firmware build and flash workflow
  - Test ROS2 monitoring with active and inactive ROS2 environment
  - Test configuration save/load persistence
  - Test graceful shutdown with multiple processes running
  - Test error handling for all failure scenarios
  - _Requirements: All requirements_
