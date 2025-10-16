# Implementation Plan

- [x] 1. Create Dockerfile for Raspberry Pi deployment
- [x] 1.1 Create multi-stage Dockerfile at `docker/Dockerfile.robot-pi`
  - Use `ros:humble-ros-base` as base image for ARM64
  - Install system dependencies (udev, usbutils, curl, gnupg)
  - Install ROS packages (ros-humble-micro-ros-agent, ros-humble-usb-cam, ros-humble-foxglove-bridge)
  - Install Tailscale client using official install script
  - Create non-root robot user with dialout and video group membership
  - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5, 11.1_

- [x] 1.2 Implement workspace build stage in Dockerfile
  - Copy entire workspace src/ directory
  - Run rosdep to install dependencies
  - Build packages with colcon using release configuration
  - Build only necessary packages: robot_bringup, robot_hardware_interfaces, robot_controller, robot_description, robot_localization_tool
  - Remove build artifacts and apt cache to reduce image size
  - _Requirements: 1.1, 1.5, 6.2, 6.3, 6.4_

- [x] 1.3 Create Tailscale startup script in Dockerfile
  - Create `/usr/local/bin/tailscale-start.sh` script
  - Check for TAILSCALE_AUTHKEY environment variable
  - Start tailscaled daemon with state persistence
  - Connect to Tailscale network with hostname and route acceptance
  - Make script executable
  - _Requirements: 11.1, 11.2, 11.3, 12.1, 12.2, 12.3_

- [x] 1.4 Configure container entrypoint
  - Create entrypoint script that sources ROS2 environment
  - Implement signal handling for graceful shutdown (SIGTERM trap)
  - Add 30-second timeout for node shutdown
  - Ensure logs are flushed before exit
  - _Requirements: 10.3, 10.4_

- [x] 2. Create Docker Compose configuration
- [x] 2.1 Create `docker/compose.robot-pi.yaml` file
  - Define microros-agent service with correct configuration
  - Define robot-bringup service with dependencies
  - Configure host network mode for both services
  - Set privileged mode for device access
  - Configure restart policy as unless-stopped
  - _Requirements: 5.1, 5.2, 5.3, 7.1, 7.2_

- [x] 2.2 Configure microros-agent service
  - Set image to mysteel/robot:humble-arm64
  - Mount /dev/ttyACM0 device
  - Set environment variables (ROS_DOMAIN_ID, RMW_IMPLEMENTATION)
  - Configure command to run micro_ros_agent with serial transport
  - Add health check using `ros2 topic list`
  - Mount /var/log/robot volume
  - _Requirements: 2.1, 2.2, 2.3, 2.4, 3.1, 3.2, 3.3, 8.1, 8.2_

- [x] 2.3 Configure robot-bringup service
  - Set dependency on microros-agent with health check condition
  - Mount /dev/video0 device for camera
  - Set environment variables including Tailscale configuration
  - Configure command to start Tailscale and launch bringup
  - Add health check using `ros2 control list_controllers`
  - Mount volumes for logs, config, and Tailscale state
  - _Requirements: 2.1, 2.2, 2.3, 4.1, 4.2, 4.3, 4.4, 4.5, 8.1, 8.2, 8.3, 8.4, 11.3, 12.4_

- [x] 2.4 Configure volume mounts and environment variables
  - Define volume mounts for /var/log/robot, /etc/robot, /var/lib/tailscale
  - Set up environment variable substitution for Tailscale auth key
  - Configure default values for optional environment variables
  - _Requirements: 5.4, 5.5, 8.1, 8.2, 8.3, 12.1, 12.2, 12.5_

- [x] 3. Create deployment documentation
- [x] 3.1 Create deployment guide at `docker/README.md`
  - Document prerequisites (Ubuntu 22.04, Docker, Docker Compose)
  - Provide build instructions for ARM64 and x86_64 with buildx
  - Document image push to Docker Hub or private registry
  - Provide deployment steps for Raspberry Pi
  - Include volume directory creation commands
  - _Requirements: 9.1, 9.2, 9.3, 9.4_

- [x] 3.2 Document Tailscale setup
  - Provide instructions for obtaining Tailscale auth key
  - Document environment variable configuration
  - Explain hostname configuration
  - Document subnet routing setup for multi-robot networks
  - _Requirements: 11.1, 11.2, 12.1, 12.2, 12.3, 12.4_

- [x] 3.3 Create troubleshooting guide
  - Document common deployment issues and solutions
  - Provide health check verification commands
  - Document log access methods (docker logs, journalctl)
  - Include network connectivity troubleshooting
  - Add Tailscale connection troubleshooting
  - _Requirements: 9.5, 10.1, 10.2, 10.5_

- [x] 4. Create systemd service for Docker Compose (optional)
- [x] 4.1 Create systemd service file template
  - Create service file at `docker/robot-docker.service`
  - Configure dependencies on docker.service and network-online.target
  - Set working directory and user
  - Configure ExecStart and ExecStop commands
  - Enable RemainAfterExit for oneshot type
  - _Requirements: 5.3, 7.4_

- [x] 4.2 Document systemd integration
  - Provide installation instructions for systemd service
  - Document service enable and start commands
  - Provide service management commands (start, stop, status, logs)
  - _Requirements: 9.4_

- [x] 5. Create example environment file
- [x] 5.1 Create `.env.robot-pi.example` file
  - Include all required environment variables with descriptions
  - Provide example values for Tailscale configuration
  - Document ROS2 configuration variables
  - Include comments explaining each variable
  - _Requirements: 5.4, 12.1, 12.2_

- [-] 6. Build and test Docker image locally
- [x] 6.1 Build Docker image for ARM64
  - Test build on ARM64 system or using buildx
  - Verify image size is under 2GB compressed
  - Check that all dependencies are installed
  - Verify ROS2 workspace is built correctly
  - _Requirements: 6.1, 6.2, 6.3, 6.4, 6.5_

- [ ] 6.2 Test image locally with Docker Compose
  - Start containers using docker compose
  - Verify microros-agent health check passes
  - Verify robot-bringup health check passes
  - Check that controllers are active
  - Verify topics are publishing data
  - _Requirements: 2.1, 2.2, 2.3, 2.4, 4.1, 4.2, 4.3, 4.4, 10.1, 10.2_

- [ ] 6.3 Test Tailscale integration
  - Configure Tailscale auth key
  - Verify Tailscale connection establishes
  - Test remote PC connectivity over Tailscale
  - Verify ROS2 topic discovery over VPN
  - _Requirements: 11.1, 11.2, 11.3, 11.4, 12.5_

- [ ] 7. Create CI/CD workflow for image builds
- [x] 7.1 Create GitHub Actions workflow at `.github/workflows/docker-build.yml`
  - Configure workflow to trigger on push to main branch
  - Set up Docker buildx for multi-platform builds
  - Build image for linux/arm64 platform
  - Push image to Docker Hub with version tags
  - _Requirements: 6.1, 6.2, 6.5_

- [x] 8. Update project documentation
- [x] 8.1 Update main README.md with Docker deployment section
  - Add Docker deployment as alternative to manual installation
  - Link to docker/README.md for detailed instructions
  - Mention Tailscale VPN support
  - _Requirements: 9.1, 9.4_

- [x] 8.2 Update raspberry_pi_setup_plan.md
  - Add Docker deployment option
  - Update troubleshooting section with Docker-specific issues
  - Reference new Docker documentation
  - _Requirements: 9.4, 9.5_
