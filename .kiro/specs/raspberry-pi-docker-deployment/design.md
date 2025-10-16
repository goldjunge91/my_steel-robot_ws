# Design Document

## Overview

This design specifies a production-ready Docker containerization solution for deploying the my_steel robot software stack on Raspberry Pi 4B hardware. The solution addresses current deployment challenges including dependency management, environment configuration, and service orchestration through a multi-container architecture with integrated Tailscale VPN support for seamless remote access.

The design follows a microservices approach with separate containers for the micro-ROS agent and main robot system, enabling flexible deployment configurations and improved fault isolation. The solution leverages Docker Compose for orchestration and includes comprehensive health monitoring, graceful shutdown handling, and persistent storage for logs and configuration.

## Architecture

### Container Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Raspberry Pi Host                        │
│  ┌───────────────────────────────────────────────────────┐  │
│  │                  Tailscale Daemon                      │  │
│  │              (VPN Network Interface)                   │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                              │
│  ┌──────────────────────┐    ┌──────────────────────────┐  │
│  │  micro-ROS Agent     │    │   Robot Bringup          │  │
│  │  Container           │    │   Container              │  │
│  │                      │    │                          │  │
│  │  - micro_ros_agent   │◄───┤  - robot_bringup        │  │
│  │  - Serial /dev/ttyACM0│   │  - ros2_control         │  │
│  │  - FastRTPS DDS      │    │  - controller_manager   │  │
│  │  - Health checks     │    │  - state_publisher      │  │
│  │                      │    │  - Health checks        │  │
│  └──────────────────────┘    └──────────────────────────┘  │
│           │                            │                     │
│           └────────────┬───────────────┘                     │
│                        │                                     │
│                  Host Network                                │
│                  (ROS2 DDS)                                  │
│                        │                                     │
│  ┌─────────────────────┴──────────────────────────────────┐ │
│  │              Shared Volumes                             │ │
│  │  - /var/log/robot  (logs)                              │ │
│  │  - /etc/robot      (config)                            │ │
│  │  - /var/lib/tailscale (VPN state)                      │ │
│  └─────────────────────────────────────────────────────────┘ │
│                                                              │
│  ┌─────────────────────────────────────────────────────────┐│
│  │              USB Devices                                 ││
│  │  - /dev/ttyACM0 (Raspberry Pi Pico)                    ││
│  │  - /dev/video0  (USB Camera)                           ││
│  └─────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
                           │
                           │ Tailscale VPN
                           │
                  ┌────────┴────────┐
                  │   Remote PC     │
                  │                 │
                  │  - RViz2        │
                  │  - Foxglove     │
                  │  - Joy Node     │
                  │  - Teleop       │
                  └─────────────────┘
```

### Multi-Stage Build Strategy

The Docker image uses a multi-stage build to optimize size and security:

1. **Builder Stage**: Compiles ROS2 packages with all build dependencies
2. **Runtime Stage**: Contains only runtime dependencies and compiled artifacts
3. **Size Target**: < 2GB compressed, < 4GB uncompressed

### Network Architecture

- **Host Network Mode**: Containers use host networking for ROS2 DDS multicast discovery
- **Tailscale Integration**: VPN overlay network for remote access without port forwarding
- **DDS Configuration**: FastRTPS with optimized buffer sizes for robot telemetry
- **Domain Isolation**: ROS_DOMAIN_ID=0 for network segmentation

## Components and Interfaces

### 1. Base Docker Image

**Purpose**: Provides ROS2 Humble foundation with ARM64 optimization

**Base Image**: `ros:humble-ros-base` (official ROS Docker image for ARM64) or `husarnet/ros:humble-ros-core` (alternative with networking optimizations)

**Key Modifications**:
- Install micro-ROS agent from apt repository
- Install system dependencies (udev rules, USB tools)
- Configure DDS middleware (FastRTPS)
- Set up non-root user for security
- Install Tailscale client

**Dockerfile Structure**:
```dockerfile
FROM ros:humble-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-micro-ros-agent \
    ros-humble-usb-cam \
    ros-humble-foxglove-bridge \
    udev \
    usbutils \
    curl \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# Install Tailscale
RUN curl -fsSL https://tailscale.com/install.sh | sh

# Create non-root user
RUN useradd -m -s /bin/bash robot && \
    usermod -aG dialout,video robot
```

### 2. Workspace Build Layer

**Purpose**: Compiles all ROS2 packages from source

**Build Process**:
1. Copy entire workspace (src/ directory with all packages)
2. Run rosdep to resolve dependencies
3. Build with colcon using release configuration
4. Build only necessary packages: robot_bringup, robot_hardware_interfaces, robot_controller, robot_description, robot_localization_tool
5. Remove build artifacts and cache to reduce image size

**Key Considerations**:
- Use `--symlink-install` for development images
- Use `--merge-install` for production images
- Build only necessary packages (robot_bringup, robot_hardware_interfaces, robot_controller)
- Strip debug symbols in release builds

### 3. micro-ROS Agent Container

**Purpose**: Bridges communication between Pico firmware and ROS2

**Configuration**:
- Serial device: `/dev/ttyACM0` (bind mounted from host)
- Baud rate: 115200
- Verbose logging enabled
- Auto-restart on failure

**Health Check**:
```bash
ros2 topic list | grep -q "/joint_states"
```

**Startup Command**:
```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 -b 115200 -v6
```

### 4. Robot Bringup Container

**Purpose**: Runs main robot software stack

**Services Launched**:
- robot_state_publisher (URDF/TF tree)
- controller_manager (ros2_control)
- drive_controller (mecanum drive controller)
- joint_state_broadcaster
- imu_broadcaster (IMU sensor broadcaster)

**Dependencies**:
- Requires micro-ROS agent to be healthy
- Waits for `/joint_states` topic before starting controllers

**Health Check**:
```bash
ros2 control list_controllers | grep -q "drive_controller.*active"
```

**Startup Command**:
```bash
ros2 launch robot_bringup bringup.launch.py \
  robot_model:=robot_xl \
  mecanum:=True \
  microros:=false
```

### 5. Tailscale Integration

**Purpose**: Provides secure VPN connectivity for remote access

**Configuration**:
- Authentication via `TAILSCALE_AUTHKEY` environment variable
- Hostname via `TAILSCALE_HOSTNAME` environment variable (default: robot-xl)
- State persistence via volume mount at `/var/lib/tailscale`
- Subnet routing support for multi-robot networks

**Startup Script** (`/usr/local/bin/tailscale-start.sh`):
```bash
#!/bin/bash
if [ -n "$TAILSCALE_AUTHKEY" ]; then
    tailscaled --state=/var/lib/tailscale/tailscaled.state &
    sleep 2
    tailscale up --authkey=$TAILSCALE_AUTHKEY \
                 --hostname=${TAILSCALE_HOSTNAME:-robot-xl} \
                 --accept-routes
fi
```

**Network Configuration**:
- Advertise ROS2 DDS traffic over Tailscale interface
- Configure FastRTPS to use Tailscale IP for discovery
- Set `ROS_LOCALHOST_ONLY=0` for network communication

### 6. Docker Compose Orchestration

**Purpose**: Manages multi-container deployment

**Services**:
1. `microros-agent`: micro-ROS bridge service
2. `robot-bringup`: Main robot software
3. `tailscale`: VPN service (optional, can be host-level)

**Key Features**:
- Dependency ordering (agent → bringup)
- Health check-based startup coordination
- Automatic restart policies
- Volume management for persistence
- Environment variable configuration

## Data Models

### Environment Variables

```yaml
# ROS Configuration
ROS_DISTRO: humble
ROS_DOMAIN_ID: 0
RMW_IMPLEMENTATION: rmw_fastrtps_cpp

# Robot Configuration
ROBOT_MODEL_NAME: robot_xl
SERIAL_PORT: /dev/ttyACM0
SERIAL_BAUDRATE: 115200

# Tailscale Configuration
TAILSCALE_AUTHKEY: tskey-auth-xxxxx (optional)
TAILSCALE_HOSTNAME: robot-xl (optional)
TAILSCALE_SUBNET_ROUTES: "" (optional)

# Logging
LOG_LEVEL: INFO
ROS_LOG_DIR: /var/log/robot
```

### Volume Mounts

```yaml
volumes:
  # Configuration persistence
  - /etc/robot:/etc/robot:ro
  
  # Log persistence
  - /var/log/robot:/var/log/robot:rw
  
  # Tailscale state
  - /var/lib/tailscale:/var/lib/tailscale:rw
  
  # USB device access
  - /dev:/dev:rw
```

### Docker Compose Schema

```yaml
version: '3.8'

services:
  microros-agent:
    image: mysteel/robot:humble-arm64
    container_name: microros-agent
    network_mode: host
    privileged: true
    restart: unless-stopped
    devices:
      - /dev/ttyACM0:/dev/ttyACM0
    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    command: >
      ros2 run micro_ros_agent micro_ros_agent serial
      --dev /dev/ttyACM0 -b 115200 -v6
    healthcheck:
      test: ["CMD", "ros2", "topic", "list"]
      interval: 10s
      timeout: 5s
      retries: 3
      start_period: 30s
    volumes:
      - /var/log/robot:/var/log/robot

  robot-bringup:
    image: mysteel/robot:humble-arm64
    container_name: robot-bringup
    network_mode: host
    privileged: true
    restart: unless-stopped
    depends_on:
      microros-agent:
        condition: service_healthy
    devices:
      - /dev/video0:/dev/video0
    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - ROBOT_MODEL_NAME=robot_xl
      - TAILSCALE_AUTHKEY=${TAILSCALE_AUTHKEY}
      - TAILSCALE_HOSTNAME=${TAILSCALE_HOSTNAME:-robot-xl}
    command: >
      bash -c "
      /usr/local/bin/tailscale-start.sh &&
      source /opt/ros/humble/setup.bash &&
      source /ros2_ws/install/setup.bash &&
      ros2 launch robot_bringup bringup.launch.py
        robot_model:=robot_xl
        mecanum:=True
        microros:=false
      "
    healthcheck:
      test: ["CMD", "ros2", "control", "list_controllers"]
      interval: 15s
      timeout: 10s
      retries: 5
      start_period: 60s
    volumes:
      - /var/log/robot:/var/log/robot
      - /etc/robot:/etc/robot:ro
      - /var/lib/tailscale:/var/lib/tailscale
```

## Error Handling

### Container Startup Failures

**Scenario**: Container fails to start due to missing dependencies or configuration

**Detection**: Docker health checks fail during start_period

**Recovery**:
1. Log detailed error information to `/var/log/robot/startup.log`
2. Retry startup up to 3 times with exponential backoff
3. Send notification via system journal
4. Remain in failed state for manual intervention

**Prevention**:
- Validate environment variables in entrypoint script
- Check for required devices before starting services
- Verify ROS2 environment is properly sourced

### USB Device Disconnection

**Scenario**: Raspberry Pi Pico disconnects during operation

**Detection**: micro-ROS agent loses serial connection

**Recovery**:
1. micro-ROS agent container restarts automatically
2. Wait for device to reappear at `/dev/ttyACM0`
3. Re-establish serial connection
4. Robot bringup container detects agent recovery via health check
5. Controllers automatically reconnect to hardware interface

**Prevention**:
- Use reliable USB cables and connections
- Implement udev rules for consistent device naming
- Add USB power management configuration

### Network Communication Failures

**Scenario**: ROS2 DDS communication fails between containers or with remote PC

**Detection**: Topics not visible, nodes cannot discover each other

**Recovery**:
1. Verify RMW_IMPLEMENTATION is set correctly
2. Check ROS_DOMAIN_ID matches across all systems
3. Restart containers to reset DDS discovery
4. Verify Tailscale connection if using VPN

**Prevention**:
- Use FastRTPS with explicit configuration
- Disable CycloneDDS if not needed
- Configure firewall rules for DDS ports
- Use Tailscale for reliable remote connectivity

### Tailscale Connection Failures

**Scenario**: Tailscale fails to connect or authenticate

**Detection**: Tailscale status shows disconnected

**Recovery**:
1. Check TAILSCALE_AUTHKEY is valid and not expired
2. Verify network connectivity to Tailscale servers
3. Clear Tailscale state and re-authenticate
4. Fall back to local network operation

**Prevention**:
- Use reusable auth keys with appropriate expiration
- Persist Tailscale state across container restarts
- Monitor Tailscale connection status

### Graceful Shutdown

**Scenario**: Container receives SIGTERM signal

**Handling**:
1. Entrypoint script traps SIGTERM signal
2. Send SIGINT to ROS2 launch process
3. Wait up to 30 seconds for nodes to shutdown cleanly
4. Force kill remaining processes after timeout
5. Flush logs to persistent storage
6. Exit with appropriate status code

**Implementation**:
```bash
#!/bin/bash
trap 'kill -INT $PID; wait $PID' SIGTERM

# Start ROS2 launch
ros2 launch robot_bringup bringup.launch.py &
PID=$!

wait $PID
```

## Testing Strategy

### Unit Testing

**Scope**: Individual container components

**Tests**:
1. Dockerfile builds successfully for ARM64
2. All required packages are installed
3. ROS2 environment sources correctly
4. Entrypoint scripts execute without errors
5. Health check scripts return correct status

**Tools**: Docker build, shell script testing

### Integration Testing

**Scope**: Multi-container orchestration

**Tests**:
1. Containers start in correct order
2. micro-ROS agent connects to Pico
3. Robot bringup launches all required nodes
4. Controllers spawn and become active
5. Topics publish data correctly
6. Health checks pass consistently
7. Tailscale establishes VPN connection

**Tools**: Docker Compose, ROS2 CLI, pytest

### Hardware-in-the-Loop Testing

**Scope**: Full system on Raspberry Pi

**Tests**:
1. Deploy containers to Raspberry Pi
2. Verify USB device access
3. Test motor control via cmd_vel
4. Verify odometry publishing
5. Test IMU data streaming
6. Verify camera feed
7. Test remote PC connectivity via Tailscale
8. Measure resource usage (CPU, memory, network)

**Tools**: SSH, ROS2 CLI, htop, iftop

### Performance Testing

**Metrics**:
- Container startup time: < 60 seconds
- Memory usage: < 1GB per container
- CPU usage: < 50% average on Raspberry Pi 4B
- Network latency: < 50ms for local DDS, < 100ms over Tailscale
- Topic publish rate: 50Hz for joint_states, 10Hz for odometry

**Tools**: docker stats, ros2 topic hz, ros2 topic bw

### Deployment Testing

**Scenarios**:
1. Fresh Raspberry Pi installation
2. Update existing deployment
3. Rollback to previous version
4. Multi-robot deployment
5. Remote PC connection from different networks

**Validation**:
- Deployment completes without errors
- Robot becomes operational within 2 minutes
- Configuration persists across restarts
- Logs are accessible and readable
- Tailscale VPN connects successfully

## Build and Deployment Process

### Building the Image

**On ARM64 (Raspberry Pi or ARM server)**:
```bash
cd /path/to/my_steel-robot_ws
docker build -f docker/Dockerfile.robot-pi -t mysteel/robot:humble-arm64 .
```

**On x86_64 (Development machine with buildx)**:
```bash
docker buildx create --name arm-builder --use
docker buildx build --platform linux/arm64 \
  -f docker/Dockerfile.robot-pi \
  -t mysteel/robot:humble-arm64 \
  --load .
```

### Pushing to Registry

```bash
# Docker Hub
docker login
docker push mysteel/robot:humble-arm64

# Private registry
docker tag mysteel/robot:humble-arm64 registry.example.com/robot:humble-arm64
docker push registry.example.com/robot:humble-arm64
```

### Deploying to Raspberry Pi

**Prerequisites**:
1. Ubuntu 22.04 installed on Raspberry Pi
2. Docker and Docker Compose installed
3. User added to docker group
4. Tailscale account and auth key (optional)

**Deployment Steps**:
```bash
# 1. Copy docker-compose.yml to Raspberry Pi
scp docker/compose.robot-pi.yaml pi@robot:/home/pi/

# 2. Create environment file
cat > /home/pi/.env << EOF
TAILSCALE_AUTHKEY=tskey-auth-xxxxx
TAILSCALE_HOSTNAME=robot-xl
ROS_DOMAIN_ID=0
EOF

# 3. Create volume directories
sudo mkdir -p /var/log/robot /etc/robot /var/lib/tailscale
sudo chown -R pi:pi /var/log/robot /etc/robot /var/lib/tailscale

# 4. Pull image
docker pull mysteel/robot:humble-arm64

# 5. Start containers
docker compose -f compose.robot-pi.yaml up -d

# 6. Verify deployment
docker compose -f compose.robot-pi.yaml ps
docker compose -f compose.robot-pi.yaml logs -f
```

### Systemd Integration (Optional)

For automatic startup on boot:

```bash
# Create systemd service
sudo tee /etc/systemd/system/robot-docker.service << EOF
[Unit]
Description=my_steel Robot Docker Containers
Requires=docker.service
After=docker.service network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=/home/pi
ExecStart=/usr/bin/docker compose -f compose.robot-pi.yaml up -d
ExecStop=/usr/bin/docker compose -f compose.robot-pi.yaml down
User=pi

[Install]
WantedBy=multi-user.target
EOF

# Enable and start
sudo systemctl daemon-reload
sudo systemctl enable robot-docker.service
sudo systemctl start robot-docker.service
```

## Remote PC Configuration

### Tailscale Setup

```bash
# Install Tailscale on remote PC
curl -fsSL https://tailscale.com/install.sh | sh

# Connect to network
sudo tailscale up

# Verify connection to robot
tailscale status | grep robot-xl
ping robot-xl
```

### ROS2 Environment

```bash
# Set environment variables
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

# Verify topic discovery
ros2 topic list

# Launch RViz
ros2 run rviz2 rviz2

# Launch Foxglove
# Connect to ws://robot-xl:8765
```

## Security Considerations

### Container Security

- Run as non-root user where possible
- Use read-only root filesystem for immutability
- Limit capabilities with `--cap-drop=ALL --cap-add=NET_ADMIN,NET_RAW`
- Scan images for vulnerabilities with Trivy or Snyk
- Use official base images from trusted sources

### Network Security

- Tailscale provides encrypted VPN tunnel
- Use Tailscale ACLs to restrict access
- Disable SSH password authentication
- Use firewall rules to limit exposed ports
- Rotate Tailscale auth keys regularly

### Secrets Management

- Never commit auth keys to version control
- Use environment variables or Docker secrets
- Rotate credentials periodically
- Use read-only mounts for sensitive configuration

## Monitoring and Observability

### Logging

- All ROS2 logs written to `/var/log/robot`
- Docker logs accessible via `docker compose logs`
- System logs via journalctl
- Log rotation configured for disk space management

### Metrics

- Container resource usage via `docker stats`
- ROS2 topic statistics via `ros2 topic hz/bw`
- System metrics via Prometheus node exporter (optional)
- Tailscale connection status via `tailscale status`

### Alerting

- Health check failures trigger container restart
- Critical errors logged to system journal
- Optional integration with monitoring systems (Grafana, Prometheus)
- Email/SMS notifications for prolonged failures (optional)
