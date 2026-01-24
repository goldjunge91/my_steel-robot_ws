# Justfile for my_steel Robot Workspace
# ROS2 Humble - Mecanum Drive Robot with micro-ROS

set shell := ["bash", "-c"]

ros_setup := 'source /opt/ros/humble/setup.bash && if [ -f install/setup.bash ]; then source install/setup.bash; fi'

# Show available recipes
default:
    @just --list

# ====================================
# Local ROS2 Workspace Commands
# ====================================

# Build the entire workspace (alle ROS2 Pakete in src/ kompilieren)
build:
    {{ros_setup}} && colcon build --symlink-install --event-handlers console_direct+

# Build workspace with merge-install (Production: alle Pakete in ein install/ Verzeichnis)
build-merge:
    {{ros_setup}} && colcon build --merge-install --symlink-install --event-handlers console_direct+

# Build nur Hardware-Pakete (robot_hardware_interfaces + robot_bringup)
build-hardware:
    {{ros_setup}} && colcon build --packages-up-to robot_hardware_interfaces robot_bringup --symlink-install --event-handlers console_direct+

# Build ein spezifisches Paket (z.B.: just build-pkg robot_description)
build-pkg package:
    {{ros_setup}} && colcon build --packages-select {{package}} --symlink-install --event-handlers console_direct+

# Build ein Paket inkl. Abhängigkeiten (z.B.: just build-pkg-deps robot_bringup)
build-pkg-deps package:
    {{ros_setup}} && colcon build --packages-up-to {{package}} --symlink-install --event-handlers console_direct+

# Tests ausführen (robot_hardware_interfaces + robot_controller)
test:
    {{ros_setup}} && colcon test --packages-select robot_hardware_interfaces robot_controller

# Alle Tests ausführen (alle Pakete im Workspace)
test-all:
    {{ros_setup}} && colcon test

# Test-Ergebnisse anzeigen (nach 'just test' oder 'just test-all')
test-results:
    colcon test-result --all --verbose

# Build-Artefakte löschen (build/, install/, log/ Verzeichnisse entfernen). Nutze dies bei Build-Problemen für einen sauberen Neustart
clean:
    rm -rf build install log

# ROS-Abhängigkeiten installieren (rosdep prüft src/ Pakete und installiert fehlende System-Dependencies)
deps:
    {{ros_setup}} && rosdep install --from-paths src --ignore-src -r -y

# Shell öffnen mit ROS-Umgebung (ROS2 + Workspace automatisch gesourced)
shell:
    {{ros_setup}} && exec bash

# ====================================
# Robot Runtime Commands (Roboter starten & steuern)
# ====================================

# Roboter starten (Hardware-Interfaces, Controller, Sensoren). Standard: robot_xl Modell
start model="robot_xl":
    {{ros_setup}} && ros2 launch robot_bringup bringup.launch.py robot_model:={{model}}

# micro-ROS Agent starten (Verbindung zum Pico-Mikrocontroller, automatische Port-Erkennung)
start-microros:
    {{ros_setup}} && python3 scripts/launch_microros_agent.py

# micro-ROS Agent mit spezifischem Device starten (z.B. /dev/ttyACM0 für Pico)
start-microros-dev device="/dev/ttyACM0":
    {{ros_setup}} && ros2 run micro_ros_agent micro_ros_agent serial --dev {{device}} -b 115200 -v

# Foxglove Bridge starten (Webbasierte Visualisierung auf Port 8765, Alternative zu RViz)
start-foxglove:
    {{ros_setup}} && ros2 run foxglove_bridge foxglove_bridge --port 8765

# Keyboard-Teleop starten (Roboter mit Tastatur steuern: i,j,k,l für Bewegung)
teleop:
    {{ros_setup}} && ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Teleop mit Topic-Remapping (für spezifische Controller-Topics)
teleop-remap topic="/mecanum_cont/cmd_vel_unstamped":
    {{ros_setup}} && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:={{topic}}

# ====================================
# Simulation Commands
# ====================================

# Launch Gazebo simulation
sim:
    {{ros_setup}} && ros2 launch robot_gazebo launch_sim.launch.py use_sim_time:=true

# Launch tmux-based simulation helper (multi pane workflow)
sim-tmux:
    ./start_sim_tmux.sh

# ====================================
# Firmware Commands
# ====================================

# Build firmware for Raspberry Pi Pico (debug)
firmware-build:
    cd firmware && make build

# Build firmware in release mode
firmware-build-release:
    cd firmware && make build_release

# Flash firmware to Pico (requires BOOTSEL mode)
firmware-flash:
    cd firmware && make flash

# Flash release firmware to Pico
firmware-flash-release:
    cd firmware && make flash-release

# Monitor firmware debug output and micro-ROS connection
firmware-monitor:
    ./firmware/monitor_firmware.sh

# Test firmware and micro-ROS connection
firmware-test:
    ./scripts/test_firmware_connection.sh

# ====================================
# Docker Commands - Bauen & Pushen
# ====================================

# Docker Test-Image lokal bauen (ARM64 für Raspberry Pi, --load lädt ins lokale Docker)
docker-build-test:
    docker buildx build --platform linux/arm64 -f docker/Dockerfile.robot-pi.test -t goldjunge491/my-steel-robot:test-local --load .

# Docker Production-Image lokal bauen (ARM64 für Raspberry Pi Deployment)
docker-build-prod:
    docker buildx build --platform linux/arm64 -f docker/Dockerfile.robot-pi -t goldjunge491/my-steel-robot:prod-local --load .

# Test-Image bauen und zu Docker Hub pushen (für CI/CD oder Team-Sharing)
docker-push-test:
    docker buildx build --platform linux/arm64 -f docker/Dockerfile.robot-pi.test -t goldjunge491/my-steel-robot:test-latest --push .

# Production-Image bauen und zu Docker Hub pushen (finales Release)
docker-push-prod:
    docker buildx build --platform linux/arm64 -f docker/Dockerfile.robot-pi -t goldjunge491/my-steel-robot:humble-arm64 --push .

# Production-Image mit mehreren Tags pushen (latest + Datum + Git-Hash, inkl. Logging)
docker-push-prod-tags:
    #!/usr/bin/env bash
    TAG_DATE=$(date +%Y%m%d)
    TAG_GIT=$(git rev-parse --short HEAD)
    docker buildx build --platform linux/arm64 -f docker/Dockerfile.robot-pi \
      -t goldjunge491/my-steel-robot:humble-arm64 \
      -t goldjunge491/my-steel-robot:latest \
      -t goldjunge491/my-steel-robot:${TAG_DATE}-${TAG_GIT} \
      --push . 2>&1 | tee docker-build-production.log

# ====================================
# Docker Commands - Running Containers
# ====================================

# Run test image interactively as robot user
docker-run-test:
    docker run -it --rm goldjunge491/my-steel-robot:test-local /bin/bash

# Run test image as root user
docker-run-test-root:
    docker run -it --rm --entrypoint /bin/bash --user root goldjunge491/my-steel-robot:test-local

# Run production image interactively
docker-run-prod:
    docker run -it --rm goldjunge491/my-steel-robot:prod-local /bin/bash

# Execute command in running container
docker-exec container cmd:
    docker exec -it {{container}} {{cmd}}

# Open bash shell in running container
docker-shell container:
    docker exec -it {{container}} /bin/bash

# Open bash shell in running container as root
docker-shell-root container:
    docker exec -it -u root {{container}} /bin/bash

# ====================================
# Docker Compose Commands (Services orchestrieren)
# ====================================

# Robot-Services starten (Test-Konfiguration, inkl. micro-ROS Agent)
compose-up-test:
    docker compose -f docker/compose.robot-pi.test.yaml up -d

# Robot-Services starten (Production-Konfiguration für Raspberry Pi)
compose-up:
    docker compose -f docker/compose.robot-pi.yaml up -d

# Simulation-Services starten (Gazebo in Docker)
compose-up-sim:
    docker compose -f docker/compose.simulation.yaml up -d

# Services stoppen und Container entfernen (Volumes bleiben erhalten)
compose-down file="docker/compose.robot-pi.yaml":
    docker compose -f {{file}} down

# Services stoppen + Container + Volumes löschen (komplette Zurücksetzung)
compose-down-v file="docker/compose.robot-pi.yaml":
    docker compose -f {{file}} down -v

# Logs aller Services anzeigen (live, mit -f follow)
compose-logs file="docker/compose.robot-pi.yaml":
    docker compose -f {{file}} logs -f

# Services neustarten (ohne neu zu bauen)
compose-restart file="docker/compose.robot-pi.yaml":
    docker compose -f {{file}} restart

# ====================================
# Docker Utility Commands
# ====================================

# Inspect image architecture and details
docker-inspect image="goldjunge491/my-steel-robot:humble-arm64":
    docker buildx imagetools inspect {{image}}

# Remove all unused Docker images
docker-clean-images:
    docker image prune -a -f

# Remove all stopped containers
docker-clean-containers:
    docker container prune -f

# Stop and remove all containers
docker-stop-all:
    docker stop $(docker ps -aq) && docker rm $(docker ps -aq)

# Remove unused volumes
docker-clean-volumes:
    docker volume prune -f

# Remove unused networks
docker-clean-networks:
    docker network prune -f

# Clean Docker build cache
docker-clean-cache:
    docker builder prune -a -f

# Complete Docker cleanup (careful!)
docker-clean-all: docker-clean-containers docker-clean-images docker-clean-volumes docker-clean-networks docker-clean-cache
    @echo "Docker cleanup complete!"

# ====================================
# Diagnostic Commands (System-Status prüfen)
# ====================================

# Hardware-Interfaces anzeigen (ros2_control: Gelenke, Sensoren, Motoren)
check-hardware:
    {{ros_setup}} && ros2 control list_hardware_interfaces 2>&1 | head -30

# Controller-Status anzeigen (z.B. mecanum_drive_controller: active/inactive)
check-controllers:
    {{ros_setup}} && ros2 control list_controllers

# Alle laufenden ROS2-Nodes auflisten (welche Prozesse sind aktiv?)
check-nodes:
    {{ros_setup}} && ros2 node list

# Alle verfügbaren ROS2-Topics auflisten (z.B. /cmd_vel, /joint_states)
check-topics:
    {{ros_setup}} && ros2 topic list

# Joint-States prüfen (Encoder-Werte der Räder anzeigen, einmal)
check-joints:
    {{ros_setup}} && ros2 topic echo /joint_states --once

# Pico-Verbindung prüfen (ist der Mikrocontroller über USB verbunden?)
check-pico:
    ls -l /dev/ttyACM* || echo "No Pico device found"

# Check current target configuration
check-target:
    @if [ -f .env ]; then export $(grep -v '^#' .env | xargs); fi && \
    echo "Current TARGET: $TARGET" && \
    if [ "$TARGET" = "robot" ]; then \
        echo "Configured for robot (SBC) deployment"; \
        echo "Packages: ROS2 core, hardware interfaces, micro-ROS, Pico SDK"; \
    elif [ "$TARGET" = "remote_pc" ]; then \
        echo "Configured for remote PC development"; \
        echo "Packages: ROS2 core, Gazebo, MoveIt, RQT, simulation tools"; \
    else \
        echo "TARGET not set. Please set TARGET=robot or TARGET=remote_pc in .env"; \
    fi

# List all robot-related ROS packages
check-packages:
    {{ros_setup}} && ros2 pkg list | grep -E "(micro_ros|robot)"

# ====================================
# Development Utilities
# ====================================

# Import VCS repositories
vcs-import:
    vcs import src < src/ros2.repos

# Update VCS repositories
vcs-update:
    vcs pull src

# Check VCS repository status
vcs-status:
    vcs status src

# Initialize git submodules
submodules-init:
    git submodule update --init --recursive lib/

# Update git submodules
submodules-update:
    git submodule update --remote lib/

# Format C++ code (if format.sh exists)
format:
    ./format.sh

# Show workspace structure
tree:
    @echo "=== ROS2 Packages ===" && \
    ls -l src/ && \
    echo "" && \
    echo "=== Install Directory ===" && \
    ls -l install/ 2>/dev/null || echo "Not built yet"
