#!/bin/bash

# Skript für Remote-PC (Ubuntu 22.04 x86_64) zum Bauen des ROS2 Workspaces
# Annahmen: Workspace ist bereits geklont in ~/my_steel-robot_ws
# Für Cross-Compilation für ARM64 (Pi) wird ros_cross_compile verwendet

set -e

echo "=== ROS2 Workspace Build Skript für Remote-PC ==="

# 1. ROS2 Humble installieren
echo "1. ROS2 Humble installieren..."
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools

# 2. Workspace vorbereiten
echo "2. Workspace vorbereiten..."
cd ~/my_steel-robot_ws
source /opt/ros/humble/setup.bash

# 3. setup.sh anpassen für Remote-PC
echo "3. setup.sh anpassen für Remote-PC..."
sed -i 's/TARGET=robot/TARGET=remote_pc/g' setup.sh
# Entferne ARM-only Pakete und füge PC-Pakete hinzu
sed -i '/ros-humble-micro-ros-agent/d' setup.sh
sed -i '/ros-humble-gazebo-ros-pkgs/d' setup.sh
echo 'sudo apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-rviz2' >> setup.sh

# 4. setup.sh ausführen
echo "4. setup.sh ausführen..."
./setup.sh

# 5. Pico SDK klonen (falls nötig)
echo "5. Pico SDK klonen..."
if [ ! -d ~/pico-sdk ]; then
    git clone https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
    cd ~/pico-sdk
    git submodule update --init
    cd -
fi
export PICO_SDK_PATH=~/pico-sdk

# 6. Cross-Compilation für ARM64 einrichten
echo "6. Cross-Compilation für ARM64 einrichten..."
sudo apt install -y qemu-user-static binfmt-support
pip3 install ros_cross_compile
# Erstelle sysroot für ARM64
mkdir -p ~/arm64_sysroot
ros_cross_compile --sysroot-path ~/arm64_sysroot --rosdistro humble --arch aarch64

# 7. Bauen für x86_64
echo "7. Bauen für x86_64..."
colcon build --symlink-install

# 8. Cross-Compilation für ARM64 (Agent und Hardware)
echo "8. Cross-Compilation für ARM64..."
# Für micro-ROS Agent
cd ~/uros_agent_ws
# Installiere Fast CDR 2 und Fast DDS 3.x auf PC
# (Angenommen, bereits gebaut wie auf Pi)
# Dann cross-compile
ros_cross_compile --sysroot-path ~/arm64_sysroot --rosdistro humble --arch aarch64 --colcon-args "build --packages-select micro_ros_agent"

# 9. Pakete zurück auf Pi kopieren
echo "9. Pakete zurück auf Pi kopieren..."
# Annahme: Pi ist erreichbar via SSH
PI_IP="192.168.1.100"  # Anpassen!
scp -r ~/my_steel-robot_ws/install pi@$PI_IP:~/workspace/ros2_dev_ws/my_steel-robot_ws/
scp -r ~/uros_agent_ws/install pi@$PI_IP:~/uros_agent_ws/

echo "=== Build abgeschlossen. Teste auf Pi! ==="