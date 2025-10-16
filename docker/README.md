# my_steel Robot Docker Deployment
## Prerequisites
Raspberry Pi 4B (4GB+ RAM), Ubuntu 22.04 ARM64, 32GB+ storage, Pico on `/dev/ttyACM0`
```bash
sudo apt-get update && sudo apt-get upgrade -y
curl -fsSL https://get.docker.com -o get-docker.sh && sudo sh get-docker.sh
sudo usermod -aG docker $USER
sudo apt-get install -y docker-compose-plugin
docker --version && docker compose version
```
## Building
### On ARM64 (Raspberry Pi)
```bash
cd /path/to/my_steel-robot_ws
docker build -f docker/Dockerfile.robot-pi -t mysteel/robot:humble-arm64 .
```
### On x86_64 (Cross-compile)
```bash
sudo apt-get install -y qemu-user-static binfmt-support
docker buildx create --name arm-builder --use
docker buildx inspect --bootstrap
docker buildx build --platform linux/arm64 -f docker/Dockerfile.robot-pi -t mysteel/robot:humble-arm64 --load .
```
### Debug Build
```bash
docker build -f docker/Dockerfile.robot-pi --build-arg CMAKE_BUILD_TYPE=Debug -t mysteel/robot:humble-arm64-debug .
```
## Push to Registry
```bash
docker login
docker tag mysteel/robot:humble-arm64 mysteel/robot:humble-arm64-v1.0.0
docker push mysteel/robot:humble-arm64
docker push mysteel/robot:humble-arm64-v1.0.0
```
## Deploy to Raspberry Pi
```bash
sudo mkdir -p /var/log/robot /etc/robot /var/lib/tailscale
sudo chown -R $USER:$USER /var/log/robot /etc/robot /var/lib/tailscale
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0005", MODE="0666"' | sudo tee /etc/udev/rules.d/99-pico.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
scp docker/compose.robot-pi.yaml pi@<raspberry-pi-ip>:~/
cat > ~/.env << 'EOF'
ROS_DOMAIN_ID=0
TAILSCALE_AUTHKEY=
TAILSCALE_HOSTNAME=robot-xl
EOF
docker pull mysteel/robot:humble-arm64
docker compose -f compose.robot-pi.yaml up -d
docker ps
docker logs microros-agent
docker logs robot-bringup
docker exec robot-bringup ros2 topic list
docker exec robot-bringup ros2 control list_controllers
```
## Manage Deployment
```bash
docker compose -f compose.robot-pi.yaml down
docker compose -f compose.robot-pi.yaml restart robot-bringup
docker compose -f compose.robot-pi.yaml logs -f
docker stats
docker compose -f compose.robot-pi.yaml pull && docker compose -f compose.robot-pi.yaml up -d
```
## Tailscale VPN
```bash
# On remote PC
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
# Get auth key from https://login.tailscale.com/admin/settings/keys
# On Raspberry Pi .env file
TAILSCALE_AUTHKEY=tskey-auth-xxxxxxxxxxxxx
TAILSCALE_HOSTNAME=robot-xl
docker compose -f compose.robot-pi.yaml down && docker compose -f compose.robot-pi.yaml up -d
docker exec robot-bringup tailscale status
docker exec robot-bringup tailscale ip
# On remote PC
tailscale status | grep robot-xl
ping robot-xl
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
ros2 topic list
```
## Troubleshooting
### Container fails to start
```bash
docker logs microros-agent
docker logs robot-bringup
docker ps -a
ls -l /dev/ttyACM0 /dev/video0
groups $USER
sudo netstat -tulpn | grep -E ':(8765|9090)'
```
### micro-ROS agent connection
```bash
ls -l /dev/ttyACM*
dmesg | tail -20
docker exec microros-agent ls -l /dev/ttyACM0
sudo udevadm control --reload-rules && sudo udevadm trigger
docker compose -f compose.robot-pi.yaml restart microros-agent
```
### Health checks failing
```bash
docker inspect microros-agent | grep -A 10 Health
docker exec microros-agent ros2 topic list
docker exec robot-bringup ros2 node list
docker exec robot-bringup ros2 control list_controllers
docker logs robot-bringup --tail 100
```
### ROS2 topics not visible
```bash
docker exec robot-bringup env | grep ROS_DOMAIN_ID
docker exec robot-bringup ros2 topic hz /joint_states
docker compose -f compose.robot-pi.yaml restart
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
```
### Tailscale issues
```bash
docker exec robot-bringup tailscale status
docker logs robot-bringup | grep -i tailscale
docker compose -f compose.robot-pi.yaml down
sudo rm -rf /var/lib/tailscale/*
docker compose -f compose.robot-pi.yaml up -d
tailscale ping robot-xl
```
### High CPU/memory
```bash
docker stats
htop
docker exec robot-bringup ps aux --sort=-%mem | head
```
### Logs
```bash
docker compose -f compose.robot-pi.yaml logs -f
docker logs microros-agent --tail 100
docker logs --timestamps robot-bringup
ls -lh /var/log/robot/
tail -f /var/log/robot/*.log
sudo journalctl -u docker -f
sudo journalctl -k | grep -i usb
```
### Network
```bash
docker exec robot-bringup ip addr show
docker exec robot-bringup ping -c 3 8.8.8.8
docker exec robot-bringup ros2 multicast receive
ros2 daemon stop && ros2 daemon start
ros2 topic list
```
### Diagnostics
```bash
uname -a
docker version
docker ps -a
docker inspect robot-bringup
ip addr show
ss -tulpn
lsusb
ls -l /dev/tty*
```
## Systemd Service (Optional)
```bash
scp docker/robot-docker.service pi@<raspberry-pi-ip>:~/
sudo cp robot-docker.service /etc/systemd/system/
sudo chmod 644 /etc/systemd/system/robot-docker.service
sudo systemctl daemon-reload
sudo systemctl enable robot-docker.service
sudo systemctl start robot-docker.service
sudo systemctl status robot-docker.service
sudo systemctl stop robot-docker.service
sudo systemctl restart robot-docker.service
sudo journalctl -u robot-docker.service -f
sudo systemctl disable robot-docker.service
sudo rm /etc/systemd/system/robot-docker.service
sudo systemctl daemon-reload
```
## Security
```bash
chmod 600 ~/.env
docker scan mysteel/robot:humble-arm64
```
