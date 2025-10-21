colcon build --symlink-install --merge-install --event-handlers console_direct+
cd /home/pi/workspace/ros2_dev_ws/my_steel-robot_ws
bash scripts/start_robot.sh 
# Micro-ROS Agent manuell starten
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
ros2 run foxglove_bridge foxglove_bridge --port 8765

 ros2 control list_hardware_interfaces
 ros2 control list_hardware_interfaces 2>&1 | head -30
 ros2 node list


https://turtlebot.github.io/turtlebot4-user-manual/

docker run --rm -v "$(pwd)":/workspace -w /workspace ros:humble bash -c "bash setup.sh && bash build.sh"  
docker run --rm -v "$(pwd)":/workspace -w /workspace ros:humble bash -c "bash setup.sh && bash build.sh && bash test.sh"  

pipx run pytest src/robot_utils/test/ -v   

lokal docker container bauen und in repo pushen

```
cd /Users/marco/Github.tmp/my_steel-robot_ws
docker buildx build \
  --platform linux/arm64 \
  -f docker/Dockerfile.robot-pi.test \
  -t goldjunge491/my-steel-robot:working-latest \
  --push \
  .
```
# Schneller Build (nutzt Cache)
docker build -f docker/Dockerfile.robot-pi.test -t robot-test:latest .

# Mit BuildKit für bessere Performance
DOCKER_BUILDKIT=1 docker build -f docker/Dockerfile.robot-pi.test -t robot-test:latest .

# Container starten
docker run -it --rm robot-test:latest bash

# Im Container - alles in einem Command:
source /opt/ros/humble/setup.bash && \
source /home/robot/workspace/my_steel-robot_ws/install/setup.bash && \
echo "=== ROS2 Packages ===" && \
ros2 pkg list | grep -E "(micro_ros|robot)" && \
echo "=== Workspace Structure ===" && \
ls -la /home/robot/workspace/my_steel-robot_ws/ && \
echo "=== Install Directory ===" && \
ls -la /home/robot/workspace/my_steel-robot_ws/install/


docker compose -f docker/compose.robot-pi.test.yaml down -v
docker compose -f docker/compose.robot-pi.test.yaml up -d

# interaktive Shell in laufendem Container (Compose)
docker compose exec -it microros-agent /bin/bash

# fallback, falls kein bash
docker compose exec -it microros-agent /bin/sh

# direkte docker exec Varianten
docker exec -it microros-agent /bin/bash
docker exec -it microros-agent /bin/sh

# als anderer User (root oder UID)
docker exec -it -u root microros-agent /bin/bash
docker exec -it -u 1000 microros-agent id

# Nicht-interaktive Befehle
docker exec microros-agent ls -la /var/log/robot
docker exec microros-agent sh -c 'ros2 topic list'

# Befehl im Hintergrund (detached)
docker exec -d microros-agent sh -c 'long_running_cmd &'

# Umgebungsvariablen setzen
docker exec -e ROS_DOMAIN_ID=1 microros-agent env | grep ROS_DOMAIN_ID
docker exec --env-file ./env.list microros-agent env

# Arbeitsverzeichnis setzen
docker exec -w /home/ros microros-agent bash -lc 'pwd && ls'

# Container-ID dynamisch auswählen
docker exec -it $(docker ps -qf "name=microros-agent") /bin/bash

# eigene Trenn-Tasten für detach
docker exec --detach-keys="ctrl-x" -it microros-agent /bin/bash

# alias: gleicher Befehl mit "docker container exec"
docker container exec -it microros-agent /bin/bash