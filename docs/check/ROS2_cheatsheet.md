## ROS2 Cheatsheet

Kurzreferenz für häufig genutzte ROS 2 CLI-Befehle und nützliche Patterns (auf Deutsch).

## Vorbereitung
- ROS2 Umgebung laden:
  source /opt/ros/humble/setup.bash
  source install/setup.bash

- Micro-ROS Agent (seriell):
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 &
  AGENT_PID=$!
  kill $AGENT_PID

## Topics
- Topics auflisten:
  ros2 topic list
- Infos zu einem Topic (Typ, Publisher/Subscriber):
  ros2 topic info /topic_name
- Typ eines Topics anzeigen:
  ros2 topic type /topic_name
- Alle verfügbaren Topics filtern:
  ros2 topic list | grep imu

## Nachrichten ansehen (Subscribe)
- Einmalig eine Nachricht empfangen:
  ros2 topic echo /topic_name --once
- Kontinuierlich streamen:
  ros2 topic echo /topic_name
- Nur erste N Zeilen anzeigen:
  ros2 topic echo /topic_name | head -n 50
- Mit Timeout (praktisch in Scripts):
  timeout 10 ros2 topic echo /topic_name --once

## Publisher (manuell senden)
- Einmalig senden:
  ros2 topic pub /motors_cmd std_msgs/msg/Float32MultiArray "data: [0.5, 0.5, 0.5, 0.5]" --once
- Periodisch senden (z. B. 1 Hz):
  ros2 topic pub /chatter std_msgs/msg/String "data: 'hello'" -r 1
- Komplexe Message (YAML/JSON Inline):
  ros2 topic pub /pose geometry_msgs/msg/PoseStamped "{ header: { frame_id: 'map' }, pose: { position: { x: 1.0, y: 0.0, z: 0.0 }, orientation: { w: 1.0 } } }" --once

## Nodes
- Nodes auflisten:
  ros2 node list
- Infos zu einer Node (Publisher, Subscriber, Services, Aktionen):
  ros2 node info /node_name
- Node-Parameter anzeigen:
  ros2 param list /node_name
- Parameter lesen:
  ros2 param get /node_name param_name
- Parameter setzen:
  ros2 param set /node_name param_name value

## Services
- Services auflisten:
  ros2 service list
- Typ eines Services herausfinden:
  ros2 service type /service_name
- Service aufrufen (Beispiel SetBool):
  ros2 service call /service_name std_srvs/srv/SetBool "{data: true}"

## Actions (Kurz)
- Actions auflisten:
  ros2 action list
- Info zu Action:
  ros2 action info /action_name

## micro-ROS spezifisch
- Agent starten (seriell):
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 &
- Agent prüfen:
  ps aux | grep micro_ros_agent
- Agent stoppen:
  kill $AGENT_PID

## Praktische Script-Patterns
- Prüfen ob ein Topic existiert:
  if ros2 topic list | grep -q "^/imu/data_raw$"; then echo "exists"; fi

- Mehrere Topics prüfen (Bash-Array):
  REQUIRED=("imu/data_raw" "motors_response")
  for t in "${REQUIRED[@]}"; do
    if ! ros2 topic list | grep -q "$t"; then echo "missing $t"; fi
  done

- Echo + Grep kombinieren (auf Feld prüfen):
  if timeout 10 ros2 topic echo /motors_response --once | grep -q "velocity:"; then echo "motor ok"; fi

- Publish mehrfach senden zur Robustheit:
  ros2 topic pub /motors_cmd std_msgs/msg/Float32MultiArray "data: [0.5,0.5,0.5,0.5]" --once >> test.log 2>&1
  ros2 topic pub /motors_cmd std_msgs/msg/Float32MultiArray "data: [0.5,0.5,0.5,0.5]" --once >> test.log 2>&1

## Nützliche Tools & Debug
- RQT GUI:
  rqt
- ros2 bag aufnehmen:
  ros2 bag record -a
- Message-Definition anzeigen:
  ros2 interface show geometry_msgs/msg/Pose
- QoS/Performance: Setze RMW_IMPLEMENTATION oder konfiguriere im Code:
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

## Beispiel (aus `test_pico_firmware.sh`)
- Topics, die das Test-Skript prüft:
  - imu/data_raw  (IMU Daten)
  - motors_response (Antwort vom Motorcontroller)
  - battery_state (Batterieinformationen)
  - motors_cmd (Motorbefehle, Float32MultiArray)

- Typischer Ablauf im Script:
  1) Agent starten
  2) Topics auflisten und Verfügbarkeit prüfen
  3) Parallel mehrere echo-Checks mit timeout laufen lassen (IMU, Motor, Batterie)
  4) Motor-Command senden (ein- oder zweimal) und motors_response auf Änderung prüfen
  5) Agent stoppen

## Schnelle Befehle (Copy-Paste)
- Setup:
  source /opt/ros/humble/setup.bash
  source install/setup.bash
- Topics:
  ros2 topic list
  ros2 topic info /topic
  ros2 topic echo /topic --once
  ros2 topic pub /topic std_msgs/msg/String "data: 'hi'" --once
- Nodes:
  ros2 node list
  ros2 node info /node
- Services:
  ros2 service list
  ros2 service call /srv std_srvs/srv/SetBool "{data: true}"

---
