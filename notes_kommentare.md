# ROS 2 Workspace & Roboter-Steuerung: Befehlsreferenz
<!-- omit from toc -->

Dies ist eine strukturierte Übersicht der Befehle und Notizen aus der bereitgestellten Datei, formatiert für bessere Lesbarkeit und Nutzung als Referenz.
<!-- omit from toc -->
## Inhaltsverzeichnis
<!-- omit from toc -->

- [ROS 2 Workspace \& Roboter-Steuerung: Befehlsreferenz](#ros-2-workspace--roboter-steuerung-befehlsreferenz)
  - [1. ROS 2 Workspace \& Roboter-Steuerung](#1-ros-2-workspace--roboter-steuerung)
  - [2. GitHub Actions (Lokales Testen mit 'act')](#2-github-actions-lokales-testen-mit-act)
  - [3. Micro-ROS \& Visualisierung](#3-micro-ros--visualisierung)
  - [4. ROS 2 Diagnose-Befehle](#4-ros-2-diagnose-befehle)
  - [5. Docker: Schneller "Smoke Test" im Container](#5-docker-schneller-smoke-test-im-container)
  - [6. Docker: Workspace-Build in sauberer Umgebung](#6-docker-workspace-build-in-sauberer-umgebung)
  - [7. Python Testing](#7-python-testing)
  - [8. Docker Build \& Push (Verschiedene Varianten)](#8-docker-build--push-verschiedene-varianten)
    - [Lokale Builds](#lokale-builds)
    - [Interaktive Tests](#interaktive-tests)
    - [Multi-Arch-Build (ARM64) für lokales Testen](#multi-arch-build-arm64-für-lokales-testen)
  - [9. Docker Compose (Für lokale Testumgebungen)](#9-docker-compose-für-lokale-testumgebungen)
    - [Interaktion mit Compose-Containern](#interaktion-mit-compose-containern)
  - [10. Docker Exec (Umfassende Referenz)](#10-docker-exec-umfassende-referenz)
  - [11. Finale Production-Builds (mit Logging)](#11-finale-production-builds-mit-logging)
  - [12. Hilfsbefehle für den Build \& Push](#12-hilfsbefehle-für-den-build--push)

-----

## 1\. ROS 2 Workspace & Roboter-Steuerung

Baut den ROS 2 Workspace mit Colcon:

* `--symlink-install`: Erstellt Symlinks statt Dateien zu kopieren (schneller).
* `--merge-install`: Legt alle Pakete in ein einziges "install"-Verzeichnis.
* `--event-handlers console_direct+`: Zeigt die Build-Ausgabe live in der Konsole an.

<!-- end list -->

```bash
colcon build --symlink-install --merge-install --event-handlers console_direct+
```

Führt das Haupt-Skript des Roboters aus (vermutlich ein Launch-File):

```bash
bash scripts/start_robot.sh
```

-----

## 2\. GitHub Actions (Lokales Testen mit 'act')

Führt einen kompletten GitHub Actions-Workflow lokal aus:

* `-W`: Gibt die Workflow-Datei an.
* `--container-architecture`: Simuliert eine ARM64-Architektur (wichtig für Pi-Builds).
* `--verbose`: Zeigt detaillierte Logs an.

<!-- end list -->

```bash
act workflow_dispatch -W .github/workflows/docker-build-robot-pi-test.yml --container-architecture linux/arm64 --verbose
```

Führt nur einen *bestimmten Job* ("build-test-image") aus dem Workflow aus:

```bash
act workflow_dispatch -W .github/workflows/docker-build-robot-pi-test.yml --container-architecture linux/arm64 --job build-test-image
```

-----

## 3\. Micro-ROS & Visualisierung

Startet den Micro-ROS Agent als Brücke zum Mikrocontroller (`--dev /dev/ttyACM0` gibt den seriellen Port/USB des Mikrocontrollers an):

```bash
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

Startet die Foxglove Bridge, um ROS 2-Daten in der Foxglove-Web-UI zu visualisieren (Alternative zu RViz):

```bash
ros2 run foxglove_bridge foxglove_bridge --port 8765
```

-----
## 4\. Docker lokale Builds

Test-Image lokal
```bash
docker buildx build --platform linux/arm64 -f docker/Dockerfile.robot-pi.test -t goldjunge491/my-steel-robot:test-local --load .
```

Production-Image lokal  
```bash
docker buildx build --platform linux/arm64 -f docker/Dockerfile.robot-pi -t goldjunge491/my-steel-robot:prod-local --load .
```
## Container starten:

Als robot user
```bash
docker run -it --rm goldjunge491/my-steel-robot:test-local /bin/bash
```

root user
```bash
docker run -it --rm --entrypoint /bin/bash --user root goldjunge491/my-steel-robot:test-local
```

-----

## 5\. ROS 2 Diagnose-Befehle

Zeigt die verfügbaren Hardware-Interfaces von `ros2_control` an (z.B. Gelenke):

```bash
ros2 control list_hardware_interfaces
```

Zeigt die Interfaces an, leitet aber Fehler (`2>&1`) um und begrenzt die Ausgabe auf 30 Zeilen, um Terminal-Spam bei Fehlern zu verhindern:

```bash
ros2 control list_hardware_interfaces 2>&1 | head -30
```

Listet alle aktiven ROS 2-Nodes auf:

```bash
ros2 node list
```

-----

## 6\. Docker Build & Push (Verschiedene Varianten)

Wechselt in das Verzeichnis, das den Build-Kontext (Dockerfile etc.) enthält:

```bash
cd /Users/marco/Github.tmp/my_steel-robot_ws
```

Baut ein Docker-Image für ARM64 (Pi) und pusht es direkt zu Docker Hub:

* `--platform`: Gibt die Zielarchitektur an.
* `-f`: Gibt den Pfad zum Dockerfile an (hier das Test-Dockerfile).
* `-t`: Taggt das Image für das Repository.
* `--push`: Pusht das Image nach erfolgreichem Build in die Registry.

<!-- end list -->

```bash
docker buildx build \
  --platform linux/arm64 \
  -f docker/Dockerfile.robot-pi.test \
  -t goldjunge491/my-steel-robot:working-latest \
  --push \
  .
```

### Lokale Builds

Schneller lokaler Build. Nutzt den Docker-Cache und baut für die Host-Architektur (z.B. x86\_64 auf einem Laptop):

```bash
docker build -f docker/Dockerfile.robot-pi.test -t goldjunge491/my-steel-robot:test-latest .
```

Gleicher Build, aber explizit mit `DOCKER_BUILDKIT=1` aktiviert (BuildKit bietet besseres Caching und parallele Build-Schritte):

```bash
DOCKER_BUILDKIT=1 docker build -f docker/Dockerfile.robot-pi.test -t goldjunge491/my-steel-robot:test-latest .
```

### Interaktive Tests

Startet eine interaktive Shell (`-it`) im frisch gebauten Test-Image (`--rm` löscht den Container nach dem Beenden):

```bash
docker run -it --rm goldjunge491/my-steel-robot:test-latest bash
```

Dieser "Copy & Paste"-Block kann *innerhalb* der laufenden Container-Shell ausgeführt werden, um die ROS-Umgebung zu überprüfen:

```bash
source /opt/ros/humble/setup.bash && \
source /home/robot/workspace/my_steel-robot_ws/install/setup.bash && \
echo "=== ROS2 Packages ===" && \
ros2 pkg list | grep -E "(micro_ros|robot)" && \
echo "=== Workspace Structure ===" && \
ls -la /home/robot/workspace/my_steel-robot_ws/ && \
echo "=== Install Directory ===" && \
ls -la /home/robot/workspace/my_steel-robot_ws/install/
```

### Multi-Arch-Build (ARM64) für lokales Testen

Baut für ARM64, aber lädt das Image in den lokalen Docker-Daemon (`--load`). Nützlich auf M1/M2 Macs oder um das Image vor dem Push zu prüfen:

```bash
docker buildx build --platform linux/arm64 -f docker/Dockerfile.robot-pi.test -t goldjunge491/my-steel-robot:pi-test-local . --load
```

-----

## 7\. Docker Compose (Für lokale Testumgebungen)

Stoppt alle Dienste und löscht die Volumes (`-v`) für einen sauberen Neustart:

```bash
docker compose -f docker/compose.robot-pi.test.yaml down -v
```

Startet alle Dienste (z.B. 'microros-agent') im Hintergrund (`-d`):

```bash
docker compose -f docker/compose.robot-pi.test.yaml up -d
```

### Interaktion mit Compose-Containern

Öffnet eine interaktive Shell im Service, der in der Compose-Datei "microros-agent" heißt:

```bash
docker compose exec -it microros-agent /bin/bash
```

Fallback-Befehl, falls der Container kein `/bin/bash` hat (`/bin/sh` ist fast immer da):

```bash
docker compose exec -it microros-agent /bin/sh
```

-----

## 8\. Docker Exec (Umfassende Referenz)

Öffnet eine interaktive Shell im Container namens "microros-agent":

```bash
docker exec -it microros-agent /bin/bash
```

Fallback mit `/bin/sh`:

```bash
docker exec -it microros-agent /bin/sh
```

Führt den Befehl als ein anderer Benutzer aus (hier: `root`):

```bash
docker exec -it -u root microros-agent /bin/bash
```

Führt `id` als Benutzer mit der UID 1000 aus (nützlich für Berechtigungen):

```bash
docker exec -it -u 1000 microros-agent id
```

Führt einen nicht-interaktiven Befehl aus (z.B. Logs auflisten):

```bash
docker exec microros-agent ls -la /var/log/robot
```

Nutzt `sh -c`, um einen Befehl mit Shell-Funktionen (wie `&&` oder `|`) auszuführen:

```bash
docker exec microros-agent sh -c 'ros2 topic list'
```

Führt einen Befehl im Hintergrund (detached) im Container aus:

```bash
docker exec -d microros-agent sh -c 'long_running_cmd &'
```

Setzt eine Umgebungsvariable nur für diesen einen Befehl:

```bash
docker exec -e ROS_DOMAIN_ID=1 microros-agent env | grep ROS_DOMAIN_ID
```

Lädt Umgebungsvariablen aus einer Datei für diesen Befehl:

```bash
docker exec --env-file ./env.list microros-agent env
```

Setzt das Arbeitsverzeichnis (`-w`) für den Befehl (`-lc` startet bash als "login shell", um `.bashrc`/.profile zu laden):

```bash
docker exec -w /home/ros microros-agent bash -lc 'pwd && ls'
```

Findet dynamisch die Container-ID/Namen über einen Filter und führt `exec` darauf aus:

```bash
docker exec -it $(docker ps -qf "name=microros-agent") /bin/bash
```

Ändert die "detach"-Tastenkombination (Standard ist Strg+P, Strg+Q):

```bash
docker exec --detach-keys="ctrl-x" -it microros-agent /bin/bash
```

`docker container exec` ist der modernere, explizitere Alias für `docker exec`:

```bash
docker container exec -it microros-agent /bin/bash
```

-----

## 9\. Finale Production-Builds (mit Logging)

Baut das *finale* Production-Image (`docker/Dockerfile.robot-pi`) für ARM64, pusht es zu Docker Hub und leitet die gesamte Ausgabe (`stdout` & `stderr`) in eine Log-Datei *und* auf die Konsole (dank `tee`):

```bash
docker buildx build --platform linux/arm64 -f docker/Dockerfile.robot-pi -t goldjunge491/my-steel-robot:humble-arm64 --push . 2>&1 | tee docker-build-production.log
```

Baut das *Production*-Image (`Dockerfile.robot-pi`), lädt es aber lokal (`--load`) und speichert das Log in einem Unterverzeichnis:

```bash
docker buildx build \
  --platform linux/arm64 \
  -f docker/Dockerfile.robot-pi \
  -t goldjunge491/my-steel-robot:humble-arm64 \
  --load . \
  2>&1 | tee docker/docker-build-production-fixed.log
```

Baut das *Test*-Image (`Dockerfile.robot-pi.test`), lädt es lokal. Das Log wird nun korrekterweise als 'test-fixed.log' gespeichert:

```bash
docker buildx build \
  --platform linux/arm64 \
  -f docker/Dockerfile.robot-pi.test \
  -t goldjunge491/my-steel-robot:humble-arm64 \
  --load . \
  2>&1 | tee docker/docker-build-test-fixed.log
```

**Best Practice:** Baut das Production-Image und versieht es mit mehreren Tags (z.B. spezifisch, `latest` und ein einzigartiger Build-Zeitstempel). Pusht alle Tags und loggt den gesamten Vorgang:

```bash
docker buildx build --platform linux/arm64 -f docker/Dockerfile.robot-pi \
  -t goldjunge491/my-steel-robot:humble-arm64 \
  -t goldjunge491/my-steel-robot:latest \
  -t goldjunge491/my-steel-robot:$(date +%Y%m%d)-$(git rev-parse --short HEAD) \
  --push . 2>&1 | tee docker-build-production.log
```

-----

## 10\. Hilfsbefehle für den Build & Push

Beobachtet die Log-Datei des Builds in Echtzeit (in einem separaten Terminal):

```bash
tail -f docker-build-production.log
```

Überprüft *nach* dem Push, ob das Image korrekt in der Registry (Docker Hub) mit den richtigen Architekturen (z.B. `linux/arm64`) vorhanden ist:

```bash
docker buildx imagetools inspect goldjunge491/my-steel-robot:humble-arm64
```

Lokaler Test: Lädt das frisch gepushte Image aus der Registry und führt einen schnellen Test aus (prüft, ob 'robot'-Pakete da sind):

```bash
docker run --rm -it goldjunge491/my-steel-robot:humble-arm64 bash -c "source /opt/ros/humble/setup.bash && source /home/robot/workspace/my_steel-robot_ws/install/setup.bash && ros2 pkg list | grep robot"
```
