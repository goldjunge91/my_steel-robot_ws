Kurzüberblick
===============

Diese Datei erklärt, wie du das vorhandene Projekt auf ein Setup mit einem Raspberry Pi 4B (als Host für ROS2) und einem Raspberry Pi Pico (als Low-Level-MCU / micro-ROS-Client) anpasst. Alle Anweisungen sind bewusst konservativ und hardware‑sicher gehalten.

## Architektur-Entscheidung

- Raspberry Pi 4B: Host mit kompletter ROS2-Installation, läuft die Controller-Nodes, `micro-ROS-Agent` (Bridge) und höhere Funktionalität.
- Raspberry Pi Pico: Low-Level-Firmware (micro-ROS-Client) für Sensordaten, Motorsteuerung, Timings und HW-safeguards.

Kommunikation: serielle Verbindung (USB-Serial über die Pico-USB oder UART Pins). Empfehlung: Verwende USB-Serial (/dev/ttyACM0) für erste Integration, später TTL-UART für dauerhafte Verkabelung.

## Verdrahtung (Minimalsetup)

- Wenn du den Pico per USB verbindest: nur USB-C Kabel und gemeinsame Masse wird automatisch hergestellt.

- Wenn du TTL-UART verwenden willst (Pi GPIO ↔ Pico UART):
  - Verbinde Pi TX (GPIO14, /dev/serial0 TXD) an Pico RX (GPIO1, UART0 RX)
  - Verbinde Pi RX (GPIO15, /dev/serial0 RXD) an Pico TX (GPIO0, UART0 TX)
  - Gemeinsame Masse (GND↔GND)
  - Achtung: Beide arbeiten mit 3.3V, Level-Shifter nicht zwingend, aber überprüfe Spannungen und Verbindungen.

## Sicherheit & Motoren

- Motoren nie direkt an Pi/Pico anschließen. Verwende geeignete Motor-Treiber/Controller mit eigener Versorgung (z.B. H-Bridge, ESC, Motorcontroller mit EN/FAULT).
- Fuses, Not-Aus (GPIO für STOP) und separate Stromversorgung für Motoren verwenden.
- Implementiere Hard- und Software‑Watchdog auf der Pico-Firmware (periodisches heartbeats topic; falls ausbleibend => Motoren abschalten).

## micro-ROS: Agent auf dem Pi, Client auf dem Pico

- Auf dem Pi (micro-ROS-Agent) läuft die Bridge zum micro-ROS-Client auf dem Pico. Standard Kommando (wenn serieller Port /dev/ttyACM0):

  micro_ros_agent serial --dev /dev/ttyACM0 -b 115200

- Setze -b (Baudrate) und ggf. -v für verbose an deine Firmware an.

## udev-Regel (empfohlen)

Erzeuge eine udev-Regel, damit der Pico immer denselben device-Namen bekommt (z.B. /dev/pico_micro_ros). Beispieldatei findest du unter `scripts/99-pico.rules`.

## systemd-Service für micro-ROS-Agent (Beispiel)

Lege eine systemd Unit an, damit der micro-ROS-Agent beim Boot startet. Beispiel in `scripts/micro_ros_agent.service`.

## Pico-Firmware (Build & Flash)

Vorbereitung (auf deinem Entwicklungsrechner oder direkt auf dem Pi):

- Dependencies (Beispiel, Ubuntu/Debian):

  sudo apt update
  sudo apt install -y build-essential cmake git python3-pip libusb-1.0-0-dev
  pip3 install -U colcon-common-extensions

- Build (angenommen `src/pico_firmware` enthält ein CMake/micro-ROS-Client-Projekt):

  cd /home/marco/workspace/robot_ws2/src/pico_firmware
  export PICO_SDK_PATH=$(pwd)/pico-sdk   # falls pico-sdk im repo liegt
  mkdir -p build && cd build
  cmake ..
  make -j$(nproc)

- Flashen: erzeugt normalerweise eine `firmware.uf2` oder `firmware.bin`. Für UF2: schalte Pico in BOOTSEL und kopiere die .uf2 auf das gemountete Laufwerk. Für serielles Flashen/Debugging: `picotool` oder OpenOCD verwenden.

## ROS2: Launch/Config-Anpassungen

- In den Controller-Parametern (z. B. `mecanum_drive_controller` oder entsprechende `yaml`) den Serial-Port als Parameter setzen:

  controller:
    port: /dev/pico_micro_ros
    baudrate: 115200

- Passe Launch-Dateien so an, dass sie auf dem Pi die micro-ROS-Agent-Bridge starten oder voraussetzen.

## Troubleshooting

- Kein /dev/ttyACM0 nach Anstecken: prüfe dmesg, lsusb und dass Pico nicht als MassStorage ohne Bootmodus hängt.
- Latenzen: wenn USB-Serial zu langsam ist, erhöhe Baudrate und aktiviere Flow control falls notwendig.

## Troubleshooting Helper Scripts

Zwei Hilfsskripte wurden unter `scripts/` hinzugefügt, um bei der Diagnose und Wiederherstellung von häufigen USB-/Seriellen Problemen zu helfen:

- `scripts/check_pico.sh` — sammelt `dmesg`, `lsusb`, `/dev`-Listings und `udevadm`-Infos, um zu helfen, warum ein Pico nicht erkannt wird.
- `scripts/unblock_and_run_agent.sh` — zeigt Prozesse, die das Gerät halten, kann sie mit `--kill` (benötigt sudo) beenden und kann den micro-ROS-Agenten mit `--start-agent` starten.

Mach die Skripte vor der Benutzung ausführbar:

  chmod +x scripts/check_pico.sh scripts/unblock_and_run_agent.sh

Beispielverwendung:

  scripts/check_pico.sh
  sudo scripts/unblock_and_run_agent.sh /dev/ttyACM0 --kill --start-agent

## Nächste Schritte

- Wenn du willst, erstelle ich:
  - Eine angepasste Launch-Datei/param YAML für `mecanum_drive_controller` mit seriellen Parametern.
  - Ein Beispiel micro-ROS-Client (Pico) Publisher/Subcriber C-Code, ready-to-build.
  - Systemd + udev Konfigurationen (habe ich als Beispiele bereits hinzugefügt).

Dateien in diesem Repo (Beispiele)
----------------------------------

- `scripts/99-pico.rules`  -> udev-Regel
- `scripts/micro_ros_agent.service` -> systemd Unit Beispiel

Wenn du möchtest, kann ich jetzt ein Beispiel micro-ROS-Client-Projekt (Publisher für /cmd_vel und Subscriber für /odom) erstellen und in `src/pico_firmware` ablegen. Soll ich das machen?
Beispielverwendung:

  scripts/check_pico.sh
  sudo scripts/unblock_and_run_agent.sh /dev/ttyACM0 --kill --start-agent

11. Nächste Schritte

--------------------

- Wenn du willst, erstelle ich:
  - Eine angepasste Launch-Datei/param YAML für `mecanum_drive_controller` mit seriellen Parametern.
  - Ein Beispiel micro-ROS-Client (Pico) Publisher/Subcriber C-Code, ready-to-build.
  - Systemd + udev Konfigurationen (habe ich als Beispiele bereits hinzugefügt).

Dateien in diesem Repo (Beispiele)
----------------------------------

- `scripts/99-pico.rules`  -> udev-Regel
- `scripts/micro_ros_agent.service` -> systemd Unit Beispiel

Wenn du möchtest, kann ich jetzt ein Beispiel micro-ROS-Client-Projekt (Publisher für /cmd_vel und Subscriber für /odom) erstellen und in `src/pico_firmware` ablegen. Soll ich das machen?
Wenn du möchtest, kann ich jetzt ein Beispiel micro-ROS-Client-Projekt (Publisher für /cmd_vel und Subscriber für /odom) erstellen und in `src/pico_firmware` ablegen. Soll ich das machen?

- Eine angepasste Launch-Datei/param YAML für `mecanum_drive_controller` mit seriellen Parametern.
- Ein Beispiel micro-ROS-Client (Pico) Publisher/Subcriber C-Code, ready-to-build.
- Systemd + udev Konfigurationen (habe ich als Beispiele bereits hinzugefügt).

Dateien in diesem Repo (Beispiele)
----------------------------------

- `scripts/99-pico.rules`  -> udev-Regel
- `scripts/micro_ros_agent.service` -> systemd Unit Beispiel

Wenn du möchtest, kann ich jetzt ein Beispiel micro-ROS-Client-Projekt (Publisher für /cmd_vel und Subscriber für /odom) erstellen und in `src/pico_firmware` ablegen. Soll ich das machen?
Wenn du möchtest, kann ich jetzt ein Beispiel micro-ROS-Client-Projekt (Publisher für /cmd_vel und Subscriber für /odom) erstellen und in `src/pico_firmware` ablegen. Soll ich das machen?
