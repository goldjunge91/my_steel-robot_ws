
```markdown name=robot_firmware/README.md
```markdown
# robot_firmware (Raspberry Pi Pico)

Kurz: Firmware für das Digital‑Board (Raspberry Pi Pico). Implementiert Encoder‑Lesung, Motorsteuerung, Sicherheits‑Watchdog und das Host‑Protokoll (USB‑CDC oder UART). Optional: micro-ROS Client.

Wichtige Verzeichnisse
- src/        — Haupt‑Quellcode (C / C++ / pico-sdk oder PlatformIO)
- include/    — Header & pinmap (pinmap.h)
- platformio.ini oder CMakeLists.txt — Build-Konfiguration
- docs/       — Protokoll & Board‑Spezifikationen
- README.md   — dieses Dokument

Schnellstart (Lokaler Build + Flash)
1) Voraussetzungen
   - PlatformIO oder Pico SDK Toolchain (cmake + arm gcc)
   - pyserial, esptool (je nach Board/usb)

2) Build & Flash (PlatformIO Beispiel)
   platformio run -e pico
   platformio run -t upload -e pico --upload-port /dev/ttyACM0

3) Flash Script (empfohlen)
   ./scripts/flash_firmware.sh --device /dev/ttyACM0 --board robot_pico

Kommunikationsprotokoll
- Host -> Pico: ASCII framed, Beispiel: `S v0 v1 v2 v3\r\n` (wheel velocities in rad/s)
- Pico -> Host: Encoder report: `R e0 e1 e2 e3 ts\r\n`
- Optional: CRC8 optional vor `\r\n`

Testing
- Unit tests (Host) mit simulierten encoder packets
- Integration: Host (ros2_control) gegen Pico im loopback / Hardware-in-the-loop

Wichtig: PINMAP.md in repo root ist autoritativ — halte board_config.h synchron.