---
title: Raspberry Pi Setup Guide
summary: Quick start guide for robot deployment
description: Step-by-step guide for setting up the my_steel robot on Raspberry Pi with Docker or manual installation
keywords: raspberry pi, deployment, docker, setup, installation
author: goldjunge91
beta: true
order: 6
---

!!! info "Deployment-Optionen"
    Es gibt zwei Möglichkeiten, den Roboter auf dem Raspberry Pi zu betreiben:

    1. **Docker Deployment (Empfohlen)**: Containerisierte Lösung mit allen Abhängigkeiten vorinstalliert
    2. **Manuelle Installation**: Traditionelle Installation direkt auf dem System

## Option 1: Docker Deployment (Empfohlen)

!!! note "Vorteile"
    - +heroicons:check-circle+ Alle Abhängigkeiten vorinstalliert
    - +lucide:refresh-cw+ Einfache Updates durch neue Images
    - +lucide:container+ Automatische Service-Orchestrierung
    - +heroicons:lock-closed+ Integrierte Tailscale VPN-Unterstützung
    - +lucide:hard-drive+ Persistente Logs und Konfiguration

!!! info "Detaillierte Anweisungen"
    Für vollständige Docker-Setup-Anweisungen siehe [README.md](README.md#docker-deployment-alternative) und [hardware_setup.md](hardware_setup.md).

**Schnellstart:**

```bash
# Docker Image herunterladen
docker pull mysteel/robot:humble-arm64

# Konfiguration kopieren
cp docker/compose.robot-pi.yaml ~/compose.robot-pi.yaml

# Roboter starten
docker compose -f ~/compose.robot-pi.yaml up -d
```

## Troubleshooting

## Troubleshooting

!!! info "Vollständige Troubleshooting-Anleitung"
    Für detaillierte Troubleshooting-Schritte siehe [hardware_setup.md](hardware_setup.md#troubleshooting).

### Häufige Probleme

- **Container startet nicht**: `docker compose logs` für Details
- **USB-Gerät nicht gefunden**: `ls -l /dev/ttyACM*` und Berechtigungen prüfen
- **Tailscale-Verbindung**: `docker exec robot-bringup tailscale status`

### Debug-Befehle

```bash
ros2 topic list
ros2 node list  
ros2 topic echo /joy
ros2 topic echo /cmd_vel
```

### Weitere Dokumentation

- **Hardware Setup**: Siehe [hardware_setup.md](hardware_setup.md)