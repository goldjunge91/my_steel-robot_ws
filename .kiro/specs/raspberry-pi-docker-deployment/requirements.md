# Anforderungsdokument

## Einführung

Diese Spezifikation definiert die Anforderungen für die Erstellung eines produktionsreifen Docker-Images und Container-Orchestrierungssystems zur Bereitstellung des my_steel-Roboter-Systems auf einem Raspberry Pi 4B. Die Lösung adressiert aktuelle Bereitstellungsherausforderungen einschließlich fehlender Abhängigkeiten, Umgebungskonfigurationsproblemen und Komplexität des Service-Managements. Das Docker-Image wird den vollständigen ROS2 Humble-Arbeitsbereich mit allen notwendigen Abhängigkeiten kapseln und eine konsistente und zuverlässige Bereitstellung auf der Raspberry Pi-Hardware-Plattform gewährleisten.

## Glossar

- **Docker_Image**: Eine schreibgeschützte Vorlage, die alle notwendigen Komponenten (Code, Laufzeitumgebung, Systemwerkzeuge, Bibliotheken und Konfigurationen) enthält, um das my_steel-Roboter-System in einem isolierten, konsistenten und ausführbaren Paket auszuführen
- **Container**: Eine laufende, isolierte Instanz eines Docker_Image, die das my_steel-Roboter-System in einer eigenen Umgebung ausführt. Er wird basierend auf dem Image erstellt, enthält aber eine zusätzliche beschreibbare Ebene für dynamische Änderungen zur Laufzeit
- **ROS2_Workspace**: Ein Verzeichnis, das den gesamten Quellcode, die Build-Artefakte und die Installationsdateien der ROS2-Pakete für das my_steel-Roboter-System organisiert. Es wird mit colcon verwaltet und ermöglicht das gleichzeitige Bauen und Verwalten mehrerer Pakete
- **micro_ROS_Agent**: Ein Bridge-Service, der die Kommunikation zwischen einem leistungsschwachen Mikrocontroller (wie dem Raspberry Pi Pico) und der vollwertigen ROS2-Umgebung ermöglicht. Er übersetzt die Kommunikation zwischen dem micro-ROS-Client und dem ROS2-Netzwerk
- **Hardware_Interface**: Ein ros2_control-Plugin, das als Schnittstelle zwischen den Steuerungs-Controllern von ROS2 und der tatsächlichen my_steel-Roboter-Hardware dient. In diesem Fall kommuniziert es über micro-ROS mit der Firmware des Raspberry Pi Pico
- **Bringup_System**: Ein ROS2-Launch-System (typischerweise eine Launch-Datei), das alle notwendigen Komponenten (wie Controller, State Publisher und Sensor-Interfaces) startet und konfiguriert, um das my_steel-Roboter-System in einen betriebsbereiten Zustand zu versetzen
- **USB_Device**: Ein über USB angeschlossenes serielles Gerät, das für die Kommunikation mit der Host-Maschine (z. B. einem Docker-Container) verwendet wird. In diesem Setup ist es die Verbindung zum Raspberry Pi Pico (häufig unter /dev/ttyACM0)
- **Volume_Mount**: Ein Mechanismus in Docker, der es ermöglicht, ein Verzeichnis vom Host-System in einen Container einzubinden. Dies wird verwendet, um Daten zwischen Host und Container zu teilen und dauerhaft zu speichern
- **Environment_Variables**: Konfigurationsparameter, die an einen Container zur Laufzeit übergeben werden. Sie können das Verhalten der darin enthaltenen Anwendung steuern, ohne das Image selbst ändern zu müssen
- **Docker_Compose**: Ein Tool, das die Definition und Verwaltung von Multi-Container-Docker-Anwendungen vereinfacht. Mithilfe einer YAML-Datei können Dienste, Netzwerke und Volumes konfiguriert werden
- **Tailscale_Network**: Ein virtuelles privates Netzwerk (VPN), das von Tailscale erstellt wird. Es ermöglicht eine sichere, direkte und einfache Verbindung zwischen Geräten (wie dem Raspberry Pi und Remote-Entwicklungsmaschinen), ohne komplexe Firewall-Konfigurationen vornehmen zu müssen

**Beispiel-Log-Struktur:**
```
/var/log/robot/
├── 17_10_25_14:30/
│   ├── ros2_bringup.log
│   ├── controller_manager.log
│   └── microros_agent.log
├── 17_10_25_15:45/
│   ├── ros2_bringup.log
│   └── microros_agent.log
```

**Beispiel-Image-Versionierung:**
```
goldjunge491/my-steel-robot:1.0.0    # Erste stabile Version
goldjunge491/my-steel-robot:1.1.0    # Neue Features hinzugefügt
goldjunge491/my-steel-robot:1.1.1    # Bugfixes
goldjunge491/my-steel-robot:latest   # Zeigt auf neueste stabile Version
goldjunge491/my-steel-robot:dev      # Entwicklungsversion
```

## Anforderungen

### Anforderung 1 (Funktional)

**User Story:** Als Roboter-Operator möchte ich ein vollständig funktionsfähiges Docker-System, das alle ROS2-Abhängigkeiten enthält und das my_steel-Roboter-System automatisch startet, damit der Roboter ohne manuelle Eingriffe betriebsbereit wird.

#### Akzeptanzkriterien

1. (Funktional) WENN DAS Docker_Image erstellt wird, SOLL DAS Docker_Image eine ROS2 Humble-Basisinstallation mit allen erforderlichen Paketen enthalten
2. (Funktional) WENN DAS Docker_Image erstellt wird, SOLL DAS Docker_Image eine mit der Raspberry Pi Pico-Firmware kompatible micro_ros_agent-Paketversion enthalten
3. (Funktional) WENN DAS Docker_Image erstellt wird, SOLL DAS Docker_Image alle benutzerdefinierten ROS2-Pakete aus dem ROS2_Workspace enthalten
4. (Funktional) WENN DAS Docker_Image erstellt wird, SOLL DAS Docker_Image alle rosdep-Abhängigkeiten für die my_steel-Roboter-Pakete auflösen
5. (Nicht-funktional) WENN DAS Docker_Image erstellt wird, SOLL DAS Docker_Image mit semantischer Versionierung (MAJOR.MINOR.PATCH + v1.2.3) getaggt werden
6. (Funktional) WENN DER Container gestartet wird, SOLL DER Container die ROS2-Umgebung vor dem Starten der Services sourcen
7. (Funktional) WENN DER Container gestartet wird, SOLL DER Container das ROS2_Workspace-Install-Verzeichnis sourcen
8. (Funktional) WENN DER Container gestartet wird, SOLL DER Container das Bringup_System mit robot_model-Parameter auf robot_xl starten
9. (Funktional) WENN DER Container gestartet wird, SOLL DER Container das Bringup_System mit mecanum:=true Parameter für Mecanum-Antrieb starten
10. (Nicht-funktional) WENN DER Container gestartet wird, SOLL DER Container bis zu 30 Sekunden auf micro_ROS_Agent-Verbindung warten, bevor das Hardware_Interface gestartet wird
11. (Funktional) WENN DER Container gestartet wird, SOLL DER Container den Host-Netzwerk-Namespace für ROS2-Kommunikation verwenden

### Anforderung 2 (Funktional)

**User Story:** Als Roboter-Operator möchte ich, dass der Docker-Container auf USB-Geräte und Netzwerk-Interfaces zugreifen kann, damit das my_steel-Roboter-System mit dem Raspberry Pi Pico und externen Systemen kommunizieren kann.

#### Akzeptanzkriterien

1. (Funktional) WENN DER Container gestartet wird, SOLL DER Container Zugriff auf USB_Device unter /dev/ttyACM0 für micro-ROS-Kommunikation haben
2. (Funktional) WENN DER Container gestartet wird, SOLL DER Container den Host-Netzwerk-Modus verwenden, um ROS2 DDS-Kommunikation zu ermöglichen
3. (Funktional) WENN DER Container gestartet wird, SOLL DER Container Zugriff auf USB-Kamera-Geräte für Vision-Fähigkeiten haben
4. (Funktional) WENN DER Container gestartet wird, SOLL DER Container Geräteberechtigungen für seriellen und USB-Zugriff bewahren
5. (Nicht-funktional) WENN DER Container gestartet wird, SOLL DER Container Hot-Plugging von USB-Geräten ohne Neustart unterstützen

### Anforderung 3 (Funktional)

**User Story:** Als Roboter-Operator möchte ich eine Docker Compose-Konfiguration für einfaches Container-Management, damit ich das my_steel-Roboter-System mit einfachen Befehlen starten, stoppen und konfigurieren kann.

#### Akzeptanzkriterien

1. (Funktional) WENN DIE Docker_Compose-Datei verwendet wird, SOLL DIE Docker_Compose-Datei Service-Konfiguration für den my_steel-Roboter-Container definieren
2. (Funktional) WENN DIE Docker_Compose-Datei verwendet wird, SOLL DIE Docker_Compose-Datei Service-Konfiguration für den micro_ROS_Agent-Container definieren
3. (Nicht-funktional) WENN DIE Docker_Compose-Datei verwendet wird, SOLL DIE Docker_Compose-Datei automatische Neustart-Richtlinie für Service-Resilienz konfigurieren
4. (Funktional) WENN DIE Docker_Compose-Datei verwendet wird, SOLL DIE Docker_Compose-Datei Environment_Variables für Laufzeit-Konfiguration bereitstellen
5. (Funktional) WENN DIE Docker_Compose-Datei verwendet wird, SOLL DIE Docker_Compose-Datei Volume_Mount für persistente Konfiguration und Logs definieren


### Anforderung 4 (Nicht-funktional)

**User Story:** Als Roboter-Operator möchte ich einen optimierten und zuverlässigen Container, der effizient auf dem Raspberry Pi läuft und sich von Fehlern erholen kann, damit das System stabil und wartbar ist.

#### Akzeptanzkriterien

1. (Nicht-funktional) WENN DAS Docker_Image erstellt wird, SOLL DAS Docker_Image ARM64-Architektur für Raspberry Pi 4B anvisieren
2. (Nicht-funktional) WENN DAS Docker_Image erstellt wird, SOLL DAS Docker_Image Build-Abhängigkeiten und Cache-Dateien nach der Kompilierung entfernen
3. (Nicht-funktional) WENN DAS Docker_Image erstellt wird, SOLL DAS Docker_Image kleiner als 2GB in komprimierter Größe sein
4. (Nicht-funktional) WENN DER Container läuft, SOLL DER Container einen Health-Check-Endpunkt für Überwachung bereitstellen
5. (Nicht-funktional) WENN DER Container-Health-Check fehlschlägt, SOLL DER Container automatisch nach 3 aufeinanderfolgenden Fehlern neu starten
6. (Nicht-funktional) WENN DER Container ein SIGTERM-Signal empfängt, SOLL DER Container ROS2-Knoten graceful herunterfahren
7. (Nicht-funktional) WENN DER Container ein SIGTERM-Signal empfängt, SOLL DER Container bis zu 30 Sekunden für sauberes Herunterfahren warten
8. (Nicht-funktional) WENN DER Container ungesund ist, SOLL DER Container Diagnoseinformationen für Troubleshooting protokollieren

### Anforderung 5 (Funktional)

**User Story:** Als Roboter-Operator möchte ich, dass der Container Logs und Konfiguration persistiert, damit ich Probleme debuggen und Einstellungen über Container-Neustarts hinweg beibehalten kann.

#### Akzeptanzkriterien

1. (Funktional) WENN DER Container Logs schreibt, SOLL DER Container ROS2-Logs in Unterordnern mit Zeitstempel-Format dd_mm_yy_hh:mm und klaren Namen organisieren
2. (Funktional) WENN DER Container Logs schreibt, SOLL DER Container micro-ROS-Agent-Logs in Unterordnern mit Zeitstempel-Format dd_mm_yy_hh:mm und klaren Namen organisieren
3. (Funktional) WENN DER Container Logs schreibt, SOLL DER Container alle Logs auf einen Volume_Mount auf dem Host-System schreiben
4. (Funktional) WENN DER Container Konfiguration liest, SOLL DER Container my_steel-Roboter-Konfiguration von einem Volume_Mount auf dem Host-System lesen
5. (Nicht-funktional) WENN DER Container neu gestartet wird, SOLL DER Container Log-Historie von vorherigen Läufen bewahren
6. (Funktional) WENN DER Container neu gestartet wird, SOLL DER Container aktualisierte Konfiguration von gemounteten Volumes anwenden

### Anforderung 6 (Funktional)

**User Story:** Als Roboter-Operator möchte ich Tailscale VPN vollständig in den Container integriert, damit das my_steel-Roboter-System und Remote-PC sicher über das Internet kommunizieren können und die Konfiguration flexibel verwaltbar ist.

#### Akzeptanzkriterien

1. (Funktional) WENN DER Container gestartet wird, SOLL DER Container den Tailscale-Client initialisieren, falls ein Authentifizierungsschlüssel bereitgestellt wird
2. (Funktional) WENN DER Container mit aktiviertem Tailscale gestartet wird, SOLL DER Container sich automatisch mit dem Tailscale_Network verbinden
3. (Funktional) WENN DER Container mit Tailscale_Network verbunden ist, SOLL DER Container ROS2 DDS-Traffic über das VPN-Interface bewerben
4. (Funktional) WO Tailscale nicht konfiguriert ist, SOLL DER Container normal mit lokalen Netzwerk-Interfaces operieren
5. (Funktional) WENN DER Container für Tailscale konfiguriert ist, SOLL DER Container Tailscale-Authentifizierungsschlüssel über Environment_Variables akzeptieren
6. (Funktional) WENN DER Container für Tailscale konfiguriert ist, SOLL DER Container Tailscale-Zustand auf einem Volume_Mount für Wiederverbindung persistieren
7. (Funktional) WENN DER Container für Tailscale konfiguriert ist, SOLL DER Container benutzerdefinierten Tailscale-Hostnamen über Environment_Variables akzeptieren
8. (Nicht-funktional) WENN DER Container neu gestartet wird, SOLL DER Container sich ohne erneute Authentifizierung mit Tailscale_Network verbinden