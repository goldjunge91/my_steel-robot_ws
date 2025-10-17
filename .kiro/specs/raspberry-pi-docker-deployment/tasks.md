# Implementierungsplan

Konvertierung des Feature-Designs in eine Serie von Prompts für die Code-Generierung, die jeden Schritt mit inkrementellem Fortschritt implementiert. Jeder Prompt baut auf den vorherigen auf und endet mit der Integration aller Komponenten. Es gibt keine hängenden oder verwaisten Code-Teile, die nicht in einen vorherigen Schritt integriert sind. Fokus NUR auf Aufgaben, die das Schreiben, Modifizieren oder Testen von Code beinhalten.

**WICHTIG**: Es existieren bereits Docker-Dateien, GitHub Actions und Scripts. Diese müssen entsprechend den Anforderungen angepasst werden.

## Aufgaben

- [ ] 1. Bestehende Docker-Images optimieren und erweitern
  - [ ] 1.1 Aktualisiere `docker/Dockerfile.robot-pi` mit INTAS-Pattern Multi-Stage Build-Optimierungen
  - [ ] 1.2 Entferne obsolete `version: '3.8'` aus bestehenden Docker Compose-Dateien
  - [ ] 1.3 Implementiere semantische Versionierung im bestehenden Build-System
  - [ ] 1.4 Optimiere ARM64-Build-Performance und reduziere Image-Größe unter 2GB
  - [ ] 1.5 Integriere Community-Patterns aus INTAS und Husarion-Referenzen
  - _Anforderungen: 1.1, 1.2, 1.3, 1.4, 1.5, 4.1, 4.2, 4.3_

- [ ] 2. Bestehende Container-Startup-Logik erweitern
  - [ ] 2.1 Erweitere bestehende entrypoint.sh mit 30-Sekunden micro-ROS-Agent Timeout
  - [ ] 2.2 Aktualisiere ROS2-Environment-Setup für konsistente robot_xl-Konfiguration
  - [ ] 2.3 Implementiere mecanum:=true Parameter in bestehenden Launch-Kommandos
  - [ ] 2.4 Verbessere graceful Shutdown-Handling mit SIGTERM-Support
  - [ ] 2.5 Integriere Host-Netzwerk-Namespace-Konfiguration
  - _Anforderungen: 1.6, 1.7, 1.8, 1.9, 1.10, 1.11, 4.6, 4.7_

- [ ] 3. Bestehende GitHub Actions erweitern und optimieren
  - [ ] 3.1 Aktualisiere `.github/workflows/docker-build-dockerhub.yml` mit ARM64-Multi-Platform-Support
  - [ ] 3.2 Erweitere `.github/workflows/ros-docker-image.yaml` mit automatischer Versionierung
  - [ ] 3.3 Integriere bestehende CI/CD-Pipeline mit neuen Docker-Deployment-Tests
  - [ ] 3.4 Konfiguriere automatische Image-Tagging basierend auf Git-Tags
  - [ ] 3.5 Implementiere Build-Matrix für verschiedene Robot-Konfigurationen
  - _Anforderungen: Build-Automatisierung, Deployment-Unterstützung_

- [ ] 4. Bestehende Docker Compose-Konfiguration modernisieren
  - [ ] 4.1 Aktualisiere `docker/compose.robot-pi.yaml` nach aktuellen Best Practices
  - [ ] 4.2 Implementiere Named Volumes für strukturierte Log-Organisation
  - [ ] 4.3 Erweitere Service-Dependencies mit erweiterten Health-Checks
  - [ ] 4.4 Konfiguriere privileged-Mode und Device-Zugriff für Hardware-Integration
  - [ ] 4.5 Integriere Tailscale-Volume-Mounts für State-Persistenz
  - _Anforderungen: 2.1, 2.2, 2.3, 2.4, 2.5, 3.1, 3.2, 3.3, 3.4, 3.5_

- [ ] 5. Hardware-Integration in bestehender Konfiguration verbessern
  - [ ] 5.1 Erweitere USB-Device-Mapping in bestehenden Compose-Dateien
  - [ ] 5.2 Implementiere Hot-Plugging-Support ohne Container-Neustart
  - [ ] 5.3 Verbessere Geräteberechtigungen für seriellen und USB-Zugriff
  - [ ] 5.4 Erweitere bestehende Health-Checks um Hardware-Validierung
  - [ ] 5.5 Integriere USB-Kamera-Support in bestehende Vision-Pipeline
  - _Anforderungen: 2.1, 2.3, 2.4, 2.5_

- [ ] 6. Strukturiertes Log-Management implementieren
  - [ ] 6.1 Erweitere bestehende Log-Konfiguration um zeitstempel-basierte Organisation
  - [ ] 6.2 Implementiere dd_mm_yy_hh:mm Format in bestehenden Log-Volumes
  - [ ] 6.3 Konfiguriere separate Log-Streams für ROS2 und micro-ROS-Agent
  - [ ] 6.4 Erweitere bestehende Volume-Mounts für strukturierte Log-Persistenz
  - [ ] 6.5 Implementiere Log-Historie-Bewahrung über Container-Neustarts
  - [ ] 6.6 Integriere automatische Konfiguration-Updates von gemounteten Volumes
  - _Anforderungen: 5.1, 5.2, 5.3, 5.4, 5.5, 5.6_

- [ ] 7. Tailscale VPN-Integration in bestehende Infrastruktur
  - [ ] 7.1 Erweitere bestehende `tailscale-start.sh` mit Environment-Variables-Management
  - [ ] 7.2 Implementiere automatische Tailscale-Network-Verbindung beim Container-Start
  - [ ] 7.3 Konfiguriere ROS2-DDS-Traffic-Routing über VPN-Interface
  - [ ] 7.4 Implementiere Fallback auf lokale Netzwerk-Interfaces
  - [ ] 7.5 Erweitere bestehende Volume-Mounts für Tailscale-State-Persistenz
  - [ ] 7.6 Implementiere benutzerdefinierten Hostnamen-Support
  - [ ] 7.7 Konfiguriere automatische Wiederverbindung ohne erneute Authentifizierung
  - [ ] 7.8 Integriere Tailscale-Health-Checks in bestehende Monitoring-Pipeline
  - _Anforderungen: 6.1, 6.2, 6.3, 6.4, 6.5, 6.6, 6.7, 6.8_

- [ ] 8. Bestehende Test-Suite erweitern und automatisieren
  - [ ] 8.1 Erweitere `docker/test-deployment.sh` um Multi-Stage Build-Validierung
  - [ ] 8.2 Implementiere Docker Compose-Konfiguration-Validierung
  - [ ] 8.3 Erweitere bestehende ROS2-Funktionalitäts-Tests um neue Features
  - [ ] 8.4 Integriere Hardware-Integration-Tests in bestehende Test-Pipeline
  - [ ] 8.5 Implementiere Tailscale-VPN-Validierung in Test-Suite
  - [ ] 8.6 Erweitere Performance-Tests um Image-Größe und Startup-Zeit-Validierung
  - [ ] 8.7 Integriere End-to-End-Tests in bestehende CI/CD-Pipeline
  - _Anforderungen: Alle Anforderungen - Validierung_

- [ ]* 9. Bestehende Deployment-Tools erweitern
  - [ ] 9.1 Erweitere bestehende Scripts um Raspberry Pi-Erstinstallation
  - [ ] 9.2 Implementiere Update-Prozess-Automatisierung mit graceful Service-Restarts
  - [ ] 9.3 Aktualisiere bestehende `.env.robot-pi.example` mit neuen Environment-Variables
  - [ ] 9.4 Erweitere Monitoring-Scripts um Container-Status und System-Health
  - [ ] 9.5 Implementiere Log-Rotation-Scripts und automatische Cleanup-Mechanismen
  - _Anforderungen: Deployment-Unterstützung_

- [ ]* 10. Dokumentation und Troubleshooting erweitern
  - [ ] 10.1 Erweitere bestehende `docker/README.md` um neue Features
  - [ ] 10.2 Erstelle Troubleshooting-Guide für häufige Deployment-Probleme
  - [ ] 10.3 Dokumentiere Backup-Strategien für Konfiguration und Tailscale-State
  - [ ] 10.4 Aktualisiere bestehende Dokumentation mit Community-Referenzen
  - _Anforderungen: Wartbarkeit und Betrieb_