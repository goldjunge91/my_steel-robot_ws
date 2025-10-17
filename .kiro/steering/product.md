# my_steel Robot - Produktbeschreibung

# Coding Conventions

## Code-Implementierungsregeln
- **Schreibe Code immer direkt in die korrekten Dateien** - Niemals Dummy- oder Pseudo-Code ohne Erlaubnis erstellen
- **Schreibe produktionsreifen Code** - Aller Code sollte vollständig, funktional und einsatzbereit sein
- **Keine Platzhalter-Implementierungen** - Vermeide Stub-Funktionen, TODOs oder unvollständige Logik, außer explizit angefordert

## Dokumentationsrichtlinien
- **Niemals READMEs, Zusammenfassungen, Dokumentation oder andere zusätzliche Artefakte erstellen** (wie README.md-Dateien, Projektübersichten oder automatische Berichte) ohne explizite Anfrage
- **Immer auf direkte Anweisung warten** vom Benutzer, bevor solche Inhalte generiert oder hinzugefügt werden
- **Fokus auf Code, nicht Dokumentation** - Wenn nach Feature-Implementierung gefragt wird, nur den Code schreiben
- **Ausnahme**: Inline-Code-Kommentare sind akzeptabel und für Klarheit erwünscht

## Was das bedeutet
Bei Anfragen wie "Feature hinzufügen" oder "X implementieren":
- ✅ Schreibe die tatsächliche Implementierung
- ✅ Füge notwendige Imports und Dependencies hinzu
- ✅ Inkludiere Inline-Kommentare für komplexe Logik
- ❌ Erstelle keine README zur Erklärung der Arbeit
- ❌ Schreibe keine Zusammenfassungsdokumente
- ❌ Generiere keine Projektübersichten oder Berichte
- ❌ Erstelle keinen Dummy- oder Pseudo-Code
- ❌ Schreibe keine .md-Datei ohne Erlaubnis

Der Benutzer wird nach Dokumentation fragen, wenn sie benötigt wird.

## Codebase-Untersuchung
- Erkunde relevante Dateien und Verzeichnisse
- Suche nach Schlüsselfunktionen, Klassen oder Variablen zum Problem
- Lese und verstehe relevante Code-Snippets
- Identifiziere die Grundursache des Problems
- Validiere und aktualisiere kontinuierlich das Verständnis beim Sammeln von Kontext

## Code-Änderungen durchführen
- **Vor dem Bearbeiten immer die relevanten Dateiinhalte oder Abschnitte lesen** für vollständigen Kontext
- **Immer Dateien lesen, die in der zu bearbeitenden Datei importiert werden** für vollständigen Kontext
- Stelle sicher, den vollständigen Kontext der Codebase vor Änderungen zu verstehen
- Denke immer über die Auswirkungen der Änderungen nach auf:
  - Die gesamte Codebase
  - Dependencies und Interaktionen mit anderen Code-Teilen
  - Edge Cases und potenzielle Fallstricke
  - Performance, Sicherheit und Wartbarkeit
- Falls ein Patch nicht korrekt angewendet wird, versuche ihn erneut anzuwenden
- Mache kleine, testbare, inkrementelle Änderungen, die logisch aus Untersuchung und Plan folgen

## Umgebungsvariablen
- Wenn erkannt wird, dass ein Projekt eine Umgebungsvariable benötigt (wie API-Key oder Secret), prüfe immer, ob eine `.env`-Datei im Projekt-Root existiert
- Falls sie nicht existiert, erstelle automatisch eine `.env`-Datei mit Platzhalter für die benötigte(n) Variable(n) und informiere den Benutzer
- Tue dies proaktiv, ohne auf Benutzeranfrage zu warten

**Omnidirektionale mobile Roboterplattform mit ROS2 Humble, Mecanum-Antrieb und interaktivem Nerf-Launcher für Bildung und Forschung.**

## Überblick

Der my_steel Robot ist eine omnidirektionale mobile Roboterplattform basierend auf ROS2 Humble. Das System kombiniert autonome Navigation mit einem interaktiven Nerf-Dart-Launcher für Bildung und Forschung.

## Kernfunktionen

- **Omnidirektionale Mobilität**: 4-Rad Mecanum-Antrieb für holonome Bewegung (seitlich, diagonal, Rotation auf der Stelle)
- **Autonome Navigation**: SLAM mit LiDAR (YDLIDAR LDS01RR) und Sensorfusion (IMU + Odometrie)
- **Computer Vision**: Gesichtserkennung und automatisches Tracking mit USB-Kamera
- **Interaktiver Launcher**: Nerf-Dart-System mit Pan/Tilt-Servos und Brushless-Motoren
- **Remote-Steuerung**: Xbox-Controller und Web-Dashboard über Tailscale VPN
- **Echtzeit-Kontrolle**: Raspberry Pi Pico mit micro-ROS für präzise Motorsteuerung

## Hardware-Architektur

- **High-Level**: Raspberry Pi 4B (ROS2, Navigation, Computer Vision)
- **Low-Level**: Raspberry Pi Pico (FreeRTOS, Motorsteuerung, Sensorik)
- **Launcher**: Arduino Nano/Pro Micro (dedizierte Nerf-Steuerung)
- **Sensorik**: LiDAR, 9-DoF IMU, ToF-Sensor, USB-Kamera
- **Antrieb**: 4x DC-Motoren mit Hall-Encodern, Mecanum-Räder

## Zielgruppen

- Robotik-Studenten und -Forscher
- Entwickler von autonomen Navigationssystemen
- Maker und Hobbyisten mit ROS2-Interesse
- Bildungseinrichtungen für praktische Robotik-Lehre