#!/bin/bash

# Kill any previous instances to ensure a clean run
killall micro_ros_agent 2>/dev/null || true

# Verbessertes Testscript für Pico micro-ROS Firmware
# Testet Publishing und Subscribing gleichzeitig, mit besserer Abbruchmöglichkeit

LOG_FILE="test_pico_firmware.log"
SUCCESS=false

# Funktion zum Loggen
log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

# Trap für sauberes Abbrechen
cleanup() {
    log "Abbruch erkannt, stoppe Agent..."
    kill $AGENT_PID 2>/dev/null
    wait $AGENT_PID 2>/dev/null
    log "Agent gestoppt."
    exit 1
}
trap cleanup INT TERM

# Log-Datei initialisieren
echo "=== Verbesserter Test Pico Firmware Log $(date) ===" > "$LOG_FILE"
log "Starte Test..."

# ROS2 Setup
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source install/setup.bash

# Agent im Hintergrund starten
log "Starte micro-ROS Agent..."
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 &
AGENT_PID=$!
sleep 5  # Warten, bis Agent bereit ist

# Prüfe, ob Agent läuft
if ! kill -0 $AGENT_PID 2>/dev/null; then
    log "FEHLER: Agent konnte nicht gestartet werden."
    exit 1
fi

log "Agent gestartet (PID: $AGENT_PID)"

# Topics auflisten
log "Verfügbare Topics:"
ros2 topic list --include-hidden-topics >> "$LOG_FILE" 2>&1

# Prüfe, ob erforderliche Topics vorhanden sind
REQUIRED_TOPICS=("/imu/data_raw" "/motors_response" "/battery_state" "/motors_cmd" "/servo_cmd")
TOPICS_OK=false
MAX_TOPIC_WAIT=30
for ((i=0; i<MAX_TOPIC_WAIT; i++)); do
    TOPICS_OK=true
    CURRENT_TOPICS=$(ros2 topic list --include-hidden-topics)
    for topic in "${REQUIRED_TOPICS[@]}"; do
        if ! printf "%s\n" "$CURRENT_TOPICS" | grep -qx "$topic"; then
            TOPICS_OK=false
            break
        fi
    done
    if $TOPICS_OK; then
        break
    fi
    sleep 1
done

if ! $TOPICS_OK; then
    for topic in "${REQUIRED_TOPICS[@]}"; do
        if ! printf "%s\n" "$CURRENT_TOPICS" | grep -qx "$topic"; then
            log "FEHLER: Topic $topic nicht gefunden."
        fi
    done
    log "FEHLER: Nicht alle Topics verfügbar."
    kill $AGENT_PID 2>/dev/null
    exit 1
fi

log "Alle erforderlichen Topics gefunden."

# Gleichzeitiges Testen: Publishing und Subscribing
log "Starte gleichzeitiges Testen von Publishing und Subscribing..."

# Publishing testen (im Hintergrund)
(
    log "Teste IMU Publishing..."
    if timeout 10 ros2 topic echo /imu/data_raw --once | grep -q "linear_acceleration:"; then
        log "IMU-Daten empfangen."
    else
        log "FEHLER: Keine IMU-Daten empfangen."
    fi
) &

IMU_PID=$!

(
    log "Teste Motor Publishing..."
    if timeout 10 ros2 topic echo /motors_response --once | grep -q "velocity:"; then
        log "Motor-Daten empfangen."
    else
        log "FEHLER: Keine Motor-Daten empfangen."
    fi
) &

MOTOR_PID=$!

(
    log "Teste Batterie Publishing..."
    if timeout 10 ros2 topic echo /battery_state --once | grep -q "voltage:"; then
        log "Batterie-Daten empfangen."
    else
        log "FEHLER: Keine Batterie-Daten empfangen."
    fi
) &

BATTERY_PID=$!

# Motor-Command senden und Response prüfen
sleep 5  # Länger warten, bis Publishing stabil ist
log "Sende Motor-Command..."
ros2 topic pub /motors_cmd std_msgs/msg/Float32MultiArray "data: [0.5, 0.5, 0.5, 0.5]" --once >> "$LOG_FILE" 2>&1
ros2 topic pub /motors_cmd std_msgs/msg/Float32MultiArray "data: [0.5, 0.5, 0.5, 0.5]" --once >> "$LOG_FILE" 2>&1  # Zweimal senden, um sicherzustellen

sleep 3  # Warten auf Response

log "Prüfe Motor-Response nach Command (empfange mehrere Nachrichten)..."
if timeout 10 ros2 topic echo /motors_response | head -20 | grep -q "0.5"; then
    log "Motor-Response nach Command empfangen (Geschwindigkeit geändert)."
    SUCCESS=true
else
    log "FEHLER: Keine Motor-Response nach Command."
fi

# Warten auf Publishing-Tests
wait $IMU_PID
wait $MOTOR_PID
wait $BATTERY_PID

# Agent stoppen
log "Stoppe Agent..."
kill $AGENT_PID 2>/dev/null
wait $AGENT_PID 2>/dev/null

# Ergebnis
if $SUCCESS; then
    log "TEST ERFOLGREICH: Pico Firmware funktioniert korrekt (bidirektional)."
    echo "SUCCESS"
else
    log "TEST FEHLGESCHLAGEN: Probleme mit Pico Firmware."
    echo "FAILURE"
fi

log "Test beendet."
