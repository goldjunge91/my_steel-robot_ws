#!/usr/bin/env bash

set -euo pipefail

LOG_FILE="test_motor_watchdog.log"

log(){
  echo "$(date '+%Y-%m-%d %H:%M:%S') - $*" | tee -a "$LOG_FILE"
}

: >"$LOG_FILE"
log "Starte motor-watchdog Test"

if ! pgrep -f "micro_ros_agent" >/dev/null; then
  log "FEHLER: micro_ros_agent läuft nicht. Bitte zuerst starten."
  exit 1
fi

log "Echo auf /motors_response (3 Sekunden)..."
if ! timeout 3 ros2 topic echo /motors_response --qos-profile sensor_data >>"$LOG_FILE" 2>&1; then
  log "WARNUNG: Kein Feedback empfangen (evtl. keine Hardware?)."
fi

log "Veröffentliche Motor-Kommandos"
ros2 topic pub /motors_cmd std_msgs/msg/Float32MultiArray "data: [0.5,0.5,0.5,0.5]" --once >>"$LOG_FILE" 2>&1
sleep 0.2
if timeout 3 ros2 topic echo /motors_response --once --qos-profile sensor_data | tee -a "$LOG_FILE" | grep -q "velocity"; then
  log "Motors_response reagiert auf Kommandos."
else
  log "WARNUNG: Keine Reaktion sichtbar."
fi

log "Warte 1s für Watchdog"
sleep 1
if timeout 3 ros2 topic echo /motors_response --once --qos-profile sensor_data | tee -a "$LOG_FILE" | grep -q "0.0"; then
  log "Watchdog scheint zu greifen (Werte ~0)."
else
  log "WARNUNG: Watchdog nicht sichtbar."
fi

log "Frage /get_cpu_id Service ab"
ros2 service call /get_cpu_id std_srvs/srv/Trigger {} | tee -a "$LOG_FILE"

log "Test abgeschlossen"
