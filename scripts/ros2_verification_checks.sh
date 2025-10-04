#!/bin/bash

# Dieses Skript führt eine Reihe von Überprüfungen durch, um die grundlegende
# Funktionalität des ROS2-Setups für den Roboter zu verifizieren.

echo "======================================================"
echo "Starte ROS2-Verifizierungs-Checks..."
echo "======================================================"

# --- 1. Motor-Loop verifizieren ---
echo ""
echo "--- 1. Motor-Loop Test ---"
echo "Sende Motorbefehl [0.3, 0.3, 0.3, 0.3]..."
ros2 topic pub /motors_cmd std_msgs/msg/Float32MultiArray "data: [0.3, 0.3, 0.3, 0.3]" --once

echo "Höre für 2 Sekunden auf /motors_response (sollte ansteigende Werte zeigen)..."
timeout 2s ros2 topic echo /motors_response

echo ""
echo "Warte 1 Sekunde, damit der Motor-Watchdog auslöst..."
sleep 1

echo "Höre erneut für 2 Sekunden auf /motors_response (sollte wieder nahe Null sein)..."
timeout 2s ros2 topic echo /motors_response
echo "Motor-Test abgeschlossen."

# --- 2. Servo-Test ---
echo ""
echo "--- 2. Servo Test ---"
echo "Sende Servobefehl [15.0, -10.0]..."
echo "Bitte beobachten Sie die physischen Servos."
ros2 topic pub /servo_cmd std_msgs/msg/Float32MultiArray "data: [15.0, -10.0]" --once
echo "Servo-Test abgeschlossen."

# --- 3. FreeRTOS-Taskliste ---
echo ""
echo "--- 3. FreeRTOS Task-Analyse ---"
echo "HINWEIS: Die Überprüfung der FreeRTOS-Tasks (vTaskList(), uxTaskGetStackHighWaterMark())"
echo "kann nicht durch dieses Skript automatisiert werden. Dies erfordert eine manuelle"
echo "Änderung im C-Code der Pico-Firmware (z.B. in main.c), um die Task-Infos"
echo "über den USB-Serial-Log auszugeben."

# --- 4. IMU/Battery Default-Werte prüfen ---
echo ""
echo "--- 4. IMU- und Batterie-Default-Werte ---"
echo "Prüfe IMU-Topic (/imu/data_raw) für 1 Sekunde..."
timeout 1s ros2 topic echo /imu/data_raw
echo ""
echo "Prüfe Batterie-Topic (/battery_state) für 1 Sekunde..."
timeout 1s ros2 topic echo /battery_state
echo "IMU/Batterie-Test abgeschlossen."

# --- 5. Timing/Frequenz prüfen ---
echo ""
echo "--- 5. Timing-Check für /motors_response ---"
echo "Messe die Frequenz des /motors_response Topics für 5 Sekunden..."
echo "Erwartet wird eine Frequenz von ca. 10 Hz."
timeout 5s ros2 topic hz /motors_response
echo "Timing-Check abgeschlossen."


echo ""
echo "======================================================"
echo "Alle automatisierten Checks abgeschlossen."
echo "======================================================"
