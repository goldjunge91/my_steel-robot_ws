#!/bin/bash

echo "=== Firmware Boot Test ==="
echo "Bitte das Pico jetzt resetten (BOOTSEL + USB oder Reset-Button)"
echo "Warte auf Serial Output..."

# Warte auf das Device
while [ ! -e /dev/ttyACM0 ]; do
    echo -n "."
    sleep 0.5
done

echo ""
echo "Device gefunden, starte Monitoring..."

# Sofort nach dem Erscheinen des Devices monitoren
timeout 30 cat /dev/ttyACM0 | tee firmware_boot.log

echo ""
echo "=== Boot Log gespeichert in firmware_boot.log ==="