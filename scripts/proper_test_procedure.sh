#!/bin/bash

echo "=== Richtiges Test-Verfahren für micro-ROS ==="
echo ""

echo "1. Alle bestehenden Agent-Prozesse beenden..."
sudo pkill -f "micro_ros_agent" || true
sleep 2

echo ""
echo "2. Serielle Verbindung freigeben..."
sudo fuser -k /dev/ttyACM0 2>/dev/null || true
sleep 1

echo ""
echo "3. Agent sauber starten..."
echo "Starte Agent im Hintergrund..."
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 &
AGENT_PID=$!
echo "Agent gestartet mit PID: $AGENT_PID"

echo ""
echo "4. Warten bis Agent bereit ist..."
sleep 3

echo ""
echo "5. Pico Reset (simuliert durch kurze Pause)..."
echo "Das Pico sollte sich jetzt neu verbinden..."
sleep 5

echo ""
echo "6. Verbindungsstatus prüfen..."
if ps -p $AGENT_PID > /dev/null; then
    echo "✅ Agent läuft noch (PID: $AGENT_PID)"
else
    echo "❌ Agent ist abgestürzt"
    exit 1
fi

echo ""
echo "7. Topics prüfen (ohne Agent zu stören)..."
echo "Verfügbare Topics:"
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 topic list | grep -E "(ddd|joint|pico)"

echo ""
echo "8. Nachrichtentest (einmalig)..."
echo "Teste Odometry (einmal):"
source /opt/ros/humble/setup.bash && source install/setup.bash
timeout 3 ros2 topic echo /ddd/odom --once > /tmp/odom_test.txt 2>&1
if [ $? -eq 0 ]; then
    echo "✅ Odometry empfangen"
    head -10 /tmp/odom_test.txt
else
    echo "❌ Keine Odometry empfangen"
fi

echo ""
echo "9. Kontinuierliche Überwachung (10 Sekunden)..."
echo "Überwache Nachrichten-Rate..."
source /opt/ros/humble/setup.bash && source install/setup.bash
timeout 10 ros2 topic hz /ddd/odom 2>/dev/null &
HZ_PID=$!

# Warte und zeige Status
for i in {1..10}; do
    echo -n "."
    sleep 1
done
echo ""

# Beende Hz-Monitoring
kill $HZ_PID 2>/dev/null || true

echo ""
echo "10. Finale Bewertung..."
if ps -p $AGENT_PID > /dev/null; then
    echo "✅ Agent ist stabil gelaufen"
    echo "✅ Test erfolgreich"
    
    echo ""
    echo "Agent läuft weiter. Zum Beenden:"
    echo "kill $AGENT_PID"
else
    echo "❌ Agent ist während des Tests abgestürzt"
    echo "❌ Test fehlgeschlagen"
fi

echo ""
echo "=== Wichtige Regeln ==="
echo "1. ❌ NIEMALS den Agent während des Tests neu starten"
echo "2. ❌ NIEMALS mehrere Agent-Instanzen gleichzeitig laufen lassen"
echo "3. ✅ Agent einmal starten und laufen lassen"
echo "4. ✅ Nur ros2 topic Befehle zum Testen verwenden"
echo "5. ✅ Bei Problemen: Agent komplett beenden, dann neu starten"