# Stabiler micro-ROS Test Guide

## ❌ Was du NICHT machen solltest:

```bash
# FALSCH: Agent während Test neu starten
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
# Ctrl+C
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200  # <- Das stört!
```

## ✅ Richtige Vorgehensweise:

### 1. Sauberer Start
```bash
# Alle alten Prozesse beenden
sudo pkill -f "micro_ros_agent"
sudo fuser -k /dev/ttyACM0

# Agent EINMAL starten
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 &
AGENT_PID=$!
```

### 2. Testen OHNE Agent zu stören
```bash
# Topics anzeigen (stört nicht)
ros2 topic list

# Nachrichten lesen (stört nicht)
ros2 topic echo /ddd/odom --once
ros2 topic echo /joint_states --once

# Rate messen (stört nicht)
ros2 topic hz /ddd/odom
```

### 3. Bei Problemen: Komplett neu starten
```bash
# Agent beenden
kill $AGENT_PID

# Kurz warten
sleep 2

# Neu starten
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 &
```

## Warum das wichtig ist:

- **Pico erwartet stabile Verbindung**: Jeder Agent-Neustart verwirrt das Pico
- **Session-IDs ändern sich**: Bei Neustart bekommt der Agent neue IDs
- **Publisher werden ungültig**: Alte Publisher-Referenzen funktionieren nicht mehr
- **Timing-Probleme**: Pico und Agent sind nicht mehr synchron

## Test mit unserem Script:

```bash
./proper_test_procedure.sh
```

Das Script macht alles richtig und zeigt dir, ob das System stabil läuft.