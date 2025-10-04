### **Session starten/verbinden:**
```bash
# Deine Robot-Simulation starten
./start_sim_tmux.sh

# Zu laufender Session verbinden
tmux attach -t robot_sim

# Oder kurz:
tmux a -t robot_sim
```

### **Navigation zwischen Fenstern:**
```bash
Ctrl+b, 0    # Zu Fenster 0 (main - Gazebo)
Ctrl+b, 1    # Zu Fenster 1 (control - Teleop)
Ctrl+b, 2    # Zu Fenster 2 (monitor - Debug)

Ctrl+b, n    # Nächstes Fenster
Ctrl+b, p    # Vorheriges Fenster
Ctrl+b, l    # Letztes Fenster
```

### **Session Management:**
```bash
Ctrl+b, d    # Session verlassen (läuft weiter im Hintergrund)
tmux ls      # Alle Sessions anzeigen
tmux kill-session -t robot_sim  # Session beenden
```

## 🎮 **Deine Robot-Session verwenden:** 

Ran terminal command: tmux ls