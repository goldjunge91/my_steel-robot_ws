---
title: tmux Usage Guide
summary: Terminal multiplexer guide for robot workflows
description: Quick reference for using tmux to manage multiple terminal sessions for robot development and operation
keywords: tmux, terminal, multiplexer, workflow
author: goldjunge91
order: 7
---

# tmux Usage Guide

!!! info "What is tmux?"
    tmux ist ein Terminal-Multiplexer, der es ermöglicht, mehrere Terminal-Sessions in einem Fenster zu verwalten. Perfekt für die Roboter-Entwicklung!

## Session starten/verbinden

/// details | Session Management
**Deine Robot-Simulation starten:**

    :::bash
    ./start_sim_tmux.sh

**Zu laufender Session verbinden:**

    :::bash
    tmux attach -t robot_sim
    # Oder kurz:
    tmux a -t robot_sim

**Alle Sessions anzeigen:**

    :::bash
    tmux ls
///

## Navigation zwischen Fenstern

!!! note "Tastenkombinationen"
    Alle Befehle beginnen mit dem Präfix `Ctrl+b`, gefolgt von einem weiteren Tastendruck:

/// details | Fenster-Navigation

    :::bash
    Ctrl+b, 0    # Zu Fenster 0 (main - Gazebo)
    Ctrl+b, 1    # Zu Fenster 1 (control - Teleop)
    Ctrl+b, 2    # Zu Fenster 2 (monitor - Debug)
    
    Ctrl+b, n    # Nächstes Fenster
    Ctrl+b, p    # Vorheriges Fenster
    Ctrl+b, l    # Letztes Fenster
///

## Session Management

/// details | Erweiterte Befehle

    :::bash
    Ctrl+b, d    # Session verlassen (läuft weiter im Hintergrund)
    tmux ls      # Alle Sessions anzeigen
    tmux kill-session -t robot_sim  # Session beenden
///

## +lucide:gamepad-2+ Deine Robot-Session verwenden


Searched for text `--bash` (`**//home/marco/.bashrc`), no results

Ran terminal command:  grep -n "fzf.*--bash" ~/.bashrc

Checked background terminal output

Ran terminal command:  cat ~/.bashrc | head -90 | tail -10

Der "unknown option: --bash" Fehler kommt wahrscheinlich vom fzf-Plugin. Lass mich dir erstmal zeigen, wie du **tmux** benutzt:

## 🖥️ tmux Schnellstart

### Grundlegende Befehle:

**Session starten:**
```bash
tmux                    # Neue Session
tmux new -s mysession   # Session mit Namen
```

**Session Management:**
```bash
tmux ls                 # Alle Sessions anzeigen
tmux attach -t 0        # An Session 0 anhängen
tmux attach -t mysession # An benannte Session
tmux kill-session -t 0  # Session beenden
```

### Wichtige Tastenkombinationen (Prefix: `Ctrl+b`):

**Windows (Tabs):**
- `Ctrl+b c` - Neues Window erstellen
- `Ctrl+b n` - Nächstes Window
- `Ctrl+b p` - Vorheriges Window
- `Ctrl+b 0-9` - Zu Window-Nummer springen
- `Ctrl+b ,` - Window umbenennen

**Panes (Split-Screen):**
- `Ctrl+b %` - Vertikaler Split
- `Ctrl+b "` - Horizontaler Split
- `Ctrl+b o` - Zwischen Panes wechseln
- `Ctrl+b x` - Pane schließen
- `Ctrl+b Pfeiltasten` - Pane-Navigation

**Session:**
- `Ctrl+b d` - Session detachen (läuft weiter im Hintergrund)
- `Ctrl+b [` - Scroll-Modus (mit `q` beenden)

**Für ROS2 besonders nützlich:**
```bash
# Multi-Pane Setup für ROS2
tmux new -s ros2
# Im tmux:
Ctrl+b "    # Horizontal split
Ctrl+b %    # Vertical split

# Jetzt kannst du in jedem Pane:
# Pane 1: ros2 launch ...
# Pane 2: ros2 topic echo ...
# Pane 3: ros2 node list
```

Der **tmux-autoattach** Plugin in deiner .bashrc sorgt dafür, dass du automatisch an eine laufende Session angehängt wirst, wenn du ein neues Terminal öffnest.

Probier's aus mit: `tmux new -s ros2` 🚀