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

!!! tip "Workflow-Tipp"
    Starte die Simulation mit `./start_sim_tmux.sh` und wechsle dann zwischen den Fenstern, um verschiedene Komponenten zu überwachen!