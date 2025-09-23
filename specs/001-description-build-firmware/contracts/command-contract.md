# USB CDC Command Protocol Contract (4-Wheel)

**Feature**: Pico Firmware Communication Interface
**Date**: 2025-09-23  
**Version**: 1.0 (Updated for 4-wheel consistency)

Commands are ASCII, newline-terminated. Parser ignores leading/trailing whitespace.

## Motor Control Commands

### Set Motor Speeds
```
Format: M FL:<speed> FR:<speed> RL:<speed> RR:<speed>\n
Parameters:
  - FL: Front-left motor speed (-255 to +255)
  - FR: Front-right motor speed (-255 to +255)  
  - RL: Rear-left motor speed (-255 to +255)
  - RR: Rear-right motor speed (-255 to +255)

Response: OK\n
Example: M FL:100 FR:100 RL:100 RR:100\n
```

### Emergency Stop
```
Format: STOP\n
Response: STOPPED\n
Example: STOP\n
```

## Shooter Control Commands

### Fire Sequence
```
Format: FIRE\n
Response: FIRING\n
Example: FIRE\n
```

### ESC Control
```
Format: ESC ESC1:<state> ESC2:<state>\n
Parameters:
  - ESC1: ESC1 enable state (0=disabled, 1=enabled)
  - ESC2: ESC2 enable state (0=disabled, 1=enabled)

Response: OK\n
Example: ESC ESC1:1 ESC2:0\n
```

### Gear Position
```
Format: GEAR:<position>\n
Parameters:
  - position: Gear state (0=retracted, 1=extended)

Response: OK\n
Example: GEAR:1\n
```

## Status Query
```
Format: STATUS\n
Response: MOTORS FL:<speed> FR:<speed> RL:<speed> RR:<speed>\n
          SHOOTER ESC1:<state> ESC2:<state> GEAR:<pos>\n
          SYSTEM UPTIME:<ms> WATCHDOG:<triggered> FREQ:<hz>\n

Example: STATUS\n
```

## Notes
- Commands must be acknowledged by the Pico with a response for debug integration
- All motor speeds clamped to [-255, +255] range
- Watchdog resets motors if no command received within 300ms
