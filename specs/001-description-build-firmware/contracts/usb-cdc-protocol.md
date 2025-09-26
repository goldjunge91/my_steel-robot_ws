# USB CDC Command Protocol Contract

**Feature**: Pico Firmware Communication Interface  
**Date**: 2025-09-23  
**Version**: 1.0  
**Transport**: USB CDC Serial over USB 2.0

## Protocol Overview

The Raspberry Pi Pico firmware exposes a line-based ASCII command protocol over USB CDC. Commands are newline-terminated strings, responses are immediate.

## Command Format

```
<COMMAND> [<PARAM1>:<VALUE1>] [<PARAM2>:<VALUE2>] ...\n
```

- Commands are case-sensitive
- Parameters are colon-separated key:value pairs
- Multiple parameters separated by spaces
- Lines terminated with `\n` (newline)
- Maximum command length: 128 bytes

## Commands Specification

### 1. Motor Control Commands

#### Set Motor Speeds
```
Command: M FL:<speed> FR:<speed> RL:<speed> RR:<speed>
Parameters:
  - FL: Front-left motor speed (-255 to +255)
  - FR: Front-right motor speed (-255 to +255)  
  - RL: Rear-left motor speed (-255 to +255)
  - RR: Rear-right motor speed (-255 to +255)

Response: OK\n
Error Response: ERROR: <reason>\n

Example:
> M FL:100 FR:100 RL:100 RR:100
< OK
```

#### Emergency Stop
```
Command: STOP
Parameters: None

Response: STOPPED\n

Example:
> STOP
< STOPPED
```

### 2. Shooter Control Commands

#### Fire Sequence
```
Command: FIRE
Parameters: None

Response: FIRING\n
Error Response: ERROR: <reason>\n

Example:
> FIRE
< FIRING
```

#### ESC Control
```
Command: ESC ESC1:<state> ESC2:<state>
Parameters:
  - ESC1: ESC1 enable state (0=disabled, 1=enabled)
  - ESC2: ESC2 enable state (0=disabled, 1=enabled)

Response: OK\n
Error Response: ERROR: <reason>\n

Example:
> ESC ESC1:1 ESC2:0
< OK
```

#### Gear Position
```
Command: GEAR:<position>
Parameters:
  - position: Gear state (0=retracted, 1=extended)

Response: OK\n
Error Response: ERROR: <reason>\n

Example:
> GEAR:1
< OK
```

### 3. System Commands

#### Status Query
```
Command: STATUS
Parameters: None

Response: Multi-line status report
FORMAT:
MOTORS FL:<speed> FR:<speed> RL:<speed> RR:<speed>
SHOOTER ESC1:<state> ESC2:<state> GEAR:<pos>
SYSTEM UPTIME:<ms> WATCHDOG:<state> FREQ:<hz>
READY\n

Example:
> STATUS
< MOTORS FL:100 FR:100 RL:100 RR:100
< SHOOTER ESC1:1 ESC2:0 GEAR:1
< SYSTEM UPTIME:45230 WATCHDOG:0 FREQ:1000
< READY
```

#### Version Information
```
Command: VERSION
Parameters: None

Response: VERSION <version_string>\n

Example:
> VERSION
< VERSION lpico-1.0.0-001-description-build-firmware
```

## Error Handling

### Error Response Format
```
ERROR: <error_code> <description>\n
```

### Error Codes
- `INVALID_CMD`: Command not recognized
- `INVALID_PARAM`: Parameter value out of range
- `SAFETY_LOCK`: Safety system preventing operation
- `TIMEOUT`: Command processing timeout
- `HARDWARE_FAULT`: Hardware malfunction detected

### Examples
```
> M FL:300 FR:100 RL:100 RR:100
< ERROR: INVALID_PARAM FL speed out of range

> FIRE
< ERROR: SAFETY_LOCK Cooldown period active

> INVALIDCMD
< ERROR: INVALID_CMD Command not recognized
```

## Safety Constraints

### Command Rate Limiting
- Motor commands: Max 100 Hz
- Fire commands: Max 10 Hz (100ms cooldown)
- Status queries: Max 10 Hz

### Watchdog Behavior
- If no command received for 300ms, all motors stop
- Watchdog reset causes all systems to safe state
- Fire sequence automatically aborts on timeout

### Parameter Validation
- Motor speeds clamped to [-255, +255]
- Boolean parameters accept only 0 or 1
- Invalid parameters rejected with error response

## Communication Timing

### Timeouts
- Command processing: <10ms typical
- Response transmission: <5ms
- Status query response: <20ms

### Baud Rate
- USB CDC virtual COM port (baud rate setting ignored)
- Actual throughput: ~1MB/s theoretical, ~100KB/s practical

## Protocol State Machine

```
IDLE ──[valid command]──> PROCESSING ──[response sent]──> IDLE
  │                            │
  └─[invalid command]─────> ERROR ──[error sent]──> IDLE
```

## Compatibility Notes

### Legacy STM32 Compatibility
- Command syntax identical to legacy firmware
- Response formats preserved for existing scripts
- Pin assignments may differ (transparent to protocol)

### Future Extensions
- Protocol designed for extension with new commands
- Versioning support for backward compatibility
- Reserved command prefixes: `DEBUG`, `DIAG`, `CFG`

---
*USB CDC Protocol Contract v1.0*