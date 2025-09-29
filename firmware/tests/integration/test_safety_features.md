# Integration Test: Safety Features

**Feature**: Safety Features Integration
**Date**: 2025-09-25
**Source**: research.md, contracts/micro-ros-contracts.md

## Test Objective

Verify that all safety features work correctly: 200ms safety timeout, critical battery reporting, safe stop on connection loss, and watchdog functionality.

## Test Environment

- **Hardware**: Raspberry Pi Pico with motors, battery monitor
- **Software**: ROS2 Humble, micro-ROS agent, firmware flashed to Pico
- **Prerequisites**: Firmware built and flashed, micro-ROS agent running

## Test Cases

### Test Case 1: Safety Timeout (200ms)

| Test ID | Test Action                                    | Expected Behavior         | Verification            |
| ------- | ---------------------------------------------- | ------------------------- | ----------------------- |
| SF-1.1  | Send motor command, wait 150ms, send another   | Motors continue running   | Command renewal works   |
| SF-1.2  | Send motor command, wait 250ms without renewal | Motors stop automatically | Safety timeout triggers |
| SF-1.3  | Send zero velocity command                     | Motors stop immediately   | Manual stop works       |

### Test Case 2: Critical Battery Reporting

| Test ID | Test Action                                 | Expected Behavior                    | Verification             |
| ------- | ------------------------------------------- | ------------------------------------ | ------------------------ |
| SF-2.1  | Monitor /battery_state when voltage > 10.5V | Normal operation, no special logging | Battery monitoring works |
| SF-2.2  | Simulate voltage < 10.5V                    | Critical battery warning published   | Firmware logs warning    |
| SF-2.3  | Check /battery_state message structure      | All required fields present          | Message compliance       |

### Test Case 3: Connection Loss Safety

| Test ID | Test Action                                         | Expected Behavior        | Verification              |
| ------- | --------------------------------------------------- | ------------------------ | ------------------------- |
| SF-3.1  | Send motor command, then disconnect micro-ROS agent | Motors stop within 200ms | Connection loss detection |
| SF-3.2  | Reconnect agent, send command                       | Motors respond normally  | Recovery works            |
| SF-3.3  | Monitor watchdog during connection loss             | No watchdog reset occurs | Proper safety shutdown    |

### Test Case 4: Watchdog Functionality

| Test ID | Test Action                                         | Expected Behavior              | Verification          |
| ------- | --------------------------------------------------- | ------------------------------ | --------------------- |
| SF-4.1  | Force main loop hang (> watchdog timeout)           | System resets via watchdog     | Watchdog triggers     |
| SF-4.2  | Check `watchdog_enable_caused_reboot()` after reset | Returns true                   | Reset cause detection |
| SF-4.3  | After watchdog reset                                | All motors stopped, safe state | Safe reset behavior   |

### Test Case 5: Safety Interlocks

| Test ID | Test Action                                     | Expected Behavior               | Verification           |
| ------- | ----------------------------------------------- | ------------------------------- | ---------------------- |
| SF-5.1  | Send launcher fire command without proper setup | Command rejected                | Safety interlocks work |
| SF-5.2  | Send out-of-range motor velocities              | Commands clamped to safe limits | Input validation       |
| SF-5.3  | Send invalid JointState messages                | Messages ignored                | Error handling         |

## Safety Requirements Validation

- **Timeout**: Motors stop within 200ms of command expiration
- **Battery**: Critical warnings when voltage < 10.5V
- **Connection Loss**: Motors stop within 200ms of agent disconnection
- **Watchdog**: System resets on software hangs
- **Interlocks**: Invalid commands are safely rejected

## Emergency Procedures

- **Motor Runaway**: Disconnect power immediately
- **System Hang**: Watchdog will reset within timeout period
- **Battery Critical**: System should warn but continue operating

## Pass Criteria

- All safety timeouts work within specified limits
- Critical battery levels trigger appropriate warnings
- Connection loss results in immediate safe stop
- Watchdog properly resets hung system
- Safety interlocks prevent dangerous operations