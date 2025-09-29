# Test Plan: USB CDC Protocol Contract

**Feature**: Pico Firmware Communication Interface
**Date**: 2025-09-23
**Source Contract**: `specs/001-description-build-firmware/contracts/usb-cdc-protocol.md`

## Test Objective

Verify that the firmware's USB CDC implementation correctly handles all commands and responses as defined in the protocol contract.

## Test Environment

- **Hardware**: Raspberry Pi Pico, USB cable
- **Software**: Serial terminal (e.g., PuTTY, screen, Arduino IDE Serial Monitor)

## Test Cases

### Test Case 1: Motor Control Commands

| Test ID | Command Sent | Expected Response | Pass/Fail |
|---|---|---|---|
| 1.1 | `M FL:100 FR:100 RL:100 RR:100\n` | `OK\n` | |
| 1.2 | `M FL:-255 FR:255 RL:-128 RR:128\n` | `OK\n` | |
| 1.3 | `M FL:300 FR:100 RL:100 RR:100\n` | `ERROR: INVALID_PARAM FL speed out of range\n` | |
| 1.4 | `STOP\n` | `STOPPED\n` | |

### Test Case 2: Shooter Control Commands

| Test ID | Command Sent | Expected Response | Pass/Fail |
|---|---|---|---|
| 2.1 | `FIRE\n` | `FIRING\n` | |
| 2.2 | `ESC ESC1:1 ESC2:0\n` | `OK\n` | |
| 2.3 | `GEAR:1\n` | `OK\n` | |
| 2.4 | `GEAR:0\n` | `OK\n` | |

### Test Case 3: System Commands

| Test ID | Command Sent | Expected Response | Pass/Fail |
|---|---|---|---|
| 3.1 | `STATUS\n` | (Multi-line status report) | |
| 3.2 | `VERSION\n` | `VERSION lpico-1.0.0-001-description-build-firmware\n` | |

### Test Case 4: Error Handling

| Test ID | Command Sent | Expected Response | Pass/Fail |
|---|---|---|---|
| 4.1 | `INVALIDCMD\n` | `ERROR: INVALID_CMD Command not recognized\n` | |
| 4.2 | `M FL:100\n` | `ERROR: INVALID_PARAM Missing parameters\n` | |