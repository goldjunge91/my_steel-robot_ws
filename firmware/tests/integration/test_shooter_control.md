# Test Plan: Shooter Control Integration

**Feature**: Shooter Control Integration
**Date**: 2025-09-23
**Source Contracts**: `command-contract.md`, `data-model.md`

## Test Objective

Verify that the firmware correctly processes shooter control commands and produces the expected physical output.

## Test Environment

- **Hardware**: Raspberry Pi Pico, 2x ESC modules, 1x Servo motor
- **Software**: Serial terminal

## Test Cases

### Test Case 1: Fire Sequence

| Test ID | Command Sent | Expected Behavior | Pass/Fail |
|---|---|---|---|
| 1.1 | `FIRE\n` | ESCs activate in sequence, then deactivate | |

### Test Case 2: ESC Control

| Test ID | Command Sent | Expected Behavior | Pass/Fail |
|---|---|---|---|
| 2.1 | `ESC ESC1:1 ESC2:0\n` | ESC1 activates, ESC2 deactivates | |
| 2.2 | `ESC ESC1:0 ESC2:1\n` | ESC1 deactivates, ESC2 activates | |
| 2.3 | `ESC ESC1:1 ESC2:1\n` | Both ESCs activate | |
| 2.4 | `ESC ESC1:0 ESC2:0\n` | Both ESCs deactivate | |

### Test Case 3: Gear Control

| Test ID | Command Sent | Expected Behavior | Pass/Fail |
|---|---|---|---|
| 3.1 | `GEAR:1\n` | Servo motor moves to extended position | |
| 3.2 | `GEAR:0\n` | Servo motor moves to retracted position | |