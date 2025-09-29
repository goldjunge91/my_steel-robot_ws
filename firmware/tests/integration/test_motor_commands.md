# Test Plan: Motor Control Commands

**Feature**: Motor Control Integration
**Date**: 2025-09-23
**Source Contracts**: `command-contract.md`, `data-model.md`

## Test Objective

Verify that the firmware correctly processes motor control commands and produces the expected physical output.

## Test Environment

- **Hardware**: Raspberry Pi Pico, 4x TB6612FNG motor drivers, 4x DC motors
- **Software**: Serial terminal

## Test Cases

### Test Case 1: Forward Motion

| Test ID | Command Sent | Expected Motor Behavior | Pass/Fail |
|---|---|---|---|
| 1.1 | `M FL:100 FR:100 RL:100 RR:100\n` | All motors turn forward at low speed | |
| 1.2 | `M FL:255 FR:255 RL:255 RR:255\n` | All motors turn forward at full speed | |

### Test Case 2: Backward Motion

| Test ID | Command Sent | Expected Motor Behavior | Pass/Fail |
|---|---|---|---|
| 2.1 | `M FL:-100 FR:-100 RL:-100 RR:-100\n` | All motors turn backward at low speed | |
| 2.2 | `M FL:-255 FR:-255 RL:-255 RR:-255\n` | All motors turn backward at full speed | |

### Test Case 3: Turning Motion

| Test ID | Command Sent | Expected Motor Behavior | Pass/Fail |
|---|---|---|---|
| 3.1 | `M FL:-100 FR:100 RL:-100 RR:100\n` | Left motors backward, right motors forward (turn right) | |
| 3.2 | `M FL:100 FR:-100 RL:100 RR:-100\n` | Left motors forward, right motors backward (turn left) | |

### Test Case 4: Stop

| Test ID | Command Sent | Expected Motor Behavior | Pass/Fail |
|---|---|---|---|
| 4.1 | `STOP\n` | All motors stop | |