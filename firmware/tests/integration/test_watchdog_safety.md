# Test Plan: Watchdog Safety

**Feature**: Watchdog Safety Integration
**Date**: 2025-09-23
**Source Contracts**: `research.md`, `data-model.md`

## Test Objective

Verify that the watchdog timer correctly resets the system in case of a software hang.

## Test Environment

- **Hardware**: Raspberry Pi Pico
- **Software**: C/C++ test firmware

## Test Cases

### Test Case 1: Watchdog Reset

| Test ID | Scenario | Expected Behavior | Pass/Fail |
|---|---|---|---|
| 1.1 | The main loop hangs for more than 300ms | The Pico resets | |
| 1.2 | The `watchdog_update()` function is not called | The Pico resets | |

### Test Case 2: Watchdog Recovery

| Test ID | Scenario | Expected Behavior | Pass/Fail |
|---|---|---|---|
| 2.1 | The Pico resets due to the watchdog | The `watchdog_enable_caused_reboot()` function returns true | |
| 2.2 | The Pico resets due to the watchdog | All motors are stopped and all systems are in a safe state | |