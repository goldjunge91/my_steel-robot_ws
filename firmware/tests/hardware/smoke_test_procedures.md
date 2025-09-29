# Smoke Test Procedures

**Feature**: Hardware-in-loop smoke testing
**Date**: 2025-09-23

## Test Objective

Perform a quick smoke test to ensure the basic functionality of the firmware is working as expected.

## Test Environment

- **Hardware**: Fully assembled robot with Raspberry Pi Pico, motors, shooters, and all other peripherals.
- **Software**: Serial terminal.

## Test Procedures

| Test ID | Action                                         | Expected Result                                             | Pass/Fail |
| ------- | ---------------------------------------------- | ----------------------------------------------------------- | --------- |
| 1       | Power on the robot                             | The robot powers on and the status LED blinks               |           |
| 2       | Connect to the robot via serial terminal       | The serial terminal connects successfully                   |           |
| 3       | Send the `STATUS` command                      | The robot returns a status message with all systems nominal |           |
| 4       | Send the `M FL:50 FR:50 RL:50 RR:50\n` command | All motors turn forward at low speed                        |           |
| 5       | Send the `STOP\n` command                      | All motors stop                                             |           |
| 6       | Send the `FIRE\n` command                      | The shooter fires                                           |           |
| 7       | Disconnect the serial terminal for 5 seconds   | The robot stops all motors (watchdog)                       |           |