# Test Plan: HAL Interface Compatibility

**Feature**: STM32 to Pico Compatibility Layer
**Date**: 2025-09-23
**Source Contract**: `specs/001-description-build-firmware/contracts/hal-interface.md`

## Test Objective

Verify that the Pico HAL implementation provides the same functionality as the STM32 HAL, as defined in the contract.

## Test Environment

- **Hardware**: Raspberry Pi Pico, Logic Analyzer
- **Software**: C/C++ test firmware

## Test Cases

### Test Case 1: GPIO Interface

| Test ID | Function Call | Expected Behavior | Pass/Fail |
|---|---|---|---|
| 1.1 | `HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)` | GPIO 0 goes high | |
| 1.2 | `HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)` | GPIO 0 goes low | |
| 1.3 | `HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1)` | GPIO 1 toggles its state | |
| 1.4 | `HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)` | Returns the state of GPIO 2 | |

### Test Case 2: PWM Interface

| Test ID | Function Call | Expected Behavior | Pass/Fail |
|---|---|---|---|
| 2.1 | `HAL_TIM_PWM_Start(&htim_fl, MOTOR_FL_PWM_PIN)` | PWM signal starts on GPIO 2 | |
| 2.2 | `__HAL_TIM_SET_COMPARE(&htim_fl, MOTOR_FL_PWM_PIN, 128)` | PWM duty cycle on GPIO 2 is 50% | |
| 2.3 | `HAL_TIM_PWM_Stop(&htim_fl, MOTOR_FL_PWM_PIN)` | PWM signal stops on GPIO 2 | |

### Test Case 3: System Functions

| Test ID | Function Call | Expected Behavior | Pass/Fail |
|---|---|---|---|
| 3.1 | `HAL_Delay(100)` | Program execution pauses for 100ms | |
| 3.2 | `HAL_GetTick()` | Returns the system uptime in ms | |

### Test Case 4: Initialization Functions

| Test ID | Function Call | Expected Behavior | Pass/Fail |
|---|---|---|---|
| 4.1 | `SystemClock_Config()` | System clock is initialized | |
| 4.2 | `MX_GPIO_Init()` | All GPIOs are initialized | |
| 4.3 | `MX_TIM_Init()` | All PWM channels are initialized | |