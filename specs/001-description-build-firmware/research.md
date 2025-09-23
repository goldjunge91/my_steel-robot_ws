# Research: Pico SDK Implementation for STM32 to Pico Firmware Port

**Feature**: Port legacy STM32 firmware to Raspberry Pi Pico (lpico)  
**Date**: 2025-09-23  
**Research Phase**: Phase 0 of implementation plan

## Research Topics & Findings

### 1. Pico SDK PWM APIs for TB6612 Motor Driver Control

**Decision**: Use Pico SDK `hardware/pwm.h` APIs with dedicated PWM channels for each motor

**Rationale**: 
- TB6612 requires dedicated PWM signals for speed control (4 motors = 4 PWM channels)
- Pico SDK provides 8 independent PWM channels (sufficient for 4 motors + shooter ESC)
- PWM frequency can be configured independently per channel
- Hardware PWM ensures consistent timing and minimal jitter

**Implementation Approach**:
```c
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// Set up PWM for each motor
uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
pwm_set_wrap(slice_num, 255);  // 8-bit resolution for compatibility
pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
pwm_set_enabled(slice_num, true);
```

**Alternatives Considered**:
- PIO (Programmable I/O) state machines: More complex, unnecessary for basic PWM
- Software PWM via timers: Less accurate, higher CPU overhead

### 2. USB CDC Implementation on RP2040 for Command Interface

**Decision**: Use Pico SDK's built-in TinyUSB CDC support with stdio functions

**Rationale**:
- Simple integration: `pico_enable_stdio_usb(target 1)` in CMakeLists.txt
- Standard C stdio functions (printf, scanf, gets) work automatically
- No manual TinyUSB configuration required
- Compatible with existing STM32 command parsing logic
- Latency <10ms achievable for simple command protocols

**Implementation Approach**:
```c
#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();  // Initialize USB CDC
    
    // Use printf/scanf for communication
    printf("Command ready\n");
    
    char command[64];
    if (gets(command) != NULL) {
        // Parse command
    }
}
```

**Alternatives Considered**:
- Custom TinyUSB implementation: More complex, unnecessary overhead
- UART with USB-TTL converter: Requires additional hardware
- Micro-ROS over USB: Overkill for simple motor commands, but future option

### 3. STM32 to Pico SDK Porting Strategies and HAL Equivalents

**Decision**: Create compatibility header mapping STM32 HAL calls to Pico SDK equivalents

**Rationale**:
- Minimizes changes to existing logic and algorithms
- Preserves exact function names as required by constitution
- Enables gradual porting with clear abstraction layer
- Maintains readability for developers familiar with STM32 HAL

**Key Mappings**:
```c
// hal_compat.h - Compatibility layer
#define HAL_GPIO_WritePin(port, pin, state) gpio_put(pin, state)
#define HAL_GPIO_ReadPin(port, pin) gpio_get(pin)
#define HAL_PWM_SetDutyCycle(channel, duty) pwm_set_gpio_level(channel, duty)
#define HAL_Delay(ms) sleep_ms(ms)

// Timer/PWM mappings
#define __HAL_TIM_SET_COMPARE(htim, channel, value) pwm_set_gpio_level(channel, value)
```

**Alternatives Considered**:
- Complete rewrite: Violates constitution requirement for exact filenames/APIs
- Direct SDK calls throughout: Makes code less portable and readable

### 4. RP2040 Watchdog Timer Configuration and Best Practices

**Decision**: Use hardware watchdog with 300ms timeout as specified in requirements

**Rationale**:
- Hardware watchdog provides reliable reset mechanism independent of software state
- 300ms timeout allows for normal control loop operation while catching hangs
- Watchdog enables checking previous reboot cause for diagnostics
- Simple API: `watchdog_enable()` and `watchdog_update()`

**Implementation Approach**:
```c
#include "hardware/watchdog.h"

void setup_watchdog() {
    // Enable with 300ms timeout, pause on debug
    watchdog_enable(300, 1);
}

void feed_watchdog() {
    watchdog_update();  // Call this in main control loop
}

bool was_watchdog_reset() {
    return watchdog_enable_caused_reboot();
}
```

**Safety Considerations**:
- Maximum timeout on RP2040 is ~8.3 seconds (sufficient for our 300ms requirement)
- Once enabled, watchdog cannot be disabled (intentional safety feature)
- Should be fed in main control loop, not in interrupt handlers

**Alternatives Considered**:
- Software watchdog via timer: Less reliable, can be disabled by software faults
- External watchdog IC: Unnecessary complexity for this application

### 5. Real-Time Motor Control Loop Design on Single-Core RP2040

**Decision**: Use timer-based control loop with non-blocking USB CDC handling

**Rationale**:
- RP2040 running at 133MHz provides sufficient performance for 1kHz motor control
- Timer interrupts ensure consistent control loop timing
- USB CDC handled in main loop to avoid blocking control loop
- FreeRTOS not required for this simple application

**Implementation Approach**:
```c
#include "hardware/timer.h"

// Control loop timer callback
bool control_loop_callback(struct repeating_timer *t) {
    // Update PWM outputs (non-blocking)
    // Read encoders (non-blocking)
    // Feed watchdog
    // Total execution time < 200μs for 1kHz loop
    return true;  // Continue repeating
}

int main() {
    struct repeating_timer timer;
    
    // 1kHz control loop (1000μs period)
    add_repeating_timer_us(1000, control_loop_callback, NULL, &timer);
    
    while (1) {
        // Handle USB CDC commands (can block)
        // Update non-critical tasks
    }
}
```

**Performance Budget**:
- Control loop frequency: 1kHz (1000μs period)
- Control loop execution time budget: <200μs (20% CPU usage)
- USB CDC command processing: <10ms latency acceptable
- Remaining CPU time: Available for diagnostics, logging

**Alternatives Considered**:
- FreeRTOS with tasks: Unnecessary complexity for single control loop
- Polling-based timing: Less accurate, subject to USB CDC delays
- Dual-core implementation: Unnecessary for current requirements

### 6. Micro-ROS Integration Considerations (Future Enhancement)

**Research Finding**: Micro-ROS PlatformIO library supports Raspberry Pi Pico

**Key Points**:
- Pico supported with `colcon.meta` configuration
- Serial transport over USB CDC already supported
- ROS 2 distributions: Humble, Jazzy, Rolling supported
- Can be added later without breaking existing command interface
- Memory requirements manageable with `colcon.meta` tuning

**Future Integration Path**:
1. Implement basic USB CDC command interface first
2. Add micro-ROS as library dependency in PlatformIO
3. Implement ROS 2 topics alongside existing command interface
4. Gradually migrate higher-level control to ROS 2 messages

## Summary of Technical Decisions

| Aspect        | Technology Choice         | Rationale                                  |
| ------------- | ------------------------- | ------------------------------------------ |
| PWM Control   | Pico SDK hardware/pwm.h   | Hardware timing, 8 channels available      |
| Communication | USB CDC via stdio         | Simple, low-latency, standard interface    |
| HAL Layer     | Compatibility headers     | Preserves STM32 API names, gradual porting |
| Watchdog      | Hardware watchdog (300ms) | Reliable reset, meets safety requirements  |
| Control Loop  | Timer interrupt (1kHz)    | Deterministic timing, non-blocking         |
| Build System  | CMake with Pico SDK       | Standard toolchain, good documentation     |

## Key Resources Identified

- **Existing Pico Infrastructure**: `src/robot_firmware/pico/` contains CMakeLists.txt and SDK imports
- **Source Reference**: `src/robot_firmware/legacy_stm32/` provides API contracts and filenames
- **Hardware Configuration**: Pin mappings defined in FR-007 of feature spec
- **Communication Protocol**: USB CDC command interface per FR-008

## Next Steps for Phase 1

1. Create data model for motor control structures and command formats
2. Define USB CDC command protocol contracts
3. Design HAL compatibility layer interface
4. Plan test procedures for each functional requirement
5. Update GitHub Copilot instructions with embedded systems context

---
*Research completed for Phase 0 of implementation plan*
