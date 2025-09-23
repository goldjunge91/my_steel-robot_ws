# Robot Firmware RP2040 Port Plan

## Goals
- Replace the STM32F4 + PlatformIO stack in `src/robot_firmware` with firmware that targets the Raspberry Pi Pico (RP2040).
- Preserve micro-ROS integrations (motor command subscriber, IMU/motor/battery publishers, `get_cpu_id` service) to remain drop-in compatible with existing ROS 2 bringup.
- Support the new digital board pinout documented in `PINMAP.md` and reuse shared components already proven in `firmware/mecabridge_pico` where possible.

## Toolchain & Build System
- **SDK**: Raspberry Pi Pico SDK with TinyUSB, hardware PWM, PIO, I2C, SPI, watchdog, and FreeRTOS helper libraries enabled through `pico_sdk_init()`.
- **CMake**: CMake-based build identical to `firmware/mecabridge_pico` (top-level `CMakeLists.txt`, `pico_sdk_import.cmake`, `micro_ros_pico_sdk_import.cmake`).
- **Dependencies**:
  - `micro_ros_pico` (USB or UART transport selectable via `MICRO_ROS_USE_UART_TRANSPORT`).
  - Optional `FreeRTOS-Kernel` if we decide to keep RTOS-style scheduling; otherwise, use cooperative loop with rclc executor timers (preferred to reduce complexity).
  - `hardware_pwm`, `hardware_adc`, `hardware_dma`, `hardware_pio` for peripherals.
  - Lightweight BNO055 driver (new implementation against Pico SDK I2C).
  - WS2812/NeoPixel driver built on PIO (use `pico-extras` `ws2812` program or port the PixelLedLib logic).

## Module Decomposition (target structure)

| Module | Responsibility | Porting Strategy |
| --- | --- | --- |
| `hardware_cfg.h` | Compile-time constants for pins, i2c buses, PWM slices, enums mirroring `hardware_cfg.h` | Generate from `PINMAP.md` template; keep naming close to ROS joints. |
| `motors.[h|cpp]` | Manage 4 TB6612-driven motors (PWM + direction) and expose velocity/position APIs | Reuse pattern from `firmware/mecabridge_pico` but extend with PID and encoder feedback. Use hardware PWM slices and encoder capture via PIO IRQ or `quadrature` example. |
| `encoder_reader.[hpp|cpp]` | 4-channel quadrature encoder sampling, integrates tick counts and velocity | Start from `encoder_reader.cpp` in `mecabridge`, refactor to produce velocity and absolute counts needed by micro-ROS message. |
| `ImuLib_cfg.[h|cpp]` | Interface BNO055 over I2C, publish orientation/accel/gyro | Implement using `hardware/i2c` + new helper; consider reusing Adafruit logic translated to C++ without Arduino dependencies. |
| `PixelLedLib_cfg.[h|cpp]` | WS2812 strip animations for status LEDs | Replace `PixelLedLib` with PIO WS2812 driver; provide `IdleAnimation` and direct color setters. |
| `UartLib.[h|cpp]` | Battery/Power board UART protocol | Implement using `hardware/uart` with DMA; replicate frame parsing in `UartLib`. Guard with feature flag (`ENABLE_POWERBOARD`). |
| `micro_ros_cfg.[h|cpp]` | micro-ROS support (entities, callbacks, timers) | Translate logic from `micro_ros_cfg.*` into RAII-style class similar to `MicroRosSupport` but supporting all topics/services. |
| `firmware_main.cpp` | System init, loops | Replace FreeRTOS tasks with cooperative loop using `absolute_time_diff_us` and rclc timers; add watchdog + failsafes. |

## Scheduling & Concurrency
- Execute periodic work via `rclc_executor` timers and `add_subscription` callbacks.
- Offload fast control (motor PID @ 1 kHz) to repeating alarms or second core. Draft approach:
  - Core0 hosts micro-ROS executor.
  - Core1 runs motor control loop using `add_repeating_timer` (1 kHz) updating PID and queueing state snapshots.
- Shared data protected via `critical_section_t` or ring buffers (replace FreeRTOS queues).

## Transport & Communication
- Default micro-ROS transport: USB CDC (TinyUSB). Provide option to use UART to SBC (config macro).
- `watchdog` logic replicates `uRosPingTask`: if agent unreachable, blink LED and force motor stop.
- Keep `_motors_cmd`, `_motors_response`, `_imu/data_raw`, `battery_state`, `get_cpu_id` interface names intact.

## Hardware Mapping Snapshot (from `PINMAP.md`)
- PWM outputs: GP2/GP3/GP4/GP5 (slice 1/2) -> motors; ESCs on GP14/GP15.
- Directions: pairs on GP17..GP22 + GP26/27/28.
- Encoders: GP6..GP13 using PIO state machines (2 per motor).
- I2C (IMU): I2C1 on GP12/GP13 (shared with RL encoder; requires mux or reassign -> TODO resolve conflict).
- LED strip: use GP0 (free) or dedicated pad -> finalize after consulting hardware team.

## Open Questions / Risks
1. **Encoder + IMU pin conflict (GP12/GP13)**: Need hardware confirmation or redesign using alternative pins or dual-purpose with PIO + I2C. Document assumption until clarified.
2. **Power board UART availability**: RP2040 only has two UARTs (0/1). Confirm which to dedicate to SBC vs power board.
3. **FreeRTOS requirement**: Determine whether micro-ROS callbacks demand RTOS; initial plan is bare-metal cooperative scheduling to minimise RAM.
4. **Battery sensing**: choose ADC pin or rely on power board telemetry.
5. **Testing**: set up hardware-in-the-loop harness to validate PWM polarity, encoder direction, BNO055 orientation.

## Immediate Next Steps
1. Generate Pico-specific project skeleton inside `src/robot_firmware/pico/` using Pico SDK imports.
2. Port micro-ROS node scaffolding (`MicrorosNode` class) with stubbed publishers/subscribers.
3. Create hardware abstraction headers with TODOs for each subsystem so incremental porting can begin.
4. Document test plan (unit tests via `pico-test`, bench tests with host) and integrate with `just build-firmware` recipe.


