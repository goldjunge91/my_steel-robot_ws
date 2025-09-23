# Tasks: Port legacy STM32 firmware to Raspberry Pi Pico (lpico)

**Input**: Design documents from `/specs/001-description-build-firmware/`
**Prerequisites**: plan.md (required), research.md, data-model.md, contracts/

## Execution Flow (main)
```
1. Load plan.md from feature directory
   → Found: tech stack (C/C++, Pico SDK), embedded firmware project
   → Extract: CMake build system, USB CDC interface, motor control
2. Load available design documents:
   → data-model.md: Motor control structures, shooter state, pin config
   → contracts/: usb-cdc-protocol.md, hal-interface.md, command-contract.md
   → research.md: PWM APIs, USB CDC, HAL porting, watchdog config
   → quickstart.md: Build/test procedures, hardware validation
3. Generate tasks by category:
   → Setup: Pico SDK project, CMake configuration, dependencies
   → Tests: Contract tests, hardware-in-loop tests, smoke tests
   → Core: HAL layer, data models, USB CDC interface, motor control
   → Integration: Watchdog, control loop, command processing
   → Polish: Hardware tests, documentation, optimization
4. Apply task rules:
   → Different files = mark [P] for parallel
   → Same file = sequential (no [P])
   → Tests before implementation (TDD)
5. Number tasks sequentially (T001, T002...)
6. Dependencies: HAL before drivers, drivers before control loop
7. Parallel execution: Contract tests, separate firmware modules
8. Validation: All contracts tested, hardware safety validated
```

## Format: `[ID] [P?] Description`
- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in firmware directory structure

## Path Conventions
- **Firmware project**: `src/robot_firmware/lpico/` (embedded C/C++)
- **Tests**: Manual hardware tests and smoke test procedures
- **Contracts**: Communication protocols and HAL interface specs

## Phase 3.1: Setup

- [x] T001 Create lpico project structure in `src/robot_firmware/lpico/`
- [x] T002 Initialize CMake project with Pico SDK dependencies in `src/robot_firmware/lpico/CMakeLists.txt`
- [ ] T003 [P] Configure build system with proper compiler flags and TinyUSB in CMake
- [ ] T004 [P] Create pin definitions header `src/robot_firmware/lpico/include/pin_config.h`
- [x] T005 [P] Copy and organize legacy STM32 source files preserving exact filenames

## Phase 3.2: Tests First (TDD) ⚠️ MUST COMPLETE BEFORE 3.3

**CRITICAL: These tests MUST be written and MUST FAIL before ANY implementation**

- [ ] T006 [P] USB CDC protocol contract test procedures in `tests/contract/test_usb_cdc_protocol.md`
- [ ] T007 [P] HAL interface compatibility test procedures in `tests/contract/test_hal_interface.md`
- [ ] T008 [P] Motor control command test procedures in `tests/integration/test_motor_commands.md`
- [ ] T009 [P] Shooter control integration test procedures in `tests/integration/test_shooter_control.md`
- [ ] T010 [P] Watchdog safety test procedures in `tests/integration/test_watchdog_safety.md`
- [ ] T011 [P] Hardware-in-loop smoke test procedures in `tests/hardware/smoke_test_procedures.md`

## Phase 3.3: Core Implementation (ONLY after tests are failing)

- [ ] T012 [P] HAL compatibility layer implementation in `src/robot_firmware/lpico/src/hal_compat.c`
- [ ] T013 [P] Motor control data structures in `src/robot_firmware/lpico/include/motor_types.h`
- [ ] T014 [P] Shooter control data structures in `src/robot_firmware/lpico/include/shooter_types.h`
- [ ] T015 [P] System configuration structures in `src/robot_firmware/lpico/include/system_config.h`
- [ ] T016 USB CDC communication interface in `src/robot_firmware/lpico/src/usb_interface.c`
- [ ] T017 TB6612 motor driver interface in `src/robot_firmware/lpico/src/motor_driver.c`
- [ ] T018 PWM control implementation in `src/robot_firmware/lpico/src/pwm_control.c`
- [ ] T019 Encoder reading implementation in `src/robot_firmware/lpico/src/encoder_reader.c`
- [ ] T020 Shooter ESC control in `src/robot_firmware/lpico/src/shooter_control.c`

## Phase 3.4: Integration

- [ ] T021 Hardware watchdog timer setup in `src/robot_firmware/lpico/src/watchdog_manager.c`
- [ ] T022 Real-time control loop implementation in `src/robot_firmware/lpico/src/control_loop.c`
- [ ] T023 Command parser and dispatcher in `src/robot_firmware/lpico/src/command_processor.c`
- [ ] T024 Main firmware application in `src/robot_firmware/lpico/src/main.c`
- [ ] T025 Interrupt handlers and timing in `src/robot_firmware/lpico/src/interrupts.c`
- [ ] T026 Safety systems integration (emergency stop, output limits)

## Phase 3.5: Polish

- [ ] T027 [P] Hardware validation with actual TB6612 drivers following smoke test procedures
- [ ] T028 [P] Performance optimization for 1kHz control loop timing
- [ ] T029 [P] Update firmware build documentation in `src/robot_firmware/lpico/README.md`
- [ ] T030 [P] Code cleanup and remove debug artifacts
- [ ] T031 [P] Final constitution compliance verification (exact filenames, safety features)
- [ ] T032 Execute complete quickstart validation procedures from specs/quickstart.md

## Dependencies

- Setup (T001-T005) before everything
- Test procedures (T006-T011) before implementation (T012-T026)
- HAL layer (T012) before all drivers (T017-T020)
- Data structures (T013-T015) before implementation tasks
- USB interface (T016) before command processing (T023)
- Motor drivers (T017-T018) before control loop (T022)
- Core components before integration (T021-T026)
- Integration before polish (T027-T032)

## Parallel Example

```bash
# Launch contract tests together (Phase 3.2):
Task: "USB CDC protocol contract test procedures in tests/contract/test_usb_cdc_protocol.md"
Task: "HAL interface compatibility test procedures in tests/contract/test_hal_interface.md"
Task: "Motor control command test procedures in tests/integration/test_motor_commands.md"

# Launch data structure headers together (Phase 3.3):
Task: "Motor control data structures in src/robot_firmware/lpico/include/motor_types.h"
Task: "Shooter control data structures in src/robot_firmware/lpico/include/shooter_types.h"
Task: "System configuration structures in src/robot_firmware/lpico/include/system_config.h"
```

## Notes

- [P] tasks target different files and have no dependencies
- Hardware tests require physical Pico and TB6612 drivers
- Preserve exact STM32 function names per constitution requirements
- Safety features (watchdog, output limits) are non-negotiable
- CMake build must work with standard Pico SDK installation
- USB CDC must not block real-time motor control loop

## Task Generation Rules

**Applied during task generation:**

1. **From Contracts**:
   - usb-cdc-protocol.md → T006 (protocol test) + T016 (implementation)
   - hal-interface.md → T007 (HAL test) + T012 (HAL implementation)
   - command-contract.md → T008 (4-wheel command test) + T023 (command processor)

2. **From Data Model**:
   - motor_state_t, robot_motors_t → T013 (motor types header)
   - shooter_state_t → T014 (shooter types header)
   - system_config_t → T015 (system config header)
   - Pin definitions → T004 (pin config header)

3. **From Research Decisions**:
   - PWM API choice → T018 (PWM control implementation)
   - USB CDC approach → T016 (USB interface implementation)
   - Watchdog configuration → T021 (watchdog manager)
   - Control loop design → T022 (control loop implementation)

4. **From Quickstart Scenarios**:
   - Build/flash procedures → T002 (CMake setup)
   - Smoke test procedures → T011 (hardware tests)
   - Performance validation → T028 (optimization)

## Validation Checklist

- [x] All contracts have corresponding tests (T006-T011)
- [x] All data model entities have header tasks (T013-T015)
- [x] All tests come before implementation (T006-T011 before T012-T026)
- [x] Parallel tasks target different files (verified [P] markings)
- [x] Each task specifies exact file path
- [x] No [P] task modifies same file as another [P] task
- [x] Safety requirements covered (watchdog T021, safety systems T026)
- [x] Constitution compliance verified (exact filenames T005, validation T031)

---
*Tasks ready for execution - 32 tasks total, estimated 2-3 weeks for embedded firmware development*