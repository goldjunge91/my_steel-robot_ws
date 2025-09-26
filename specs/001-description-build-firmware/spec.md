# Feature Specification: Port legacy STM32 firmware to Raspberry Pi Pico (lpico)

**Feature Branch**: `001-description-build-firmware`
**Created**: 2025-09-23
**Status**: Draft
**Input**: User description: "Build firmware for Raspberry Pi Pico by porting legacy STM32 firmware into a new 'lpico' folder, keeping exact filenames from legacy_stm32, KISS/DRY, minimal implementation to control wheels and nerf shooter, target build and test"

## Execution Flow (main)
```
1. Parse user description from Input
   → If empty: ERROR "No feature description provided"
2. Extract key concepts from description
   → Identify: actors, actions, data, constraints
3. For each unclear aspect:
   → Mark with [NEEDS CLARIFICATION: specific question]
4. Fill User Scenarios & Testing section
   → If no clear user flow: ERROR "Cannot determine user scenarios"
5. Generate Functional Requirements
   → Each requirement must be testable
   → Mark ambiguous requirements
6. Identify Key Entities (if data involved)
7. Run Review Checklist
   → If any [NEEDS CLARIFICATION]: WARN "Spec has uncertainties"
   → If implementation details found: ERROR "Remove tech details"
8. Return: SUCCESS (spec ready for planning)
```

---

## ⚡ Quick Guidelines
- ✅ Focus on WHAT users need and WHY
- ❌ Avoid HOW to implement (we'll still include minimal technical notes for developers in implementation tasks)
- 👥 Written for stakeholders and implementers to align on scope

### Section Requirements
- **Mandatory sections**: Must be completed for every feature
- **Optional sections**: Include only when relevant to the feature
- When a section doesn't apply, remove it entirely

---

## User Scenarios & Testing *(mandatory)*

### Primary User Story
As a developer, I want the legacy STM32 firmware ported to run on a Raspberry Pi Pico so the robot can be driven (4 wheels with individual control) and fire the nerf shooter with minimal, testable firmware that reuses the same filenames and APIs where practical.

### Acceptance Scenarios
1. Given the repo with `legacy_stm32` code, when the porting task runs, then a new `lpico/` folder exists under `src/robot_firmware/` containing source and include files with the same filenames as in `legacy_stm32` and a minimal build that flashes onto the Pico.
2. Given the Pico firmware is flashed on a Raspberry Pi Pico connected to the robot hardware (or simulator), when commands to drive individual wheels (FL/FR/RL/RR) or trigger shooter are issued via USB CDC, then the motors spin independently and the shooter fires as expected.

### Edge Cases
- What happens when hardware is not present? The firmware should degrade gracefully and not crash (provide safe no-op behavior).
- If exact low-level peripherals (timers, ADCs) differ between STM32 and Pico, the port should implement simple hardware abstraction layer stubs that allow basic motor PWM and digital output for the shooter.

## Requirements *(mandatory)*

### Functional Requirements
- **FR-001**: The system MUST provide a `lpico` firmware folder mirroring `legacy_stm32` filenames exactly (sources and headers).
- **FR-002**: The firmware MUST build for Raspberry Pi Pico (CMake/PIO) and produce a flashable binary.
- **FR-003**: The firmware MUST implement motor control for 4 wheels (front-left, front-right, rear-left, rear-right) using the Pico PWM APIs with direction and speed control.
- **FR-004**: The firmware MUST implement a shooter trigger output (digital GPIO) controllable from main loop or simple command interface.
- **FR-005**: The implementation MUST follow KISS and DRY: keep logic minimal, avoid STM32-specific subsystems; provide small HAL layer with identical function names where feasible.
- **FR-006**: The system MUST include build instructions and a minimal README explaining how to flash and test on the Pico.

*Clarifications & decisions*:
- **FR-007 (Pin mapping)**: Use the following Raspberry Pi Pico pin mapping for TB6612 motor drivers, encoders, and ESC/gear lines. These matches the provided `config.h` layout and will be applied to the `lpico` firmware exactly (pin names kept as macros):

```cpp
// ===== Laufzeit-Defaults =====
#define PWM_MAX            255   // UNO PWM 8-bit
#define WATCHDOG_MS        300   // Stop, wenn solange kein Kommando
#define LOOP_MS            10    // Regeltakt (ms)
#define DEADZONE_DEFAULT   30    // 0..80 (Anlauf-Offset)
#define SLEW_DEFAULT       8     // 1..30 (max. PWM-Änderung pro LOOP_MS)

// ===== Offset Motoren(pi pico) =====
// Richtungskorrektur: 1 oder -1 (wenn "vorwärts" falsch herum ist)
// Richtungskorrektur
#define OFFSET_FL (+1)
#define OFFSET_FR (+1)
#define OFFSET_RL (+1)
#define OFFSET_RR (+1)

// ===== Verkabelung TB6612 =====n+// Linkes Board (2 Motoren)

// Gemeinsamer Standby-Pin für beide TB6612
#define STBY 28

// Front-Left  (TB1 A)
#define FL_IN1 18
#define FL_IN2 19
#define FL_PWM 2
#define ENC_FL_A   8
#define ENC_FL_B   9
// Front-Right (TB1 B)
#define FR_IN1 17
#define FR_IN2 20
#define FR_PWM 3
#define ENC_FR_A   10
#define ENC_FR_B   11

// Rear-Left   (TB2 A)
#define RL_IN1 21
#define RL_IN2 22
#define RL_PWM 4
#define ENC_RL_A   12
#define ENC_RL_B   13
// Rear-Right  (TB2 B)
#define RR_IN1 26
#define RR_IN2 27
#define RR_PWM 5
#define ENC_RR_A   6
#define ENC_RR_B   7

// Feintrimm je Motor (PWM-Offset in Schritten, zum Geradeauslauf)
#define TRIM_FL     0
#define TRIM_FR     0
#define TRIM_RL     0
#define TRIM_RR     0

// // ##
#define ESC1_PIN   14
#define ESC2_PIN   15
#define GEAR_PIN   16
```

- **FR-008 (Control interface)**: Communication to the higher-level SBC will be USB CDC (USB serial) from the Pico to a Raspberry Pi 4B. The Pico will enumerate as a USB CDC device; the RPi4B will connect as host and send control commands. The firmware implements a 4-wheel command protocol supporting individual motor control (FL/FR/RL/RR format) and shooter functions as specified in the contracts.

### Key Entities
- **Firmware HAL**: Minimal abstraction layer mapping existing STM32 function names to Pico implementations for PWM, GPIO, and timers.
- **Motor Controller**: Module providing 4-wheel motor control with individual speed and direction for each wheel.
- **Shooter Controller**: Module providing trigger_shot() with safety debounce and ESC control.

---

## Review & Acceptance Checklist

### Content Quality
- [x] Feature name, branch, and input recorded
- [x] Pin mappings and command interface fully specified and consistent

### Requirement Completeness
- [x] Requirements are actionable and testable
- [x] Pin mappings and 4-wheel control interface specified (see FR-007/FR-008)
- [x] All motor control specifications align with 4-wheel hardware design

---

## Execution Status
- [x] User description parsed
- [x] Key concepts extracted
- [x] Ambiguities marked and resolved for pinmap & control interface
- [x] User scenarios defined (drafted)
- [x] Requirements generated (drafted)
- [x] Entities identified
- [ ] Review checklist passed (pending implementation review)

---

## Next Steps (Implementation Plan)

1. Create `src/robot_firmware/lpico/` and copy files from `src/robot_firmware/legacy_stm32/`, preserving filenames exactly.
2. Replace STM32 HAL calls with a tiny Pico HAL in files under `include/`/`src/` while keeping original function names where possible.
3. Provide a top-level CMakeLists.txt (or adapt existing Pico files in repo) and a minimal `README.md` for build/flash steps.
4. Add a 4-wheel motor command interface and shooter control for manual testing.
