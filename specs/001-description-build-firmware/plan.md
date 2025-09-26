# Implementation Plan: Port legacy STM32 firmware to Raspberry Pi Pico (lpico)

**Branch**: `001-description-build-firmware` | **Date**: 2025-09-23 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-description-build-firmware/spec.md`

## Execution Flow (/plan command scope)
```
1. Load feature spec from Input path
   → If not found: ERROR "No feature spec at {path}"
2. Fill Technical Context (scan for NEEDS CLARIFICATION)
   → Detect Project Type from context (web=frontend+backend, mobile=app+api)
   → Set Structure Decision based on project type
3. Fill the Constitution Check section based on the content of the constitution document.
4. Evaluate Constitution Check section below
   → If violations exist: Document in Complexity Tracking
   → If no justification possible: ERROR "Simplify approach first"
   → Update Progress Tracking: Initial Constitution Check
5. Execute Phase 0 → research.md
   → If NEEDS CLARIFICATION remain: ERROR "Resolve unknowns"
6. Execute Phase 1 → contracts, data-model.md, quickstart.md, agent-specific template file (e.g., `CLAUDE.md` for Claude Code, `.github/copilot-instructions.md` for GitHub Copilot, `GEMINI.md` for Gemini CLI, `QWEN.md` for Qwen Code or `AGENTS.md` for opencode).
7. Re-evaluate Constitution Check section
   → If new violations: Refactor design, return to Phase 1
   → Update Progress Tracking: Post-Design Constitution Check
8. Plan Phase 2 → Describe task generation approach (DO NOT create tasks.md)
9. STOP - Ready for /tasks command
```

**IMPORTANT**: The /plan command STOPS at step 7. Phases 2-4 are executed by other commands:
- Phase 2: /tasks command creates tasks.md
- Phase 3-4: Implementation execution (manual or via tools)

## Summary
Port legacy STM32 firmware to Raspberry Pi Pico by creating a new `lpico/` folder that maintains exact filename compatibility while implementing minimal motor control (4 wheels) and shooter functionality using Pico SDK, with USB CDC communication interface and safety watchdog features.

## Technical Context
**Language/Version**: C/C++ (C17/C++17, Pico SDK 1.5.x compatible)  
**Primary Dependencies**: Pico SDK, CMake build system, USB CDC (TinyUSB)  
**Storage**: N/A (embedded firmware, flash memory for code)  
**Testing**: Hardware-in-loop tests, manual bench tests, smoke tests with physical hardware  
**Target Platform**: Raspberry Pi Pico (RP2040 microcontroller)  
**Project Type**: Single embedded project  
**Performance Goals**: Real-time motor control loop (200-1000 Hz), USB CDC latency <10ms, non-blocking control flow  
**Constraints**: KISS principle, exact filename compatibility with legacy_stm32, safety watchdog <300ms timeout, PWM output clamping  
**Scale/Scope**: Single microcontroller, 4 TB6612 motor drivers, 1 shooter ESC/gear output, USB CDC interface to RPi4B host

## Constitution Check
*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [ ] **Real-Time Control Integrity**: Document motor control loop budgets (target 1 kHz for PWM updates, 100 Hz for command processing) and ensure USB CDC handling doesn't block motor control timing.
- [ ] **Fail-Safe Actuation Envelope**: Implement watchdog timer (300ms per spec), PWM output clamping (0-255 range), and deadman switch that zeros motors when commands stop arriving.
- [ ] **ROS 2 Contract Fidelity**: USB CDC command protocol must be compatible with higher-level ROS2 nodes expecting specific message formats from STM32 legacy version.
- [ ] **Test-First Validation Discipline**: Create manual test procedures for motor direction/speed, shooter trigger, watchdog reset, and USB CDC command parsing before implementation.
- [ ] **Reproducible Firmware Delivery**: Establish CMake build process, document exact flash procedure (`picotool load`), and create smoke test checklist for validation.

## Project Structure

### Documentation (this feature)
```
specs/001-description-build-firmware/
├── plan.md              # This file (/plan command output)
├── research.md          # Phase 0 output (/plan command)
├── data-model.md        # Phase 1 output (/plan command)
├── quickstart.md        # Phase 1 output (/plan command)
├── contracts/           # Phase 1 output (/plan command)
└── tasks.md             # Phase 2 output (/tasks command - NOT created by /plan)
```

### Source Code (repository root)
```
# Single embedded project structure
src/robot_firmware/
├── lpico/              # NEW: Ported firmware
│   ├── include/        # Headers matching legacy_stm32 filenames
│   ├── src/            # Source files matching legacy_stm32 filenames  
│   ├── CMakeLists.txt  # Pico SDK build configuration
│   └── README.md       # Build and flash instructions
├── legacy_stm32/       # EXISTING: Original firmware for reference
└── pico/               # EXISTING: Current Pico project (may be merged/replaced)

tests/
├── hardware/           # HIL test procedures
├── bench/              # Manual bench test scripts
└── smoke/              # Quick validation procedures
```

**Structure Decision**: Single embedded project - firmware-specific structure under `src/robot_firmware/lpico/`

## Phase 0: Outline & Research
1. **Extract unknowns from Technical Context** above:
   - Pico SDK PWM API best practices for motor control
   - USB CDC implementation patterns and performance characteristics
   - Porting strategies from STM32 HAL to Pico SDK equivalents
   - Watchdog timer implementation on RP2040
   - Real-time scheduling considerations for motor control loops

2. **Generate and dispatch research agents**:
   ```
   For each unknown in Technical Context:
     Task: "Research Pico SDK PWM APIs for TB6612 motor driver control"
     Task: "Research USB CDC implementation on RP2040 for command interfaces"
     Task: "Research STM32 to Pico SDK porting strategies and HAL equivalents"
     Task: "Research RP2040 watchdog timer configuration and best practices"
     Task: "Research real-time motor control loop design on single-core RP2040"
   ```

3. **Consolidate findings** in `research.md` using format:
   - Decision: [what was chosen]
   - Rationale: [why chosen]
   - Alternatives considered: [what else evaluated]

**Output**: research.md with all Pico SDK implementation approaches resolved

## Phase 1: Design & Contracts
*Prerequisites: research.md complete*

1. **Extract entities from feature spec** → `data-model.md`:
   - Motor control structures (speed, direction, trim per motor)
   - Command message formats for USB CDC protocol
   - Safety state machine states and transitions
   - Configuration structures for pin mappings and parameters

2. **Generate API contracts** from functional requirements:
   - USB CDC command protocol specification
   - Motor control HAL interface contracts
   - Safety/watchdog interface specifications
   - Hardware abstraction layer (HAL) API contracts
   - Output protocol specifications to `/contracts/`

3. **Generate contract tests** from contracts:
   - Manual test procedures for each command
   - Hardware validation scripts
   - Safety behavior verification procedures
   - Tests must fail (no implementation yet)

4. **Extract test scenarios** from user stories:
   - Motor control scenario → HIL test procedure
   - Shooter control scenario → manual test procedure
   - Safety/watchdog scenario → failure injection test
   - Quickstart test = full system validation steps

5. **Update agent file incrementally** (O(1) operation):
   - Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType copilot`
     **IMPORTANT**: Execute it exactly as specified above for GitHub Copilot.
   - Add embedded systems and Pico SDK context
   - Add firmware safety and real-time considerations
   - Preserve manual additions between markers
   - Update recent changes (keep last 3)
   - Keep under 150 lines for token efficiency
   - Output to `.github/copilot-instructions.md`

**Output**: data-model.md, /contracts/*, failing test procedures, quickstart.md, .github/copilot-instructions.md

## Phase 2: Task Planning Approach
*This section describes what the /tasks command will do - DO NOT execute during /plan*

**Task Generation Strategy**:
- Load `.specify/templates/tasks-template.md` as base
- Generate tasks from Phase 1 design docs (contracts, data model, quickstart)
- Each contract → manual test procedure task [P]
- Each data model entity → implementation task [P] 
- Each user story → integration test task
- Implementation tasks to make tests pass

**Ordering Strategy**:
- TDD order: Test procedures before implementation 
- Dependency order: HAL layer before motor control before command interface
- Safety features integrated throughout (not afterthought)
- Mark [P] for parallel execution (independent modules)

**Estimated Output**: 15-20 numbered, ordered tasks in tasks.md

**IMPORTANT**: This phase is executed by the /tasks command, NOT by /plan

## Phase 3+: Future Implementation
*These phases are beyond the scope of the /plan command*

**Phase 3**: Task execution (/tasks command creates tasks.md)  
**Phase 4**: Implementation (execute tasks.md following constitutional principles)  
**Phase 5**: Validation (run HIL tests, execute quickstart.md, safety validation)

## Complexity Tracking
*Fill ONLY if Constitution Check has violations that must be justified*

| Violation | Why Needed | Simpler Alternative Rejected Because     |
| --------- | ---------- | ---------------------------------------- |
| N/A       | N/A        | All requirements align with constitution |

## Progress Tracking
*This checklist is updated during execution flow*

**Phase Status**:
- [x] Phase 0: Research complete (/plan command)
- [x] Phase 1: Design complete (/plan command)
- [x] Phase 2: Task planning complete (/plan command - describe approach only)
- [x] Phase 3: Tasks generated (/tasks command)
- [ ] Phase 4: Implementation complete
- [ ] Phase 5: Validation passed

**Gate Status**:
- [x] Initial Constitution Check: PASS
- [x] Post-Design Constitution Check: PASS
- [x] All NEEDS CLARIFICATION resolved
- [x] Complexity deviations documented

---
*Based on Constitution v1.0.0 - See `.specify/memory/constitution.md`*
