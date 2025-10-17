<!--
Sync Impact Report
Version change: 2.1.1 → 3.0.0
Modified principles:
- Placeholder Principle 1 → Safety-Critical Autonomy
- Placeholder Principle 2 → Deterministic Control & Real-Time Assurance
- Placeholder Principle 3 → Simulation-First Integration
- Placeholder Principle 4 → Observability & Telemetry Preservation
- Placeholder Principle 5 → Modular ROS 2 Contracts
Added sections:
- Operational Constraints
- Development Workflow & Quality Gates
Removed sections:
- None
Templates requiring updates:
- ✅ .specify/templates/plan-template.md (alignment verified; Constitution Check references retained)
- ✅ .specify/templates/spec-template.md (alignment verified; user story/test guidance matches principles)
- ✅ .specify/templates/tasks-template.md (alignment verified; user-story framing kept)
Follow-up TODOs:
- TODO(RATIFICATION_DATE): Capture original adoption date once project owners confirm
-->
# my_steel Robot Workspace Constitution

## Core Principles

### Safety-Critical Autonomy
- All motion-control, launcher, and manipulator nodes MUST expose a documented
  emergency-stop interface and prove it in simulation before hardware tests.
- Any change that can energize actuators MUST include a hazard analysis and
  mitigation checklist in the associated spec or plan prior to merge.
- Remote operation flows MUST fail safe: absence of operator heartbeat for 500 ms
  or more MUST transition the robot to an idle state.

Rationale: The platform operates mobile hardware with projectiles; the only
acceptable default is to halt safely on every uncertainty.

### Deterministic Control & Real-Time Assurance
- Control loops MUST declare their target frequency and ship automated
  monitoring (test or ros2 launch check) showing <20% drift over a 10 s run.
- Firmware and ROS2 nodes that miss two consecutive control cycles MUST degrade
  to a safe hold state and raise a `/diagnostics` fault entry.
- Timing-sensitive code MUST keep blocking operations off the main loop;
  background tasks belong in separate executors or microcontroller cores.

Rationale: Stable motion and firing depend on predictable timing; deterministic
behaviour prevents oscillations and damage.

### Simulation-First Integration
- New behaviour MUST land with a reproducible Gazebo (or equivalent) scenario
  and pass it before requesting physical testing.
- Specs MUST document simulation gaps; when hardware parity is impossible, a
  mitigation plan MUST be approved in the plan.md Constitution Check.
- Launch files MUST support a `sim:=true` switch that exercises the same nodes
  and topics intended for the real robot.

Rationale: Simulation parity protects hardware, operators, and schedule by
catching regressions before physical trials.

### Observability & Telemetry Preservation
- Every ROS2 node MUST emit structured logs (severity + context) and provide a
  ros2 topic or service for live diagnostics.
- Critical telemetry topics (drive IMU, odometry, launcher state, safety status)
  MUST use reliable QoS and be recorded to bag files during testing.
- Release candidates MUST include an operations note covering log collection,
  bag rotation, and metric thresholds for alerting.

Rationale: High-fidelity telemetry is required to debug field issues, prove
safety compliance, and tune control algorithms.

### Modular ROS 2 Contracts
- Interface definitions (messages, services, actions) MUST live in dedicated
  interface packages; implementation packages import them.
- Breaking changes to topics or messages MUST go through spec → plan → tasks
  with explicit contract review and a semantic version bump.
- Package dependency graphs MUST stay acyclic between `src/` and `lib/`; shared
  utilities belong in `lib/` with documentation.

Rationale: Clear contracts and modular packages keep the workspace maintainable
as teams extend the platform.

## Operational Constraints
- Hardware bring-up MUST include documented wiring, calibration, and firmware
  flashing steps in `docs/` before features are marked done.
- Networked operation MUST default to authenticated tunnels (e.g., Tailscale)
  and document fallback when VPN is unavailable.
- Deployments on Raspberry Pi targets MUST reserve at least 15% CPU headroom
  for safety monitors and keep GPU workloads optional.

## Development Workflow & Quality Gates
- Every feature spec and plan MUST complete the Constitution Check section,
  mapping affected principles and mitigation tasks before Phase 0 concludes.
- Plans MUST articulate how simulation, telemetry, and contract obligations will
  be satisfied; tasks.md MUST reflect those obligations explicitly.
- No code merges without: passing automated tests, demonstrated simulation run,
  and updated documentation for setup, hazards, and operations.
- Releases MUST tag the workspace with the feature branch reference, include
  rosbag archives for validation, and capture any deviations from principles.

## Governance
- Amendments: proposals open an issue referencing the impacted principles,
  document rationale, and obtain owner approval before merging changes into this
  file.
- Versioning: semantic versioning applies (MAJOR for principle changes/removals,
  MINOR for new principles/sections, PATCH for clarifications); version history
  lives in the version line below.
- Compliance Reviews: Constitution Checks occur at plan kickoff and pre-merge;
  failures block progress until resolved or waived by project owners with
  documented justification.
- Audit Trail: Keep prior versions in git history; plans referencing older
  versions MUST note compensating controls until upgraded.

**Version**: 3.0.0 | **Ratified**: TODO(RATIFICATION_DATE): original adoption date unknown | **Last Amended**: 2025-10-18
