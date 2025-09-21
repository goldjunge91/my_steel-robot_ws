# Phase 1: Data Model

## Entities
- Robot
- HAL
- ControllerManager
- Wheel
- ESC
- Teleop
- Watchdog
- GeometryConfig

### Robot
- id: string (pkg-local name)
- pose: {x: float, y: float, theta: float}
- velocity: {vx: float, vy: float, omega: float}
- mode: enum {SIMULATION, HARDWARE}
- sensors: list of sensor ids

### HAL
- name: string
- mode: enum {sim, hw}
- enabled: bool
- safety_state: enum {SAFE, ARMED, FAULT}
- diagnostics_topic: string
- telemetry_topic: string

### ControllerManager
- controllers: list of controller names
- active_controller: string

### Wheel
- id: string
- radius: float
- position: {x: float, y: float}
- has_encoder: bool

### ESC
- id: string
- max_rpm: int
- enabled: bool

### Teleop
- input_device: enum {XBOX, KEYBOARD}
- linear_scale: float
- angular_scale: float

### Watchdog
- timeout_s: float
- last_heartbeat: timestamp
- status: enum {OK, TIMEOUT}

### GeometryConfig
- wheel_base: float
- wheel_track: float
- wheel_radius: float

## Relationships
- Robot has many Wheels
- HAL manages ControllerManager
- ControllerManager controls Wheels via ESCs
- Teleop sends commands to ControllerManager
- Watchdog monitors HAL and Pico

## Validation Rules
- All wheel geometry parameters must be > 0
- Watchdog timeout must be < 1s
- Teleop commands must be rate-limited
- HAL.mode must be one of {sim, hw}
- Robot.mode must be one of {SIMULATION, HARDWARE}
- ControllerManager.active_controller must be in controllers list

## State Transitions
- HAL: disabled → enabled (via service)
- Watchdog: healthy → timeout
- ControllerManager: inactive → active

Notes:
- Switching modes (SIMULATION ↔ HARDWARE) must perform a safe-stop: disable motion, wait for controllers to stop, switch backend, re-enable if requested.
- When hardware backend fails to initialize, HAL must revert to sim mode and publish a diagnostic with reason.
# Phase 1: Data Model

## Entities
- Robot
- HAL
- ControllerManager
- Wheel
- ESC
- Teleop
- Watchdog
- GeometryConfig

## Relationships
- Robot has many Wheels
- HAL manages ControllerManager
- ControllerManager controls Wheels via ESCs
- Teleop sends commands to ControllerManager
- Watchdog monitors HAL and Pico

## Validation Rules
- All wheel geometry parameters must be > 0
- Watchdog timeout must be < 1s
- Teleop commands must be rate-limited

## State Transitions
- HAL: disabled → enabled (via service)
- Watchdog: healthy → timeout
- ControllerManager: inactive → active
