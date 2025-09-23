# Data Model: Pico Firmware Motor Control System

**Feature**: Port legacy STM32 firmware to Raspberry Pi Pico (lpico)  
**Date**: 2025-09-23  
**Phase**: Phase 1 - Design & Contracts

## Core Data Structures

### 1. Motor Control State

```c
typedef struct {
    int8_t direction;       // -1 (reverse), 0 (stop), +1 (forward)
    uint8_t speed;          // 0-255 PWM duty cycle
    int8_t trim;            // Fine-tuning offset (-127 to +127)
    uint8_t enabled;        // Motor enable flag
} motor_state_t;

typedef struct {
    motor_state_t front_left;
    motor_state_t front_right;
    motor_state_t rear_left;
    motor_state_t rear_right;
} robot_motors_t;
```

**Rationale**: Preserves exact data layout from legacy STM32 for compatibility

### 2. Shooter Control State

```c
typedef struct {
    uint8_t esc1_enabled;   // ESC1 enable flag
    uint8_t esc2_enabled;   // ESC2 enable flag
    uint8_t gear_state;     // Gear position (0=retracted, 1=extended)
    uint32_t last_fire_ms;  // Timestamp of last fire command (debounce)
} shooter_state_t;
```

**Validation Rules**:
- ESC enable states are boolean (0 or 1)
- Gear state debounce prevents rapid switching (<100ms)
- Fire commands limited to max 10Hz to prevent hardware damage

### 3. System Configuration

```c
#define PWM_MAX            255   // 8-bit PWM resolution
#define WATCHDOG_MS        300   // Safety timeout
#define LOOP_MS            10    // Control loop period
#define DEADZONE_DEFAULT   30    // Motor startup deadzone
#define SLEW_DEFAULT       8     // Max PWM change per loop

typedef struct {
    uint8_t pwm_max;
    uint16_t watchdog_ms;
    uint8_t loop_ms;
    uint8_t deadzone;
    uint8_t slew_rate;
} system_config_t;
```

**Immutable Pin Configuration** (from FR-007):
```c
// TB6612 Motor Driver Pins
#define STBY        28    // Common standby for both TB6612s

// Front-Left Motor (TB1 A)
#define FL_IN1      18
#define FL_IN2      19
#define FL_PWM      2
#define ENC_FL_A    8
#define ENC_FL_B    9

// Front-Right Motor (TB1 B)
#define FR_IN1      17
#define FR_IN2      20
#define FR_PWM      3
#define ENC_FR_A    10
#define ENC_FR_B    11

// Rear-Left Motor (TB2 A)
#define RL_IN1      21
#define RL_IN2      22
#define RL_PWM      4
#define ENC_RL_A    12
#define ENC_RL_B    13

// Rear-Right Motor (TB2 B)
#define RR_IN1      26
#define RR_IN2      27
#define RR_PWM      5
#define ENC_RR_A    6
#define ENC_RR_B    7

// Shooter Control
#define ESC1_PIN    14
#define ESC2_PIN    15
#define GEAR_PIN    16
```

### 4. Command Message Format

```c
typedef enum {
    CMD_MOTOR_SET,      // Set motor speeds: "M FL:100 FR:100 RL:100 RR:100"
    CMD_MOTOR_STOP,     // Emergency stop: "STOP"
    CMD_SHOOTER_FIRE,   // Fire sequence: "FIRE"
    CMD_SHOOTER_ESC,    // ESC control: "ESC1:1 ESC2:0"
    CMD_GEAR_SET,       // Gear position: "GEAR:1"
    CMD_STATUS,         // Status query: "STATUS"
    CMD_INVALID
} command_type_t;

typedef struct {
    command_type_t type;
    union {
        robot_motors_t motors;
        shooter_state_t shooter;
        uint8_t gear_position;
    } data;
} command_message_t;
```

### 5. Status Response Format

```c
typedef struct {
    robot_motors_t current_motors;
    shooter_state_t shooter_status;
    uint32_t uptime_ms;
    uint8_t watchdog_triggered;
    uint16_t loop_frequency_hz;
    char status_string[128];
} status_response_t;
```

### 6. Encoder State (Simplified)

```c
typedef struct {
    volatile int32_t count;     // Encoder tick count
    volatile uint32_t last_update_ms;   // Last encoder update
    uint8_t direction;          // Last known direction
} encoder_state_t;

typedef struct {
    encoder_state_t front_left;
    encoder_state_t front_right;
    encoder_state_t rear_left;
    encoder_state_t rear_right;
} robot_encoders_t;
```

**Note**: Encoder implementation simplified for MVP - basic edge counting only

## State Transitions

### Motor State Machine

```
STOPPED ─┬─> FORWARD (speed > deadzone, direction = +1)
         ├─> REVERSE (speed > deadzone, direction = -1)
         └─> STOPPED (speed <= deadzone)

FORWARD ─┬─> STOPPED (speed <= deadzone OR emergency stop)
         ├─> REVERSE (direction = -1, via STOPPED state)
         └─> FORWARD (speed adjustment)

REVERSE ─┬─> STOPPED (speed <= deadzone OR emergency stop)
         ├─> FORWARD (direction = +1, via STOPPED state)
         └─> REVERSE (speed adjustment)
```

**Safety Rule**: Direction changes must pass through STOPPED state to prevent motor damage

### Shooter State Machine

```
IDLE ────> FIRING (fire command received)
FIRING ──> COOLDOWN (fire sequence complete)
COOLDOWN -> IDLE (debounce timeout expired)
```

**Safety Rules**:
- Minimum 100ms cooldown between fire commands
- ESC outputs automatically disabled after 5 seconds of inactivity
- Gear changes blocked during FIRING state

## Error Conditions & Recovery

### Watchdog Recovery
```c
typedef enum {
    RECOVERY_CLEAN_BOOT,
    RECOVERY_WATCHDOG_RESET,
    RECOVERY_BROWNOUT,
    RECOVERY_UNKNOWN
} recovery_reason_t;
```

**Recovery Actions**:
1. All motors set to STOPPED state
2. Shooter set to IDLE state
3. System configuration reset to defaults
4. Status LED indicates recovery reason

### Communication Timeouts
- Command timeout: 300ms (triggers emergency stop)
- Status reporting: Every 1000ms when active
- Error reporting: Immediate on fault detection

## Memory Layout Considerations

**Stack Usage**: <2KB for control loop and ISR contexts
**Heap Usage**: Minimal - mostly static allocation
**Flash Usage**: <32KB estimated for core functionality
**RAM Usage**: <8KB for all data structures and buffers

---
*Data model designed for Phase 1 of implementation plan*
