# Pico Firmware Architecture

## Overview

Real-time firmware for Raspberry Pi Pico running FreeRTOS with micro-ROS for ROS2 communication. Handles motor control, sensor reading, and odometry calculation.

## System Architecture

```mermaid
graph TB
    subgraph "Raspberry Pi Pico"
        direction TB
        
        subgraph "FreeRTOS"
            TASKS[Agent Tasks<br/>100 Hz]
            SCHED[Scheduler<br/>Priority-based]
        end
        
        subgraph "micro-ROS"
            UROS[uRosBridge<br/>Singleton]
            PUBS[Publishers]
            SUBS[Subscribers]
        end
        
        subgraph "Application Agents"
            DDD[DDD Agent<br/>Odometry & Control]
            MOTORS[Motors Agent<br/>PID Control]
            IMU[IMU Agent<br/>ICM20948]
            TOF[VL6180X Agent<br/>ToF Sensor]
            US[HCSR04 Agent<br/>Ultrasonic]
        end
        
        subgraph "HAL"
            PWM[PWM Manager]
            ENC[Encoder Manager]
            SPI[SPI Driver]
            I2C[I2C Driver]
        end
        
        TASKS --> DDD
        TASKS --> MOTORS
        TASKS --> IMU
        TASKS --> TOF
        TASKS --> US
        
        DDD --> UROS
        MOTORS --> UROS
        IMU --> UROS
        TOF --> UROS
        US --> UROS
        
        UROS --> PUBS
        UROS --> SUBS
        
        MOTORS --> PWM
        MOTORS --> ENC
        IMU --> SPI
        TOF --> I2C
        US --> GPIO
    end
    
    subgraph "Hardware"
        MOTOR_HW[4x DC Motors<br/>with Encoders]
        IMU_HW[ICM20948<br/>9-DOF IMU]
        TOF_HW[VL6180X<br/>ToF Sensor]
        US_HW[HC-SR04<br/>Ultrasonic]
    end
    
    PWM --> MOTOR_HW
    ENC --> MOTOR_HW
    SPI --> IMU_HW
    I2C --> TOF_HW
    GPIO --> US_HW
    
    UROS <-->|USB Serial| ROS2[ROS2 Host<br/>micro-ROS Agent]
    
    style UROS fill:#4CAF50
    style DDD fill:#2196F3
    style MOTORS fill:#FF9800
```

## Agent Architecture

```mermaid
classDiagram
    class Agent {
        <<abstract>>
        #name: string
        #priority: uint
        #stack_size: uint
        +init() bool
        +run() void*
        +getName() string
        +getPriority() uint
    }
    
    class BlinkAgent {
        -led_pin: uint
        -blink_rate: uint
        +init() bool
        +run() void*
    }
    
    class DDD {
        -odom_publisher: rcl_publisher_t
        -cmd_vel_subscriber: rcl_subscription_t
        -twist_msg: Twist
        -odom_msg: Odometry
        -position: Vector3f
        -orientation: float
        +init() bool
        +run() void*
        +calculateOdometry() void
        +publishOdometry() void
        +cmdVelCallback() void
    }
    
    class MotorsAgent {
        -joint_state_publisher: rcl_publisher_t
        -motor_mgr: MotorMgr*
        -pid_controllers: MotorPID[4]
        +init() bool
        +run() void*
        +updatePID() void
        +publishJointStates() void
    }
    
    class ImuAgent {
        -imu_publisher: rcl_publisher_t
        -imu_sensor: ICM20948*
        -imu_msg: Imu
        +init() bool
        +run() void*
        +readSensor() void
        +publishImuData() void
    }
    
    class vl6180xAgent {
        -range_publisher: rcl_publisher_t
        -illuminance_publisher: rcl_publisher_t
        -sensor: VL6180X*
        +init() bool
        +run() void*
        +readRange() void
        +readIlluminance() void
    }
    
    Agent <|-- BlinkAgent
    Agent <|-- DDD
    Agent <|-- MotorsAgent
    Agent <|-- ImuAgent
    Agent <|-- vl6180xAgent
```

## Data Flow Diagram

```mermaid
flowchart TB
    subgraph "ROS2 Host"
        AGENT[micro-ROS Agent]
        CTRL[Controller]
    end
    
    subgraph "Pico Firmware"
        direction TB
        
        subgraph "micro-ROS Layer"
            UROS[uRosBridge]
            SUB[/rt/cmd_vel<br/>Subscriber]
            PUB_JS[/rt/joint_states<br/>Publisher]
            PUB_IMU[/rt/imu/data_raw<br/>Publisher]
            PUB_ODOM[/rt/odom<br/>Publisher]
            PUB_SENS[/rt/sensors/*<br/>Publishers]
        end
        
        subgraph "Control Layer"
            DDD_A[DDD Agent<br/>Odometry]
            MOTOR_A[Motors Agent<br/>PID Control]
        end
        
        subgraph "Sensor Layer"
            IMU_A[IMU Agent]
            TOF_A[ToF Agent]
        end
        
        subgraph "Hardware Layer"
            PWM_M[PWM Manager]
            ENC_M[Encoder Manager]
            SPI_D[SPI Driver]
            I2C_D[I2C Driver]
        end
    end
    
    subgraph "Hardware"
        MOTORS[DC Motors]
        ENCODERS[Encoders]
        IMU_HW[ICM20948]
        TOF_HW[VL6180X]
    end
    
    CTRL -->|/cmd_vel| AGENT
    AGENT <-->|USB Serial| UROS
    
    UROS --> SUB
    SUB --> DDD_A
    DDD_A --> MOTOR_A
    
    MOTOR_A --> PWM_M
    PWM_M --> MOTORS
    
    ENCODERS --> ENC_M
    ENC_M --> MOTOR_A
    MOTOR_A --> PUB_JS
    
    MOTOR_A --> DDD_A
    DDD_A --> PUB_ODOM
    
    IMU_HW --> SPI_D
    SPI_D --> IMU_A
    IMU_A --> PUB_IMU
    
    TOF_HW --> I2C_D
    I2C_D --> TOF_A
    TOF_A --> PUB_SENS
    
    PUB_JS --> UROS
    PUB_IMU --> UROS
    PUB_ODOM --> UROS
    PUB_SENS --> UROS
    
    UROS --> AGENT
    AGENT --> CTRL
```

## Sequence Diagram: Initialization

```mermaid
sequenceDiagram
    participant Main
    participant FreeRTOS
    participant uRosBridge
    participant Agents
    participant Hardware
    
    Main->>Hardware: Initialize GPIO, PWM, SPI, I2C
    Main->>uRosBridge: Initialize micro-ROS
    uRosBridge->>uRosBridge: Create node
    uRosBridge->>uRosBridge: Create executor
    
    Main->>Agents: Create agent instances
    loop For each agent
        Main->>Agents: agent->init()
        Agents->>uRosBridge: Create publishers/subscribers
        Agents->>Hardware: Initialize hardware
        Agents-->>Main: Success
    end
    
    Main->>FreeRTOS: Create agent tasks
    loop For each agent
        FreeRTOS->>Agents: xTaskCreate(agent->run)
    end
    
    Main->>FreeRTOS: Start scheduler
    
    Note over FreeRTOS,Agents: Tasks running at 100 Hz
```

## Sequence Diagram: Control Loop

```mermaid
sequenceDiagram
    participant Host as ROS2 Host
    participant uRos as uRosBridge
    participant DDD as DDD Agent
    participant Motors as Motors Agent
    participant PWM as PWM Manager
    participant Enc as Encoders
    
    loop 100 Hz Control Loop
        Host->>uRos: /cmd_vel (Twist)
        uRos->>DDD: cmdVelCallback()
        DDD->>DDD: Store target velocities
        
        DDD->>Motors: Set target wheel velocities<br/>(inverse kinematics)
        
        Enc->>Motors: Read encoder positions
        Motors->>Motors: Calculate velocities
        Motors->>Motors: PID control
        Motors->>PWM: Set PWM duty cycles
        PWM->>PWM: Update motor outputs
        
        Motors->>uRos: Publish /joint_states
        uRos->>Host: Forward joint states
        
        Motors->>DDD: Provide wheel velocities
        DDD->>DDD: Calculate odometry<br/>(forward kinematics)
        DDD->>uRos: Publish /odom
        uRos->>Host: Forward odometry
    end
```

## Task Scheduling

```mermaid
gantt
    title FreeRTOS Task Scheduling (10ms period)
    dateFormat X
    axisFormat %L ms
    
    section High Priority
    Motors Agent (100Hz)    :0, 2
    DDD Agent (100Hz)       :2, 4
    
    section Medium Priority
    IMU Agent (50Hz)        :4, 6
    
    section Low Priority
    ToF Agent (Variable)    :6, 7
    Ultrasonic (Variable)   :7, 8
    Blink Agent (1Hz)       :8, 9
    
    section Idle
    Idle Task               :9, 10
```

## Memory Layout

```mermaid
graph TB
    subgraph "Pico Memory (264 KB RAM)"
        direction TB
        
        STACK[FreeRTOS Stacks<br/>~64 KB]
        HEAP[FreeRTOS Heap<br/>~128 KB]
        UROS[micro-ROS Buffers<br/>~32 KB]
        STATIC[Static Data<br/>~16 KB]
        BSS[BSS<br/>~8 KB]
        FREE[Free<br/>~16 KB]
    end
    
    style STACK fill:#4CAF50
    style HEAP fill:#2196F3
    style UROS fill:#FF9800
```

## Pin Configuration

```mermaid
graph LR
    subgraph "GPIO Pins"
        direction TB
        
        subgraph "Motors"
            M_FL[GP20/21<br/>Front Left PWM]
            M_FR[GP4/5<br/>Front Right PWM]
            M_RL[GP14/15<br/>Rear Left PWM]
            M_RR[GP22/28<br/>Rear Right PWM]
        end
        
        subgraph "Encoders"
            E_FL[GP6/7<br/>Front Left Enc]
            E_FR[GP8/9<br/>Front Right Enc]
            E_RL[GP10/11<br/>Rear Left Enc]
            E_RR[GP12/13<br/>Rear Right Enc]
        end
        
        subgraph "Sensors"
            IMU_SPI[GP16-19<br/>IMU SPI0]
            TOF_I2C[GP2/3<br/>ToF I2C1]
        end
        
        subgraph "Status"
            LED[GP26<br/>Status LED]
        end
    end
    
    style M_FL fill:#4CAF50
    style M_FR fill:#4CAF50
    style M_RL fill:#4CAF50
    style M_RR fill:#4CAF50
```

## PID Control Flow

```mermaid
flowchart TB
    START[Control Loop Start] --> READ[Read Encoder Position]
    READ --> CALC_VEL[Calculate Velocity<br/>position delta / dt]
    CALC_VEL --> ERROR[Calculate Error<br/>target - actual]
    
    ERROR --> P[Proportional Term<br/>Kp * error]
    ERROR --> I[Integral Term<br/>Ki * Σerror]
    ERROR --> D[Derivative Term<br/>Kd * Δerror]
    
    P --> SUM[Sum PID Terms]
    I --> SUM
    D --> SUM
    
    SUM --> LIMIT[Apply Limits<br/>-100% to +100%]
    LIMIT --> PWM[Set PWM Duty Cycle]
    PWM --> PUBLISH[Publish Joint State]
    PUBLISH --> WAIT[Wait 10ms]
    WAIT --> START
    
    style ERROR fill:#4CAF50
    style SUM fill:#2196F3
    style PWM fill:#FF9800
```

## Odometry Calculation

```mermaid
flowchart TB
    START[Get Wheel Velocities] --> FK[Forward Kinematics<br/>Mecanum Drive]
    
    FK --> VX[Calculate vx<br/>Σwheel_vel / 4]
    FK --> VY[Calculate vy<br/>weighted sum]
    FK --> OMEGA[Calculate ω<br/>weighted sum]
    
    VX --> INTEGRATE[Integrate Velocities<br/>Δt = 10ms]
    VY --> INTEGRATE
    OMEGA --> INTEGRATE
    
    INTEGRATE --> UPDATE_POS[Update Position<br/>x, y, θ]
    UPDATE_POS --> CREATE_MSG[Create Odometry Message]
    CREATE_MSG --> PUBLISH[Publish /odom]
    
    style FK fill:#4CAF50
    style INTEGRATE fill:#2196F3
```

## Communication Protocol

```mermaid
sequenceDiagram
    participant Pico as Pico Firmware
    participant USB as USB CDC
    participant Agent as micro-ROS Agent
    participant ROS2 as ROS2 Network
    
    Note over Pico,Agent: Initialization
    Pico->>USB: Open USB CDC
    Agent->>USB: Connect to /dev/ttyACM0
    Pico->>Agent: micro-ROS handshake
    Agent-->>Pico: Connection established
    
    Note over Pico,ROS2: Runtime Communication
    
    loop Every 10ms
        Pico->>Agent: Publish /joint_states
        Agent->>ROS2: Forward to ROS2 network
        
        ROS2->>Agent: /cmd_vel command
        Agent->>Pico: Forward to firmware
    end
    
    loop Every 20ms
        Pico->>Agent: Publish /imu/data_raw
        Pico->>Agent: Publish /odom
        Agent->>ROS2: Forward to ROS2 network
    end
```

## Build System

```mermaid
graph TB
    subgraph "Source Files"
        SRC[src/*.cpp]
        HDR[src/*.h]
        HAL[src/hal/*.cpp]
        APP[src/application/*.cpp]
    end
    
    subgraph "Dependencies"
        PICO_SDK[Pico SDK]
        FREERTOS[FreeRTOS]
        MICROROS[micro-ROS]
        EIGEN[Eigen]
    end
    
    subgraph "Build Process"
        CMAKE[CMakeLists.txt]
        MAKE[Makefile Wrapper]
    end
    
    subgraph "Output"
        ELF[firmware.elf]
        UF2[firmware.uf2]
        BIN[firmware.bin]
    end
    
    SRC --> CMAKE
    HDR --> CMAKE
    HAL --> CMAKE
    APP --> CMAKE
    
    PICO_SDK --> CMAKE
    FREERTOS --> CMAKE
    MICROROS --> CMAKE
    EIGEN --> CMAKE
    
    CMAKE --> MAKE
    MAKE --> ELF
    ELF --> UF2
    ELF --> BIN
    
    style CMAKE fill:#4CAF50
    style UF2 fill:#2196F3
```

## Configuration Parameters

| Parameter | Location | Default | Description |
|-----------|----------|---------|-------------|
| Control Loop Rate | MotorsAgent | 100 Hz | PID update frequency |
| IMU Rate | ImuAgent | 50 Hz | IMU reading frequency |
| PID Gains (Kp, Ki, Kd) | MotorPID | Tuned | Per-motor PID parameters |
| Wheel Radius | DDD | 0.047 m | For odometry calculation |
| Wheel Base | DDD | 0.220 m | For odometry calculation |
| Encoder CPR | MotorMgr | 1440 | Counts per revolution |
| PWM Frequency | PWMManager | 20 kHz | Motor PWM frequency |

## Performance Characteristics

- **Control Loop**: 100 Hz (10 ms period)
- **IMU Reading**: 50 Hz (20 ms period)
- **Odometry Publishing**: 50 Hz
- **Joint States Publishing**: 100 Hz
- **USB Latency**: < 2 ms
- **PID Computation**: < 0.5 ms
- **Total CPU Usage**: ~60% (with all agents)

## Error Handling

| Error Condition | Detection | Action | Recovery |
|----------------|-----------|--------|----------|
| micro-ROS connection lost | Timeout | Log error, continue operation | Auto-reconnect |
| Encoder read failure | Invalid value | Use last valid value | Retry next cycle |
| IMU read failure | SPI error | Skip publish | Retry next cycle |
| Motor stall | Zero velocity despite command | Reduce PWM | Automatic |
| Watchdog timeout | No host communication | Stop motors | Wait for reconnect |

## Dependencies

- **Pico SDK**: Hardware abstraction, USB, GPIO
- **FreeRTOS**: Real-time task scheduling
- **micro-ROS**: ROS2 communication
- **Eigen**: Linear algebra for odometry

## Build Commands

```bash
# Debug build
cd firmware && make build

# Release build
cd firmware && make build_release

# Flash to Pico
make flash

# Monitor output
./monitor_firmware.sh

# Run tests
cd tests && mkdir -p build && cd build
cmake .. && make && ctest
```

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Firmware won't flash | Pico not in BOOTSEL mode | Hold BOOTSEL while connecting |
| No USB device | Driver issue | Check /dev/ttyACM* exists |
| Motors don't respond | PWM not initialized | Check pin configuration |
| Encoders not counting | Interrupt not working | Verify encoder connections |
| IMU data all zeros | SPI not working | Check SPI wiring |
| High CPU usage | Too many agents | Reduce agent frequencies |
