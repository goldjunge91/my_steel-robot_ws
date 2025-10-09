# System Architecture Overview

## Complete System Diagram

```mermaid
graph TB
    subgraph "User Interface"
        KB[Keyboard Teleop]
        JOY[Joystick]
        RVIZ[RViz2]
        FOX[Foxglove]
    end
    
    subgraph "Raspberry Pi 4B - ROS2 Humble"
        direction TB
        
        subgraph "Navigation Stack"
            NAV2[Nav2<br/>Path Planning]
            SLAM[SLAM<br/>Cartographer]
            LOC[Localization<br/>robot_localization]
        end
        
        subgraph "Control Layer"
            CM[Controller Manager<br/>ros2_control]
            MDC[Mecanum Drive<br/>Controller]
            JSB[Joint State<br/>Broadcaster]
            IB[IMU<br/>Broadcaster]
        end
        
        subgraph "Hardware Interface"
            HWI[RobotSystem<br/>Hardware Interface]
            IMU_HW[RobotImuSensor<br/>IMU Interface]
        end
        
        subgraph "Communication"
            MRA[micro-ROS Agent<br/>Topic Remapping]
        end
        
        subgraph "Perception"
            VIS[Vision<br/>Face Detection]
            LIDAR_PROC[LiDAR Processing]
        end
    end
    
    subgraph "Raspberry Pi Pico - FreeRTOS"
        direction TB
        
        subgraph "micro-ROS Client"
            UROS[uRosBridge<br/>Publishers/Subscribers]
        end
        
        subgraph "Control Agents"
            DDD[DDD Agent<br/>Odometry]
            MOTORS[Motors Agent<br/>PID Control]
        end
        
        subgraph "Sensor Agents"
            IMU_A[IMU Agent<br/>ICM20948]
            TOF_A[ToF Agent<br/>VL6180X]
        end
        
        subgraph "Hardware Abstraction"
            PWM[PWM Manager]
            ENC[Encoders]
            SPI[SPI Driver]
            I2C[I2C Driver]
        end
    end
    
    subgraph "Hardware"
        MOTORS_HW[4x DC Motors<br/>Mecanum Wheels]
        IMU_SENSOR[ICM20948<br/>9-DOF IMU]
        TOF_SENSOR[VL6180X<br/>ToF Sensor]
        LIDAR_HW[RPLiDAR<br/>2D Scanner]
        CAM[USB Camera]
    end
    
    KB --> MDC
    JOY --> MDC
    NAV2 --> MDC
    
    MDC --> CM
    CM --> HWI
    CM --> IMU_HW
    
    HWI <--> MRA
    IMU_HW <--> MRA
    
    MRA <-->|USB Serial| UROS
    
    UROS <--> DDD
    UROS <--> MOTORS
    UROS <--> IMU_A
    UROS <--> TOF_A
    
    DDD --> MOTORS
    MOTORS --> PWM
    MOTORS --> ENC
    IMU_A --> SPI
    TOF_A --> I2C
    
    PWM --> MOTORS_HW
    ENC --> MOTORS_HW
    SPI --> IMU_SENSOR
    I2C --> TOF_SENSOR
    
    LIDAR_HW --> LIDAR_PROC
    CAM --> VIS
    
    LIDAR_PROC --> SLAM
    SLAM --> LOC
    LOC --> NAV2
    
    HWI --> JSB
    IMU_HW --> IB
    JSB --> RVIZ
    IB --> RVIZ
    MDC --> RVIZ
    
    RVIZ --> FOX
    
    style NAV2 fill:#4CAF50
    style CM fill:#9C27B0
    style HWI fill:#FF9800
    style MRA fill:#2196F3
    style UROS fill:#4CAF50
    style DDD fill:#2196F3
```

## Layered Architecture

```mermaid
graph TB
    subgraph "Layer 1: User Interface"
        UI[Teleop, RViz, Foxglove, Nav2]
    end
    
    subgraph "Layer 2: ROS2 Application"
        APP[Navigation, SLAM, Vision, Localization]
    end
    
    subgraph "Layer 3: ROS2 Control"
        CTRL[Controllers: Mecanum Drive, Broadcasters]
    end
    
    subgraph "Layer 4: Hardware Interface"
        HW_IF[ros2_control Hardware Interface]
    end
    
    subgraph "Layer 5: Communication Bridge"
        BRIDGE[micro-ROS Agent with Topic Remapping]
    end
    
    subgraph "Layer 6: Firmware"
        FW[FreeRTOS + micro-ROS Client]
    end
    
    subgraph "Layer 7: Hardware Abstraction"
        HAL[PWM, Encoders, SPI, I2C Drivers]
    end
    
    subgraph "Layer 8: Physical Hardware"
        HW[Motors, Sensors, Actuators]
    end
    
    UI --> APP
    APP --> CTRL
    CTRL --> HW_IF
    HW_IF --> BRIDGE
    BRIDGE --> FW
    FW --> HAL
    HAL --> HW
    
    style UI fill:#E1BEE7
    style APP fill:#C5CAE9
    style CTRL fill:#B2DFDB
    style HW_IF fill:#C8E6C9
    style BRIDGE fill:#FFF9C4
    style FW fill:#FFCCBC
    style HAL fill:#D7CCC8
    style HW fill:#CFD8DC
```

## Data Flow: Command to Motion

```mermaid
sequenceDiagram
    participant User
    participant Teleop
    participant Controller
    participant HWI as Hardware Interface
    participant Agent as micro-ROS Agent
    participant Pico as Pico Firmware
    participant Motors
    
    User->>Teleop: Press key
    Teleop->>Controller: /cmd_vel (Twist)<br/>vx=0.1, vy=0, ω=0
    
    Note over Controller: Inverse Kinematics
    Controller->>Controller: Calculate wheel velocities
    
    Controller->>HWI: Write command interfaces<br/>4x wheel velocities
    
    Note over HWI: Forward Kinematics
    HWI->>HWI: Convert to Twist
    HWI->>Agent: Publish /cmd_vel (Twist)
    
    Agent->>Agent: Remap /cmd_vel → /rt/cmd_vel
    Agent->>Pico: Forward via USB serial
    
    Note over Pico: Inverse Kinematics
    Pico->>Pico: Calculate wheel velocities
    Pico->>Pico: PID control
    Pico->>Motors: Set PWM duty cycles
    
    Motors->>Motors: Rotate wheels
    
    Note over Motors,User: Robot moves forward
```

## Data Flow: Sensor to State

```mermaid
sequenceDiagram
    participant Encoders
    participant Pico as Pico Firmware
    participant Agent as micro-ROS Agent
    participant HWI as Hardware Interface
    participant Controller
    participant RViz
    
    loop Every 10ms
        Encoders->>Pico: Encoder counts
        Pico->>Pico: Calculate positions & velocities
        Pico->>Pico: Calculate odometry
        
        Pico->>Agent: Publish /rt/joint_states
        Pico->>Agent: Publish /rt/odom
        
        Agent->>Agent: Remap topics
        Agent->>HWI: /joint_states
        Agent->>RViz: /odom
        
        HWI->>HWI: Update state interfaces
        HWI->>Controller: Read state interfaces
        
        Controller->>Controller: Calculate control
        Controller->>RViz: Publish /odometry/wheels
    end
```

## Topic Flow Diagram

```mermaid
flowchart LR
    subgraph "User Input"
        KB[Keyboard]
        JOY[Joystick]
        NAV[Nav2]
    end
    
    subgraph "Standard ROS2 Topics"
        CMD[/cmd_vel<br/>Twist]
        JS[/joint_states<br/>JointState]
        IMU[/imu/data_raw<br/>Imu]
        ODOM[/odom<br/>Odometry]
        ODOM_W[/odometry/wheels<br/>Odometry]
        SCAN[/scan<br/>LaserScan]
    end
    
    subgraph "micro-ROS Agent Remapping"
        RT_CMD[/rt/cmd_vel]
        RT_JS[/rt/joint_states]
        RT_IMU[/rt/imu/data_raw]
        RT_ODOM[/rt/odom]
    end
    
    subgraph "Firmware Topics"
        FW[Pico Firmware<br/>Publishers/Subscribers]
    end
    
    KB --> CMD
    JOY --> CMD
    NAV --> CMD
    
    CMD --> RT_CMD
    RT_CMD --> FW
    
    FW --> RT_JS
    FW --> RT_IMU
    FW --> RT_ODOM
    
    RT_JS --> JS
    RT_IMU --> IMU
    RT_ODOM --> ODOM
    
    JS --> ODOM_W
    
    style CMD fill:#4CAF50
    style JS fill:#2196F3
    style IMU fill:#FF9800
    style ODOM fill:#9C27B0
```

## Package Dependencies

```mermaid
graph TB
    subgraph "Application Packages"
        RB[robot_bringup]
        RV[robot_vision]
        RNL[robot_nerf_launcher]
        RLT[robot_localization_tool]
    end
    
    subgraph "Core Packages"
        RC[robot_controller]
        RHI[robot_hardware_interfaces]
        RD[robot_description]
        RCS[robot_controllers]
    end
    
    subgraph "Simulation"
        RG[robot_gazebo]
        RGW[robot_gz_worlds]
    end
    
    subgraph "External"
        MRA[micro-ROS-Agent]
        OMX[open_manipulator_x]
        DYN[dynamixel_hardware_interface]
    end
    
    subgraph "Firmware"
        FW[firmware/]
    end
    
    RB --> RC
    RB --> RHI
    RB --> RD
    RB --> MRA
    
    RC --> RCS
    RC --> RHI
    
    RHI --> RD
    
    RG --> RD
    RG --> RGW
    
    RLT --> RD
    
    MRA --> FW
    
    style RB fill:#4CAF50
    style RHI fill:#FF9800
    style MRA fill:#2196F3
    style FW fill:#9C27B0
```

## Hardware Architecture

```mermaid
graph TB
    subgraph "Compute"
        RPI4[Raspberry Pi 4B<br/>4GB RAM<br/>Quad-core ARM]
        PICO[Raspberry Pi Pico<br/>264KB RAM<br/>Dual-core ARM]
    end
    
    subgraph "Sensors"
        IMU[ICM20948<br/>9-DOF IMU<br/>SPI]
        TOF[VL6180X<br/>ToF Sensor<br/>I2C]
        LIDAR[RPLiDAR A1<br/>2D Scanner<br/>USB]
        CAM[USB Camera<br/>640x480]
    end
    
    subgraph "Actuators"
        M1[Motor FL<br/>with Encoder]
        M2[Motor FR<br/>with Encoder]
        M3[Motor RL<br/>with Encoder]
        M4[Motor RR<br/>with Encoder]
    end
    
    subgraph "Power"
        BATT[LiPo Battery<br/>11.1V 3S]
        BEC[BEC 5V<br/>for Pi4]
        REG[3.3V Regulator<br/>for Pico]
    end
    
    subgraph "Communication"
        USB[USB Connection<br/>Pi4 ↔ Pico]
    end
    
    RPI4 <-->|USB Serial| PICO
    RPI4 <-->|USB| LIDAR
    RPI4 <-->|USB| CAM
    
    PICO <-->|SPI0| IMU
    PICO <-->|I2C1| TOF
    PICO -->|PWM| M1
    PICO -->|PWM| M2
    PICO -->|PWM| M3
    PICO -->|PWM| M4
    PICO <-->|GPIO| M1
    PICO <-->|GPIO| M2
    PICO <-->|GPIO| M3
    PICO <-->|GPIO| M4
    
    BATT --> BEC
    BATT --> M1
    BATT --> M2
    BATT --> M3
    BATT --> M4
    BEC --> RPI4
    BEC --> REG
    REG --> PICO
    
    style RPI4 fill:#4CAF50
    style PICO fill:#2196F3
    style BATT fill:#FF9800
```

## Network Architecture

```mermaid
graph TB
    subgraph "Robot Network"
        RPI[Raspberry Pi 4B<br/>192.168.1.100]
        PICO[Pico via USB<br/>/dev/ttyACM0]
    end
    
    subgraph "Development PC"
        PC[Development PC<br/>192.168.1.10]
    end
    
    subgraph "Communication Protocols"
        DDS[ROS2 DDS<br/>UDP Multicast]
        USB_CDC[USB CDC Serial<br/>115200 baud]
        SSH[SSH<br/>Remote Access]
    end
    
    RPI <-->|DDS| PC
    RPI <-->|USB Serial| PICO
    PC <-->|SSH| RPI
    
    RPI -.->|WiFi/Ethernet| DDS
    PC -.->|WiFi/Ethernet| DDS
    RPI -.->|USB| USB_CDC
    
    style RPI fill:#4CAF50
    style PICO fill:#2196F3
    style PC fill:#FF9800
```

## State Machine: System Lifecycle

```mermaid
stateDiagram-v2
    [*] --> PowerOn: Power applied
    
    PowerOn --> FirmwareBoot: Pico boots
    FirmwareBoot --> WaitingForAgent: micro-ROS initialized
    
    PowerOn --> HostBoot: Pi4 boots
    HostBoot --> LaunchBringup: ROS2 started
    LaunchBringup --> StartAgent: Launch micro-ROS agent
    
    StartAgent --> WaitingForAgent: Agent running
    WaitingForAgent --> Connected: USB connection established
    
    Connected --> HardwareInit: Hardware interface initializing
    HardwareInit --> WaitingForData: Waiting for joint states
    
    WaitingForData --> HardwareActive: First data received
    HardwareActive --> ControllersLoading: Load controllers
    ControllersLoading --> ControllersActive: Controllers activated
    
    ControllersActive --> Operational: System ready
    
    Operational --> Operational: Normal operation
    
    Operational --> Error: Connection lost
    Error --> WaitingForAgent: Reconnect
    
    Operational --> Shutdown: User shutdown
    Shutdown --> [*]
    
    note right of Operational
        - Publishing joint states at 100 Hz
        - Publishing IMU at 50 Hz
        - Publishing odometry at 50 Hz
        - Accepting velocity commands
    end note
```

## Performance Metrics

| Metric | Target | Actual | Notes |
|--------|--------|--------|-------|
| Control Loop Frequency | 100 Hz | 100 Hz | Firmware PID loop |
| Joint State Publishing | 100 Hz | 109.8 Hz | Measured |
| IMU Publishing | 50 Hz | 50 Hz | Firmware |
| Odometry Publishing | 50 Hz | 50 Hz | Firmware |
| Command Latency | < 20 ms | ~15 ms | End-to-end |
| USB Serial Latency | < 2 ms | ~1 ms | Pico ↔ Pi4 |
| Hardware Interface Cycle | < 10 ms | ~8 ms | Read + Write |
| CPU Usage (Pi4) | < 50% | ~35% | With all nodes |
| CPU Usage (Pico) | < 80% | ~60% | With all agents |
| Memory Usage (Pi4) | < 2 GB | ~1.2 GB | ROS2 + nodes |
| Memory Usage (Pico) | < 200 KB | ~180 KB | Firmware |

## Key Design Decisions

### 1. Two-Tier Architecture

**Decision**: Use Raspberry Pi 4B for high-level control and Pico for real-time control

**Rationale**:
- Pi4 provides ROS2 ecosystem and computational power
- Pico provides real-time guarantees for motor control
- Clear separation of concerns
- Cost-effective solution

### 2. micro-ROS Communication

**Decision**: Use micro-ROS over USB serial for Pico ↔ Pi4 communication

**Rationale**:
- Native ROS2 integration
- Standard message types
- Automatic serialization/deserialization
- Topic-based architecture

### 3. Standard Topic Names

**Decision**: Follow REP-105 for topic naming

**Rationale**:
- Compatibility with Nav2, RViz, and other tools
- Clear semantic meaning
- Industry standard
- Easier integration

### 4. ros2_control Framework

**Decision**: Use ros2_control for hardware abstraction

**Rationale**:
- Standard ROS2 approach
- Controller plugins
- Lifecycle management
- Simulation compatibility

### 5. FreeRTOS on Pico

**Decision**: Use FreeRTOS for firmware task scheduling

**Rationale**:
- Real-time guarantees
- Priority-based scheduling
- Well-tested and stable
- Good micro-ROS integration

## Security Considerations

- **USB Serial**: No encryption, physical access required
- **ROS2 DDS**: No authentication by default, use DDS security if needed
- **SSH Access**: Use key-based authentication
- **Firmware Updates**: Require physical access (BOOTSEL mode)
- **Network**: Isolate robot network from public internet

## Scalability

- **Additional Sensors**: Add new agents in firmware, new topics in ROS2
- **Additional Actuators**: Extend hardware interface, add controllers
- **Multiple Robots**: Use ROS2 namespaces and DDS domain IDs
- **Cloud Integration**: Add ROS2 bridge to cloud services
- **Fleet Management**: Use fleet management tools (e.g., RMF)

## Maintenance

- **Firmware Updates**: Flash new .uf2 file via BOOTSEL mode
- **ROS2 Updates**: `colcon build` and restart services
- **Configuration Changes**: Edit YAML files, restart nodes
- **Calibration**: Update parameters in URDF and controller configs
- **Diagnostics**: Use `ros2 topic`, `ros2 control`, and firmware monitor

## References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ros2_control Documentation](https://control.ros.org/)
- [micro-ROS Documentation](https://micro.ros.org/)
- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)
- [REP-105: Coordinate Frames](https://www.ros.org/reps/rep-0105.html)
