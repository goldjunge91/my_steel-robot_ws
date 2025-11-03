---
title: Firmware Architektur
summary: Übersicht der FreeRTOS/micro-ROS Firmware für den Raspberry Pi Pico
description: Visuelle Darstellung von Task-Topologie, Datenpfaden und Abläufen zwischen Firmware und ROS 2 Host
keywords: firmware, freertos, micro-ros, pico, architektur
author: goldjunge91
order: 3
---
# Firmware Architecture

Die Firmware bildet die Hardware-Ebene des Roboters und kapselt alle zeitkritischen Aufgaben.

## Systemübersicht

```mermaid
graph TB
    subgraph "FreeRTOS"
        SCHED[Scheduler]
        TASK100[Main Loop\n100 Hz]
        TASK50[Sensor Loop\n50 Hz]
    end

    subgraph "micro-ROS"
        BRIDGE[micro-ROS Client]
        PUB[Publisher]
        SUB[Subscriber]
    end

    subgraph "Agenten"
        MOTORS[Motors Agent]
        ODOM[Odometry Agent]
        IMU[IMU Agent]
        TOF[ToF Agent]
        HEART[Status Agent]
    end

    subgraph "HAL"
        PWM[PWM Manager]
        ENC[Encoder Manager]
        SPI[SPI Driver]
        I2C[I²C Driver]
    end

    TASK100 --> MOTORS --> PWM
    TASK100 --> ODOM --> ENC
    TASK50 --> IMU --> SPI
    TASK50 --> TOF --> I2C
    HEART --> BRIDGE
    MOTORS --> BRIDGE
    ODOM --> BRIDGE
    IMU --> BRIDGE
    BRIDGE -->|USB| Host[ROS 2 Host]
```

## Configuration Parameters

!!! note "Tunable Parameters"
    These parameters can be adjusted for your specific hardware configuration and performance requirements.

| Parameter | Location | Default | Description |
|-----------|----------|---------|-------------|
| Control Loop Rate | MotorsAgent | 100 Hz | PID update frequency |
| IMU Rate | ImuAgent | 50 Hz | IMU reading frequency |
| PID Gains (Kp, Ki, Kd) | MotorPID | Tuned | Per-motor PID parameters |
| Wheel Radius | DDD | 0.047 m | For odometry calculation |
| Wheel Base | DDD | 0.220 m | For odometry calculation |
| Encoder CPR | MotorMgr | 1440 | Counts per revolution |
| PWM Frequency | PWMManager | 20 kHz | Motor PWM frequency |

### Systemübersicht-Diagramme

#### Komponenten-Architektur

[![Komponenten-Diagramm](https://img.plantuml.biz/plantuml/dsvg/RP9DRi9038NtSmgBLQ8gBAgkmg842RMgG20YbKqsWPZ4Ol2W6GVQ7g07w1bwazwa9-aa_5RjSlIU_VpPqtUIMwatMWc9HaW5QxGrbowtoue4rr9R26QJz1PshWlJ-HY6oqhUa2HKggrMj8AWarpy-xpIzvqGf4BsocGhP1YIEED4QtIGroj0Ojp0mEIsw8GA89xbRhj0QrzXId2NSHiPF59PHIFSeGuple6n4MEZPeXWPFP7eUVBsGY2zMRn3u2fDGkDcnX6nFKBa1DO-yJpwD4axHwCyt7qH29euVlz0sfsUUsdG1ZW7ak_XyQ6NBG12hN32z300psWM6JsuLVy1w06estwXwINyPIMfXMlYCvx_GVesLX1ql6PO6r9QDskikbrf6aYaKNdJb2wa1vX4dQdk_xcuO9MIiL35hUia0fly50dFYXn__LPQUdvGvkjwdk__oe_)](https://editor.plantuml.com/uml/RP9DRi9038NtSmgBLQ8gBAgkmg842RMgG20YbKqsWPZ4Ol2W6GVQ7g07w1bwazwa9-aa_5RjSlIU_VpPqtUIMwatMWc9HaW5QxGrbowtoue4rr9R26QJz1PshWlJ-HY6oqhUa2HKggrMj8AWarpy-xpIzvqGf4BsocGhP1YIEED4QtIGroj0Ojp0mEIsw8GA89xbRhj0QrzXId2NSHiPF59PHIFSeGuple6n4MEZPeXWPFP7eUVBsGY2zMRn3u2fDGkDcnX6nFKBa1DO-yJpwD4axHwCyt7qH29euVlz0sfsUUsdG1ZW7ak_XyQ6NBG12hN32z300psWM6JsuLVy1w06estwXwINyPIMfXMlYCvx_GVesLX1ql6PO6r9QDskikbrf6aYaKNdJb2wa1vX4dQdk_xcuO9MIiL35hUia0fly50dFYXn__LPQUdvGvkjwdk__oe_)

#### Datenfluss & Kommunikation

[![Datenfluss-Diagramm](https://img.plantuml.biz/plantuml/dsvg/RPBFQW8n4CRlUOh1NboAj2Zqe8UghAtGjPIkNaelqPtD3iQ99DcBp-C3z04yrHThzWyenLlc-y9ylqncxBoqljnfuSbhX1JP6KjRoCyd2saoMGXHNn6KGxVJs1VpkHE1Bv23bL0y-Un40c1O7qTmbv0g_5IN64Gs7i5MGYW0xc5k2eGFUpuUo9497Vfnr5g3fyVV7vYiCzniIrQjkcBSxNFYJDEc08Kgn2RXs3JimPjn7ZaKo5aT9r0xhB6NX3doLkxGoWRnMe5kwj6YULYQbsXqY_NrV6BdIQ5jXLc8HHZ4lLWscHN059L5FdXqa5PSEUgijMfEoVdvg3Mf_uo1SAMzbMvJIAqRm2lX9bAAXGNLShKfNoM4SI7Bykhx6kkzj30FiqFBvD1kaRcPx0M4Cyg56CxOXrYaBFZ3Rwx_4gHsC9MENDFOcdNfgdVsuzv2xlV4QXJMnEp5EBIfjkaF)](https://editor.plantuml.com/uml/RPBFQW8n4CRlUOh1NboAj2Zqe8UghAtGjPIkNaelqPtD3iQ99DcBp-C3z04yrHThzWyenLlc-y9ylqncxBoqljnfuSbhX1JP6KjRoCyd2saoMGXHNn6KGxVJs1VpkHE1Bv23bL0y-Un40c1O7qTmbv0g_5IN64Gs7i5MGYW0xc5k2eGFUpuUo9497Vfnr5g3fyVV7vYiCzniIrQjkcBSxNFYJDEc08Kgn2RXs3JimPjn7ZaKo5aT9r0xhB6NX3doLkxGoWRnMe5kwj6YULYQbsXqY_NrV6BdIQ5jXLc8HHZ4lLWscHN059L5FdXqa5PSEUgijMfEoVdvg3Mf_uo1SAMzbMvJIAqRm2lX9bAAXGNLShKfNoM4SI7Bykhx6kkzj30FiqFBvD1kaRcPx0M4Cyg56CxOXrYaBFZ3Rwx_4gHsC9MENDFOcdNfgdVsuzv2xlV4QXJMnEp5EBIfjkaF)

#### Deployment-Architektur

[![Deployment-Diagramm](https://img.plantuml.biz/plantuml/dsvg/LP6nJiCm48PtFuNLgHb25wPse4L228aAYOgjYualpidEkN8kWFeyVGOcjhmObpfLORt_z_z_lbjtR1BtRL2C741lY1F2U-SZ7U70UdwhNuUJXLGeTROghFLyTW-Vu7fodKpkPeNc1aZ6JGoHOhkdUMk-RlJMqF3z8Ncf7auDEW_8nQnGMYzVMaAB2JnZ97CjXfMevuetOaWkzMJ_5BXWc1TFP6DCvo29sa9bg6_Bp3c-Xz21c7kIFBhOniQ_7h9Ogyb9M3LSNkbhqwGZz4weAOWsGJz20F8CUg77NQJ_r7LCFbVjFDfonv2RnhaspY1bTLDvIAqXAfybJPOuuUI0pAMM3yHTrG1QCIptTFdaLAZhuIVfl8Toup0W3oTUoOdYGQqbOxpeBm00)](https://editor.plantuml.com/uml/LP6nJiCm48PtFuNLgHb25wPse4L228aAYOgjYualpidEkN8kWFeyVGOcjhmObpfLORt_z_z_lbjtR1BtRL2C741lY1F2U-SZ7U70UdwhNuUJXLGeTROghFLyTW-Vu7fodKpkPeNc1aZ6JGoHOhkdUMk-RlJMqF3z8Ncf7auDEW_8nQnGMYzVMaAB2JnZ97CjXfMevuetOaWkzMJ_5BXWc1TFP6DCvo29sa9bg6_Bp3c-Xz21c7kIFBhOniQ_7h9Ogyb9M3LSNkbhqwGZz4weAOWsGJz20F8CUg77NQJ_r7LCFbVjFDfonv2RnhaspY1bTLDvIAqXAfybJPOuuUI0pAMM3yHTrG1QCIptTFdaLAZhuIVfl8Toup0W3oTUoOdYGQqbOxpeBm00)
