# my_steel Robot Firmware

Raspberry Pi Pico firmware for the my_steel robot, implementing motor control, sensor integration, and micro-ROS communication.

## Architecture

The firmware uses a modular agent-based architecture built on FreeRTOS:

- **uRosBridge**: Singleton managing micro-ROS communication over USB
- **MotorsAgent**: PID-controlled motor management with encoder feedback
- **HCSR04Agent**: Ultrasonic distance sensor management
- **ImuAgent**: IMU (ICM20948) data acquisition via SPI
- **Vl6180xAgent**: Time-of-Flight sensor via I2C
- **DDD**: Main robot control agent handling odometry and cmd_vel

## Hardware Configuration

### Pin Mapping

| Component           | Pin  | Function             |
| ------------------- | ---- | -------------------- |
| Left Motor PWM CW   | GP9  | Motor control        |
| Left Motor PWM CCW  | GP8  | Motor control        |
| Left Encoder A      | GP14 | Encoder feedback     |
| Left Encoder B      | GP15 | Encoder feedback     |
| Right Motor PWM CW  | GP6  | Motor control        |
| Right Motor PWM CCW | GP7  | Motor control        |
| Right Encoder A     | GP12 | Encoder feedback     |
| Right Encoder B     | GP13 | Encoder feedback     |
| IMU CS              | GP17 | SPI chip select      |
| IMU SCK             | GP18 | SPI clock            |
| IMU MOSI            | GP19 | SPI data out         |
| IMU MISO            | GP16 | SPI data in          |
| VL6180X SDA         | GP2  | I2C data             |
| VL6180X SCL         | GP3  | I2C clock            |
| Status LED          | GP2  | Connection indicator |
| Debug LED           | GP3  | Activity indicator   |

### Communication

- **USB**: micro-ROS transport (primary communication with ROS2)
- **UART0 (GP0/GP1)**: Debug output at 115200 baud

## Building

### Prerequisites

- Raspberry Pi Pico SDK
- CMake 3.13+
- GCC ARM toolchain
- FreeRTOS (included as submodule)
- micro-ROS for Pico (included as submodule)

### Build Commands

```bash
# Debug build
make build

# Release build  
make build_release

# Clean build artifacts
make clean
```

### Using Just (from workspace root)

```bash
# Build firmware
just build-firmware

# Build release version
just build-firmware-release

# Flash to Pico
just flash-firmware
```

## Flashing

### Method 1: USB Mass Storage (BOOTSEL)

1. Hold BOOTSEL button while connecting Pico to USB
2. Pico appears as USB drive "RPI-RP2"
3. Copy `build/src/my_firmware.uf2` to the drive
4. Pico automatically reboots with new firmware

```bash
# Automated flashing (if mounted)
make flash
```

### Method 2: picotool

```bash
# Install picotool first
sudo apt install picotool

# Flash and reboot
picotool load -f build/src/my_firmware.uf2
picotool reboot
```

## Testing & Debugging

### Monitor Debug Output

```bash
# Monitor UART debug output
just monitor-firmware

# Or manually
screen /dev/ttyAMA0 115200
```

### Test micro-ROS Connection

```bash
# Comprehensive connection test
just test-firmware

# Manual micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v
```

### Expected ROS Topics

When firmware is running and micro-ROS agent is connected:

```bash
# Published by Pico
/pico_count        # std_msgs/Int32 - heartbeat counter
/odom              # nav_msgs/Odometry - robot odometry
/imu/data          # sensor_msgs/Imu - IMU data
/range_front       # sensor_msgs/Range - front distance
/range_back        # sensor_msgs/Range - back distance

# Subscribed by Pico  
/cmd_vel           # geometry_msgs/Twist - velocity commands
```

## Configuration

### Robot Parameters

Edit `firmware/src/DDD.h` for robot-specific parameters:

```cpp
#define WHEEL_RADIUS 0.065      // meters
#define WHEEL_DEPTH 0.055       // meters  
#define WHEELS_SEP 0.204        // meters
#define WHEELS_OFFSET 0.010     // meters
```

### PID Tuning

Edit `firmware/src/main.cpp` for motor PID parameters:

```cpp
#define KP 0.55    // Proportional gain
#define KI 0.019   // Integral gain  
#define KD 0.24    // Derivative gain
```

## Troubleshooting

### Common Issues

1. **No USB device detected**
   - Check USB cable (data cable, not power-only)
   - Verify Pico is not in BOOTSEL mode
   - Try different USB port

2. **micro-ROS agent connection fails**
   - Check serial port permissions: `sudo usermod -a -G dialout $USER`
   - Verify correct baud rate (115200)
   - Ensure no other processes using the port

3. **No debug output on UART**
   - Check UART wiring (GP0=TX, GP1=RX)
   - Verify baud rate (115200)
   - Try different terminal program

4. **Motors not responding**
   - Check motor driver connections
   - Verify power supply (motors need separate power)
   - Check encoder wiring

### Debug Logging

The firmware provides extensive UART logging:

```
[HEARTBEAT] Uptime: 5000 ms
[STATE] Changed from 0 to 1
[DIAG] State=CONNECTED, Queue: 0/128, SessionReady=1
[PUB] Attempting publish...
[TIMER PUB] pico_count=42 publish_ret=0 count=5
```

## Development

### Adding New Sensors

1. Create new agent class inheriting from `Agent`
2. Implement hardware initialization in constructor
3. Add to main task initialization in `main.cpp`
4. Register with `DDD` agent for ROS integration

### Modifying ROS Interface

1. Edit `DDD.h` and `DDD.cpp` for new topics
2. Update entity counts in `getCount()` and `getHandles()`
3. Add publishers/subscribers in `createEntities()`
4. Handle messages in `handleSubscriptionMsg()`

## Performance Notes

- Main loop runs at ~100Hz
- micro-ROS publishing is queued (128 message buffer)
- IMU sampling at 100Hz
- Motor PID control at 50Hz
- Sensor readings at 10Hz

## Dependencies

- **FreeRTOS**: Real-time task scheduling
- **micro-ROS**: ROS2 communication over USB
- **Pico SDK**: Hardware abstraction
- **Eigen**: Linear algebra for odometry calculations









# Firmware Release Process

## 🚀 Schneller Release

```bash
./create_release.sh v1.0.0
```

Das Script macht automatisch:
1. ✅ Build Release-Version
2. ✅ Erstellt `.uf2` in `releases/`
3. ✅ Erstellt Git Tag
4. ✅ Pusht Tag zu GitHub
5. ✅ Erstellt GitHub Release (mit `gh` CLI)

---

## 📦 Manueller Release

### 1. Build Release
```bash
make release VERSION=v1.0.0
```

### 2. Git Tag erstellen
```bash
git tag -a v1.0.0 -m "Release v1.0.0"
git push origin v1.0.0
```

### 3. GitHub Release erstellen
- Gehe zu: https://github.com/goldjunge91/my_steel-robot_ws/releases/new
- Tag: `v1.0.0`
- Upload: `releases/my_firmware_v1.0.0.uf2`

---

## 📋 Versioning

**Format:** `vMAJOR.MINOR.PATCH`

- `v1.0.0` - Erster stabiler Release
- `v1.1.0` - Neue Features
- `v1.1.1` - Bugfixes

---

## 🛠️ GitHub CLI installieren (optional)

```bash
# Ubuntu/Debian
sudo apt install gh

# Login
gh auth login
```

Mit `gh` CLI wird der Release automatisch hochgeladen!

---

## 📁 Warum nicht in Git?

**Binaries (.uf2) gehören NICHT ins Repository!**

✅ Vorteile von GitHub Releases:
- Keine Repo-Bloat
- Schnellere Clones
- Offizielle Release-Artefakte
- Download-Statistiken
- Release Notes

❌ Nachteile von Git-versioned Binaries:
- Repository wird riesig
- Langsame Clones
- Verschmutzt Git-Historie
