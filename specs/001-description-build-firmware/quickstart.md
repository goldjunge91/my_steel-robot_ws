# Quickstart: Pico Firmware Build and Test

**Feature**: Port legacy STM32 firmware to Raspberry Pi Pico (lpico)  
**Date**: 2025-09-23  
**Target**: Developers building and testing the ported firmware

## Prerequisites

### Hardware Required
- Raspberry Pi Pico (RP2040-based)
- USB-A to USB-C cable
- 4x TB6612FNG motor driver breakouts
- 4x DC motors with encoders
- 2x ESC modules for shooter
- Breadboard and jumper wires
- (Optional) Logic analyzer for debugging

### Software Required
- Git
- CMake (3.13 or later)
- Pico SDK (1.5.x)
- ARM GCC cross-compiler
- `picotool` (for flashing)
- Serial terminal (Arduino IDE Serial Monitor, PuTTY, etc.)

## Quick Setup Commands

### 1. Environment Setup
```bash
# Clone the repository (if not already done)
git clone https://github.com/goldjunge91/my_steel-robot_ws.git
cd my_steel-robot_ws

# Set Pico SDK path (adjust for your installation)
export PICO_SDK_PATH=/path/to/pico-sdk

# Navigate to firmware directory
cd src/robot_firmware
```

### 2. Build the Firmware
```bash
# Create and enter lpico directory
mkdir -p lpico
cd lpico

# Copy source files from legacy_stm32 (preserving filenames)
cp -r ../legacy_stm32/src/* ./src/
cp -r ../legacy_stm32/include/* ./include/

# Build the firmware
mkdir build && cd build
cmake ..
make -j4

# Verify build output
ls -la *.uf2    # Should show lpico.uf2 file
```

### 3. Flash to Pico
```bash
# Method 1: Bootloader mode (recommended for first flash)
# 1. Hold BOOTSEL button on Pico
# 2. Connect USB cable while holding BOOTSEL
# 3. Release BOOTSEL when Pico appears as USB drive
# 4. Copy firmware file
cp lpico.uf2 /media/RPI-RP2/   # Linux
# or drag-and-drop to RPI-RP2 drive on Windows/Mac

# Method 2: Using picotool (for subsequent flashes)
picotool load lpico.uf2 --force --execute
```

### 4. Verify Communication
```bash
# Find the USB CDC device
ls /dev/ttyACM*    # Linux: usually /dev/ttyACM0
# Windows: Check Device Manager for COM port

# Test basic communication
screen /dev/ttyACM0   # Linux
# or use serial terminal of choice

# Send test commands
> STATUS
< MOTORS FL:0 FR:0 RL:0 RR:0
< SHOOTER ESC1:0 ESC2:0 GEAR:0  
< SYSTEM UPTIME:1234 WATCHDOG:0 FREQ:1000
< READY

> VERSION
< VERSION lpico-1.0.0-001-description-build-firmware
```

## Hardware Connection Guide

### TB6612 Motor Driver Wiring
```
| Pico Pin | TB6612 Pin | Function                 |
| -------- | ---------- | ------------------------ |
| GPIO 28  | STBY       | Standby (both drivers)   |
| GPIO 18  | AIN1       | Front-left direction 1   |
| GPIO 19  | AIN2       | Front-left direction 2   |
| GPIO 2   | PWMA       | Front-left speed         |
| GPIO 17  | BIN1       | Front-right direction 1  |
| GPIO 20  | BIN2       | Front-right direction 2  |
| GPIO 3   | PWMB       | Front-right speed        |
| ...      | ...        | (repeat for rear motors) |
```

### Encoder Connections
```
| Pico Pin | Encoder  | Function                   |
| -------- | -------- | -------------------------- |
| GPIO 8   | FL_ENC_A | Front-left encoder A       |
| GPIO 9   | FL_ENC_B | Front-left encoder B       |
| GPIO 10  | FR_ENC_A | Front-right encoder A      |
| GPIO 11  | FR_ENC_B | Front-right encoder B      |
| ...      | ...      | (repeat for rear encoders) |
```

### Shooter Connections  
```
| Pico Pin | Function          |
| -------- | ----------------- |
| GPIO 14  | ESC1 signal       |
| GPIO 15  | ESC2 signal       |
| GPIO 16  | Gear servo signal |
```

## Basic Testing Procedures

### Test 1: System Startup
1. Connect Pico via USB
2. Open serial terminal
3. Send `STATUS` command
4. Verify all systems report `0` (safe state)
5. Check `WATCHDOG:0` (not triggered)

### Test 2: Motor Control
```bash
# Test individual motor (low speed)
> M FL:50 FR:0 RL:0 RR:0
< OK

# Verify motor movement (front-left should turn slowly)
# Send stop command
> STOP  
< STOPPED

# Test all motors (very low speed for safety)
> M FL:30 FR:30 RL:30 RR:30
< OK

# Stop after 2 seconds
> STOP
< STOPPED
```

### Test 3: Shooter Control
```bash
# Enable ESC1 only
> ESC ESC1:1 ESC2:0
< OK

# Test gear movement
> GEAR:1
< OK

# Wait 1 second, retract gear
> GEAR:0  
< OK

# Disable ESCs
> ESC ESC1:0 ESC2:0
< OK
```

### Test 4: Safety Systems
```bash
# Test watchdog by disconnecting serial for >300ms
# 1. Send motor command
> M FL:100 FR:100 RL:100 RR:100
< OK

# 2. Disconnect serial terminal
# 3. Wait 1 second
# 4. Reconnect terminal
# 5. Check status
> STATUS
< MOTORS FL:0 FR:0 RL:0 RR:0    # Should be stopped
< SYSTEM UPTIME:5678 WATCHDOG:1 FREQ:1000  # Watchdog triggered
```

## Troubleshooting

### Build Issues
```bash
# Missing Pico SDK
export PICO_SDK_PATH=/path/to/pico-sdk

# CMake not finding compiler
sudo apt install gcc-arm-none-eabi

# Permission issues
sudo usermod -a -G dialout $USER  # Linux serial access
```

### Communication Issues
```bash
# Device not found
lsusb | grep "2e8a"  # Should show Raspberry Pi device

# No response from commands
# 1. Check baud rate (ignored for USB CDC, but try 115200)
# 2. Verify line endings (\n vs \r\n)
# 3. Try shorter commands first

# Commands rejected
# 1. Check parameter ranges (motors: -255 to +255)
# 2. Verify command syntax (case-sensitive)
# 3. Check for typos in parameter names
```

### Hardware Issues
```bash
# Motors not moving
# 1. Check TB6612 power supply connections
# 2. Verify STBY pin is HIGH (GPIO 28)
# 3. Test with multimeter on motor outputs
# 4. Check motor connections and polarity

# Erratic encoder readings
# 1. Check encoder power supply (3.3V/5V)
# 2. Verify signal integrity with scope
# 3. Check for electromagnetic interference
# 4. Test with slower motor speeds
```

## Performance Validation

### Expected Performance
- Command response time: <10ms
- Motor control loop: 1000Hz (1ms period)  
- USB CDC latency: <5ms
- Encoder update rate: Variable (based on motor speed)

### Monitoring Commands
```bash
# Check control loop frequency
> STATUS
< SYSTEM UPTIME:12345 WATCHDOG:0 FREQ:1000  # Should be ~1000Hz

# Monitor in real-time (Linux)
while true; do echo "STATUS"; sleep 0.1; done | nc localhost 12345
```

## Notes
- Document exact pico-sdk commit or tag used in final README for reproducible builds.
- Exact STM32 function names preserved for constitution compliance
- Watchdog timeout fixed at 300ms per constitution requirements

---
*Quickstart Guide v1.0 - Get running in 15 minutes*
