#!/bin/bash
# Reflash Firmware Script

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Firmware Reflash Script${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 1. Check if firmware directory exists
if [ ! -d "firmware" ]; then
    echo -e "${RED}✗${NC} firmware/ directory not found!"
    echo "Run this script from the workspace root"
    exit 1
fi

cd firmware

# 2. Clean old build
echo -e "${YELLOW}[1/5] Cleaning old build...${NC}"
if [ -d "build" ]; then
    rm -rf build
    echo -e "${GREEN}✓${NC} Build directory cleaned"
else
    echo -e "${GREEN}✓${NC} No old build found"
fi
echo ""

# 3. Build firmware
echo -e "${YELLOW}[2/5] Building firmware...${NC}"
echo "This may take a minute..."
echo ""

if make build; then
    echo ""
    echo -e "${GREEN}✓${NC} Firmware built successfully"
else
    echo ""
    echo -e "${RED}✗${NC} Build failed!"
    exit 1
fi
echo ""

# 4. Check for Pico in BOOTSEL mode
echo -e "${YELLOW}[3/5] Checking for Pico in BOOTSEL mode...${NC}"
echo ""
echo -e "${YELLOW}⚠ IMPORTANT:${NC}"
echo "  1. Disconnect the Pico from USB"
echo "  2. Hold the BOOTSEL button on the Pico"
echo "  3. While holding BOOTSEL, connect USB"
echo "  4. Release BOOTSEL button"
echo ""
echo "The Pico should appear as a USB drive (RPI-RP2)"
echo ""

read -p "Press ENTER when Pico is in BOOTSEL mode..."

# Check if RPI-RP2 drive is mounted
if ls /media/*/RPI-RP2 2>/dev/null || ls /run/media/*/RPI-RP2 2>/dev/null; then
    MOUNT_POINT=$(ls -d /media/*/RPI-RP2 /run/media/*/RPI-RP2 2>/dev/null | head -1)
    echo -e "${GREEN}✓${NC} Pico found at: ${MOUNT_POINT}"
else
    echo -e "${RED}✗${NC} Pico not found in BOOTSEL mode!"
    echo ""
    echo "Troubleshooting:"
    echo "  - Make sure you're holding BOOTSEL while connecting USB"
    echo "  - Try a different USB cable"
    echo "  - Try a different USB port"
    exit 1
fi
echo ""

# 5. Flash firmware
echo -e "${YELLOW}[4/5] Flashing firmware...${NC}"

UF2_FILE="build/my_firmware.uf2"

if [ ! -f "$UF2_FILE" ]; then
    echo -e "${RED}✗${NC} Firmware file not found: ${UF2_FILE}"
    exit 1
fi

echo "Copying ${UF2_FILE} to ${MOUNT_POINT}..."
cp "$UF2_FILE" "$MOUNT_POINT/"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Firmware flashed successfully!"
    echo ""
    echo "The Pico will automatically reboot..."
    sleep 2
else
    echo -e "${RED}✗${NC} Flash failed!"
    exit 1
fi
echo ""

# 6. Wait for Pico to reboot
echo -e "${YELLOW}[5/5] Waiting for Pico to reboot...${NC}"
echo "Waiting 5 seconds..."
sleep 5

# Check if Pico is back
if ls /dev/ttyACM* 2>/dev/null; then
    DEVICE=$(ls /dev/ttyACM* | head -1)
    echo -e "${GREEN}✓${NC} Pico rebooted and found at: ${DEVICE}"
else
    echo -e "${YELLOW}⚠${NC} Pico not found yet, may need more time"
    echo "Check manually: ls /dev/ttyACM*"
fi
echo ""

# 7. Summary
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Flash Complete!${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "Next steps:"
echo "  1. Start micro-ROS agent:"
echo "     ros2 run micro_ros_agent micro_ros_agent serial --dev ${DEVICE:-/dev/ttyACM0}"
echo ""
echo "  2. Monitor firmware output:"
echo "     screen ${DEVICE:-/dev/ttyACM0} 115200"
echo "     (Press Ctrl+A then K to exit)"
echo ""
echo "  3. Run diagnostics:"
echo "     bash diagnose_firmware.sh"
echo ""
echo "  4. Test motor:"
echo "     bash test_front_right_motor.sh"
echo ""

cd ..
