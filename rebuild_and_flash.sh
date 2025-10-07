#!/bin/bash
# Complete Rebuild and Flash Script

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Complete Firmware Rebuild & Flash${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Change to firmware directory
cd firmware || { echo "firmware/ not found!"; exit 1; }

# 1. Complete clean
echo -e "${YELLOW}[1/6] Complete clean...${NC}"
rm -rf build
rm -rf CMakeFiles
rm -f CMakeCache.txt
echo -e "${GREEN}✓${NC} Cleaned"
echo ""

# 2. Show what will be built
echo -e "${YELLOW}[2/6] Checking source files...${NC}"
echo "Main file:"
ls -lh src/main.cpp
echo ""
echo "Key source files:"
ls -1 src/*.cpp | head -10
echo ""

# 3. Build
echo -e "${YELLOW}[3/6] Building firmware...${NC}"
echo "This will take 1-2 minutes..."
echo ""

if make build 2>&1 | tee /tmp/build.log; then
    echo ""
    echo -e "${GREEN}✓${NC} Build successful"
    
    # Show build output size
    if [ -f "build/my_firmware.uf2" ]; then
        SIZE=$(ls -lh build/my_firmware.uf2 | awk '{print $5}')
        echo "  Firmware size: ${SIZE}"
    fi
else
    echo ""
    echo -e "${RED}✗${NC} Build FAILED!"
    echo ""
    echo "Last 20 lines of build log:"
    tail -20 /tmp/build.log
    exit 1
fi
echo ""

# 4. Verify build contains expected code
echo -e "${YELLOW}[4/6] Verifying build...${NC}"
if [ -f "build/my_firmware.elf" ]; then
    # Check if MotorsAgent symbols are in the binary
    if strings build/my_firmware.elf | grep -q "Starting MotorsAgent"; then
        echo -e "${GREEN}✓${NC} MotorsAgent code found in binary"
    else
        echo -e "${RED}✗${NC} MotorsAgent code NOT found in binary!"
        echo "  This suggests a build problem"
    fi
    
    if strings build/my_firmware.elf | grep -q "joint_states"; then
        echo -e "${GREEN}✓${NC} joint_states topic found in binary"
    else
        echo -e "${YELLOW}⚠${NC} joint_states topic not found"
    fi
fi
echo ""

# 5. Prepare for flash
echo -e "${YELLOW}[5/6] Preparing to flash...${NC}"
echo ""
echo -e "${BLUE}INSTRUCTIONS:${NC}"
echo "  1. Unplug the Pico from USB"
echo "  2. Hold down the BOOTSEL button (white button on Pico)"
echo "  3. While holding BOOTSEL, plug in USB cable"
echo "  4. Release BOOTSEL button"
echo "  5. The Pico should appear as USB drive 'RPI-RP2'"
echo ""
read -p "Press ENTER when ready to flash..."
echo ""

# Wait for BOOTSEL mode
echo "Waiting for Pico in BOOTSEL mode..."
TIMEOUT=30
COUNT=0
MOUNT_POINT=""

while [ $COUNT -lt $TIMEOUT ]; do
    MOUNT_POINT=$(ls -d /media/*/RPI-RP2 /run/media/*/RPI-RP2 2>/dev/null | head -1)
    if [ -n "$MOUNT_POINT" ]; then
        break
    fi
    sleep 1
    COUNT=$((COUNT + 1))
    echo -n "."
done
echo ""

if [ -z "$MOUNT_POINT" ]; then
    echo -e "${RED}✗${NC} Pico not found in BOOTSEL mode after ${TIMEOUT}s"
    echo ""
    echo "Troubleshooting:"
    echo "  - Hold BOOTSEL button BEFORE plugging in USB"
    echo "  - Try different USB cable/port"
    echo "  - Check if RPI-RP2 drive appears: ls /media/*/RPI-RP2"
    exit 1
fi

echo -e "${GREEN}✓${NC} Pico found at: ${MOUNT_POINT}"
echo ""

# 6. Flash
echo -e "${YELLOW}[6/6] Flashing firmware...${NC}"

UF2_FILE="build/my_firmware.uf2"

if [ ! -f "$UF2_FILE" ]; then
    echo -e "${RED}✗${NC} ${UF2_FILE} not found!"
    exit 1
fi

echo "Copying ${UF2_FILE} to ${MOUNT_POINT}..."
cp -v "$UF2_FILE" "$MOUNT_POINT/"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Firmware flashed!"
    echo ""
    echo "Pico will reboot automatically..."
    sleep 3
else
    echo -e "${RED}✗${NC} Flash failed!"
    exit 1
fi
echo ""

# Wait for reboot
echo "Waiting for Pico to reboot..."
sleep 5

# Check if back
if ls /dev/ttyACM* 2>/dev/null; then
    DEVICE=$(ls /dev/ttyACM* | head -1)
    echo -e "${GREEN}✓${NC} Pico found at: ${DEVICE}"
else
    echo -e "${YELLOW}⚠${NC} Pico not detected yet"
    DEVICE="/dev/ttyACM0"
fi
echo ""

# Summary
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Flash Complete!${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "Next steps:"
echo ""
echo "1. Monitor firmware output:"
echo "   screen ${DEVICE} 115200"
echo "   (Press Ctrl+A then K to exit)"
echo ""
echo "   You should see:"
echo "   - Starting BlinkAgent..."
echo "   - Starting MotorsAgent..."
echo "   - Starting HCSR04Agent..."
echo "   - Starting Vl6180xAgent..."
echo "   - Starting ImuAgent..."
echo "   - Starting DDD Agent..."
echo ""
echo "2. Start micro-ROS agent (in another terminal):"
echo "   ros2 run micro_ros_agent micro_ros_agent serial --dev ${DEVICE}"
echo ""
echo "3. Check topics:"
echo "   ros2 topic list"
echo "   ros2 topic echo /joint_states"
echo ""
echo "4. Run diagnostics:"
echo "   bash diagnose_firmware.sh"
echo ""
echo "5. Test motor:"
echo "   bash test_front_right_motor.sh"
echo ""

cd ..
