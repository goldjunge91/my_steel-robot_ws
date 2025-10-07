#!/bin/bash
# Firmware Build Verification Test
# Tests that firmware is built correctly with expected components

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PASSED=0
FAILED=0

pass() {
    echo -e "${GREEN}✓ PASS${NC}: $1"
    PASSED=$((PASSED + 1))
}

fail() {
    echo -e "${RED}✗ FAIL${NC}: $1"
    FAILED=$((FAILED + 1))
}

section() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
}

section "Firmware Build Verification Tests"

# TEST 1: Source files exist
section "TEST 1: Source File Verification"

REQUIRED_FILES=(
    "firmware/src/main.cpp"
    "firmware/src/MotorsAgent.cpp"
    "firmware/src/MotorsAgent.h"
    "firmware/src/DDD.cpp"
    "firmware/src/DDD.h"
    "firmware/src/uRosBridge.cpp"
    "firmware/src/application/ImuAgent.cpp"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$file" ]; then
        pass "Source file exists: $file"
    else
        fail "Source file MISSING: $file"
    fi
done

# TEST 2: main.cpp content
section "TEST 2: main.cpp Configuration"

if [ -f "firmware/src/main.cpp" ]; then
    # Test 2.1: Motor pins defined
    if grep -q "FRONT_RIGHT_PWR_CW" firmware/src/main.cpp; then
        pass "Front Right motor pins defined"
    else
        fail "Front Right motor pins NOT defined"
    fi
    
    # Test 2.2: MotorsAgent instantiated
    if grep -q "MotorsAgent motors" firmware/src/main.cpp; then
        pass "MotorsAgent instantiated"
    else
        fail "MotorsAgent NOT instantiated"
    fi
    
    # Test 2.3: Motor added
    if grep -q "motors.addMotor.*FRONT_RIGHT" firmware/src/main.cpp; then
        pass "Front Right motor added to MotorsAgent"
    else
        fail "Front Right motor NOT added"
    fi
    
    # Test 2.4: MotorsAgent started
    if grep -q 'motors.start.*"Motors"' firmware/src/main.cpp; then
        pass "MotorsAgent start() called"
    else
        fail "MotorsAgent start() NOT called"
    fi
    
    # Test 2.5: DDD configured with MotorsAgent
    if grep -q "ddd.setMotorsAgent" firmware/src/main.cpp; then
        pass "DDD configured with MotorsAgent"
    else
        fail "DDD NOT configured with MotorsAgent"
    fi
    
    # Test 2.6: Check startup order
    if grep -B5 "Starting DDD Agent" firmware/src/main.cpp | grep -q "Starting MotorsAgent"; then
        pass "MotorsAgent starts before DDD (correct order)"
    else
        fail "MotorsAgent startup order incorrect"
    fi
fi

# TEST 3: Build artifacts
section "TEST 3: Build Artifacts"

if [ -d "firmware/build" ]; then
    pass "Build directory exists"
    
    # Test 3.1: UF2 file
    if [ -f "firmware/build/my_firmware.uf2" ]; then
        SIZE=$(stat -f%z "firmware/build/my_firmware.uf2" 2>/dev/null || stat -c%s "firmware/build/my_firmware.uf2" 2>/dev/null)
        if [ -n "$SIZE" ] && [ "$SIZE" -gt 100000 ]; then
            pass "Firmware UF2 exists (${SIZE} bytes)"
        else
            fail "Firmware UF2 too small or invalid"
        fi
    else
        fail "Firmware UF2 NOT found - need to build"
    fi
    
    # Test 3.2: ELF file
    if [ -f "firmware/build/my_firmware.elf" ]; then
        pass "Firmware ELF exists"
        
        # Test 3.2.1: Check for MotorsAgent symbols
        if strings firmware/build/my_firmware.elf | grep -q "Starting MotorsAgent"; then
            pass "MotorsAgent code in binary"
        else
            fail "MotorsAgent code NOT in binary!"
        fi
        
        # Test 3.2.2: Check for joint_states
        if strings firmware/build/my_firmware.elf | grep -q "joint_states"; then
            pass "joint_states topic in binary"
        else
            fail "joint_states topic NOT in binary"
        fi
        
        # Test 3.2.3: Check for cmd_vel
        if strings firmware/build/my_firmware.elf | grep -q "cmd_vel"; then
            pass "cmd_vel topic in binary"
        else
            fail "cmd_vel topic NOT in binary"
        fi
        
        # Test 3.2.4: Check for Front Right motor
        if strings firmware/build/my_firmware.elf | grep -q "front_right_wheel_joint"; then
            pass "front_right_wheel_joint in binary"
        else
            fail "front_right_wheel_joint NOT in binary"
        fi
    else
        fail "Firmware ELF NOT found"
    fi
else
    fail "Build directory does not exist - firmware not built"
fi

# TEST 4: CMake configuration
section "TEST 4: Build Configuration"

if [ -f "firmware/CMakeLists.txt" ]; then
    pass "CMakeLists.txt exists"
    
    # Check if MotorsAgent is included
    if grep -q "MotorsAgent" firmware/CMakeLists.txt || \
       grep -q "MotorsAgent" firmware/src/CMakeLists.txt; then
        pass "MotorsAgent in build configuration"
    else
        fail "MotorsAgent NOT in build configuration"
    fi
fi

# SUMMARY
section "Build Test Summary"

echo "Results:"
echo -e "  ${GREEN}Passed:${NC} ${PASSED}"
echo -e "  ${RED}Failed:${NC} ${FAILED}"
echo ""

if [ ${FAILED} -eq 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  BUILD VERIFICATION PASSED! ✓${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo "Firmware is built correctly with all required components."
    echo ""
    echo "Next steps:"
    echo "  1. Flash firmware: bash rebuild_and_flash.sh"
    echo "  2. Test firmware: bash test_firmware_complete.sh"
    EXIT_CODE=0
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}  BUILD VERIFICATION FAILED! ✗${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo "The firmware build is missing required components."
    echo ""
    echo "Actions needed:"
    echo "  1. Check firmware/src/main.cpp configuration"
    echo "  2. Rebuild firmware: cd firmware && make build"
    echo "  3. Run this test again"
    EXIT_CODE=1
fi

echo ""
exit ${EXIT_CODE}
