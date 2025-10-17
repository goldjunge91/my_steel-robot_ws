#!/bin/bash
# Master Test Runner
# Runs all firmware and system tests in sequence

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Robot Firmware Test Suite            ║${NC}"
echo -e "${BLUE}║  Complete System Verification         ║${NC}"
echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo ""

TOTAL_PASSED=0
TOTAL_FAILED=0

# Make all scripts executable
chmod +x test_firmware_build.sh
chmod +x test_firmware_complete.sh
chmod +x diagnose_firmware.sh
chmod +x test_front_right_motor.sh

# Test 1: Build Verification
echo -e "${BLUE}═══════════════════════════════════════${NC}"
echo -e "${BLUE} Phase 1: Build Verification${NC}"
echo -e "${BLUE}═══════════════════════════════════════${NC}"
echo ""

if bash test_firmware_build.sh; then
    echo -e "${GREEN}✓ Phase 1 PASSED${NC}"
    TOTAL_PASSED=$((TOTAL_PASSED + 1))
else
    echo -e "${RED}✗ Phase 1 FAILED${NC}"
    TOTAL_FAILED=$((TOTAL_FAILED + 1))
    echo ""
    echo "Build verification failed. Cannot continue."
    echo "Please rebuild firmware: bash rebuild_and_flash.sh"
    exit 1
fi

echo ""
read -p "Press ENTER to continue to firmware tests..."
echo ""

# Test 2: Firmware Diagnostics
echo -e "${BLUE}═══════════════════════════════════════${NC}"
echo -e "${BLUE} Phase 2: Firmware Diagnostics${NC}"
echo -e "${BLUE}═══════════════════════════════════════${NC}"
echo ""

if bash diagnose_firmware.sh; then
    echo -e "${GREEN}✓ Phase 2 PASSED${NC}"
    TOTAL_PASSED=$((TOTAL_PASSED + 1))
else
    echo -e "${YELLOW}⚠ Phase 2 had issues${NC}"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
read -p "Press ENTER to continue to comprehensive tests..."
echo ""

# Test 3: Comprehensive Firmware Test
echo -e "${BLUE}═══════════════════════════════════════${NC}"
echo -e "${BLUE} Phase 3: Comprehensive Firmware Test${NC}"
echo -e "${BLUE}═══════════════════════════════════════${NC}"
echo ""

if bash test_firmware_complete.sh; then
    echo -e "${GREEN}✓ Phase 3 PASSED${NC}"
    TOTAL_PASSED=$((TOTAL_PASSED + 1))
else
    echo -e "${RED}✗ Phase 3 FAILED${NC}"
    TOTAL_FAILED=$((TOTAL_FAILED + 1))
    echo ""
    echo "Firmware tests failed."
    read -p "Continue to motor test anyway? (y/n) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
read -p "Press ENTER to continue to motor test..."
echo ""

# Test 4: Motor Test
echo -e "${BLUE}═══════════════════════════════════════${NC}"
echo -e "${BLUE} Phase 4: Front Right Motor Test${NC}"
echo -e "${BLUE}═══════════════════════════════════════${NC}"
echo ""

if bash test_front_right_motor.sh; then
    echo -e "${GREEN}✓ Phase 4 PASSED${NC}"
    TOTAL_PASSED=$((TOTAL_PASSED + 1))
else
    echo -e "${RED}✗ Phase 4 FAILED${NC}"
    TOTAL_FAILED=$((TOTAL_FAILED + 1))
fi

# Final Summary
echo ""
echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Final Test Summary                    ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
echo ""
echo "Test Phases:"
echo "  1. Build Verification"
echo "  2. Firmware Diagnostics"
echo "  3. Comprehensive Firmware Test"
echo "  4. Motor Test"
echo ""
echo "Results:"
echo -e "  ${GREEN}Passed:${NC} ${TOTAL_PASSED}/4"
echo -e "  ${RED}Failed:${NC} ${TOTAL_FAILED}/4"
echo ""

if [ ${TOTAL_FAILED} -eq 0 ]; then
    echo -e "${GREEN}╔════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║  ALL TESTS PASSED! ✓✓✓                 ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════╝${NC}"
    echo ""
    echo "🎉 Your robot firmware is working perfectly!"
    echo ""
    echo "Next steps:"
    echo "  - Connect remaining motors (FL, RL, RR)"
    echo "  - Update firmware to add all 4 motors"
    echo "  - Test full mecanum drive"
    echo "  - Calibrate odometry"
    EXIT_CODE=0
else
    echo -e "${RED}╔════════════════════════════════════════╗${NC}"
    echo -e "${RED}║  SOME TESTS FAILED ✗                   ║${NC}"
    echo -e "${RED}╚════════════════════════════════════════╝${NC}"
    echo ""
    echo "Troubleshooting guide:"
    echo ""
    echo "If Build Verification failed:"
    echo "  - Check firmware/src/main.cpp"
    echo "  - Rebuild: cd firmware && make build"
    echo ""
    echo "If Firmware Diagnostics failed:"
    echo "  - Reflash firmware: bash rebuild_and_flash.sh"
    echo "  - Check micro-ROS agent is running"
    echo "  - Check USB connection"
    echo ""
    echo "If Comprehensive Test failed:"
    echo "  - Check firmware serial output: screen /dev/ttyACM0 115200"
    echo "  - Verify MotorsAgent is starting"
    echo "  - Check /joint_states topic"
    echo ""
    echo "If Motor Test failed:"
    echo "  - Check motor connections (Pins 4,5,8,9)"
    echo "  - Check motor power supply"
    echo "  - Verify encoder is connected"
    EXIT_CODE=1
fi

echo ""
exit ${EXIT_CODE}
