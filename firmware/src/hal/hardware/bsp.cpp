/**
 * @file bsp.cpp
 * @author Maciej Kurcius, refactored for Pico SDK by Gemini
 * @brief Native Pico SDK implementation of the board support package.
 * @version 0.2
 * @date 2025-09-25
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "hardware/bsp.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include <stdio.h>
#include <cstring>

// Initialize the UART communication for the power board using native Pico SDK types
UartProtocolClass PowerBoardSerial(uart1, PWR_BRD_SERIAL_BAUDRATE, PWR_BRD_SERIAL_TX, PWR_BRD_SERIAL_RX);
std::string PowerBoardFirmwareVersion = "";
std::string PowerBoardVersion = "";

void BoardGpioInit(void) {
    // Green LED
    gpio_init(GRN_LED);
    gpio_set_dir(GRN_LED, GPIO_OUT);
    gpio_put(GRN_LED, 0);

    // 5V Enable
    gpio_init(EN_LOC_5V);
    gpio_set_dir(EN_LOC_5V, GPIO_OUT);
    gpio_put(EN_LOC_5V, 0);

    // Red LED
    gpio_init(RD_LED);
    gpio_set_dir(RD_LED, GPIO_OUT);
    gpio_put(RD_LED, 0);

    // Power Board GPIO Output
    gpio_init(PWR_BRD_GPIO_OUTPUT);
    gpio_set_dir(PWR_BRD_GPIO_OUTPUT, GPIO_OUT);
    gpio_put(PWR_BRD_GPIO_OUTPUT, 0);

    // Power Board GPIO Input
    gpio_init(PWR_BRD_GPIO_INPUT);
    gpio_set_dir(PWR_BRD_GPIO_INPUT, GPIO_IN);
    gpio_pull_up(PWR_BRD_GPIO_INPUT);
}

void SetLocalPower(SwitchStateTypeDef State_) {
    if (State_ == Off)
        gpio_put(EN_LOC_5V, 0);
    else if (State_ == On)
        gpio_put(EN_LOC_5V, 1);
    else if (State_ == Toggle)
        gpio_put(EN_LOC_5V, !gpio_get(EN_LOC_5V));
}

void SetGreenLed(SwitchStateTypeDef State_) {
    if (State_ == Off)
        gpio_put(GRN_LED, 0);
    else if (State_ == On)
        gpio_put(GRN_LED, 1);
    else if (State_ == Toggle)
        gpio_put(GRN_LED, !gpio_get(GRN_LED));
}

void SetRedLed(SwitchStateTypeDef State_) {
    if (State_ == Off)
        gpio_put(RD_LED, 0);
    else if (State_ == On)
        gpio_put(RD_LED, 1);
    else if (State_ == Toggle)
        gpio_put(RD_LED, !gpio_get(RD_LED));
}

void BoardPheripheralsInit(void) {
    BoardGpioInit();
    // stdio (for USB serial) is initialized in main.cpp
    printf("Hello SBC\n");
    SetLocalPower(On);
    I2cBusInit();
    sleep_ms(250);
}

PowerOffSignalTypeDef PowerOffSignalLoopHandler(void) {
    if (gpio_get(PWR_BRD_GPIO_INPUT))
        return Shutdown;
    else
        return Idle;
}

std::string GetBoardVersion(void) {
    static std::string BoardVersion = "unknown";
    if (BoardVersion == "unknown") {
        char RetVal[BOARD_VER_MEM_SIZE + 1];
        RetVal[BOARD_VER_MEM_SIZE] = '\0';
        for (uint8_t i = 0; i < BOARD_VER_READ_ATTEMPTS; i++) {
            if (EepromReadPage(BOARD_VER_MEM_BLOCK, BOARD_VER_MEM_ADDR, (uint8_t *)RetVal,
                                       BOARD_VER_MEM_SIZE) == 0) {
                BoardVersion = std::string(RetVal);
                return BoardVersion;
            }
        }
        BoardVersion = "v1.1-read-failed"; // If all attempts fail
    }
    return BoardVersion;
}

void I2cBusInit(void) {
    // Initialize I2C0 controller at 100kHz
    i2c_init(i2c0, 100 * 1000);
    // Setup I2C pins
    gpio_set_function(IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA);
    gpio_pull_up(IMU_SCL);
}

void TestFunction(uint8_t state) {
    if (state == 1)
        SetGreenLed(On);
    if (state == 0)
        SetGreenLed(Off);
    UartProtocolFrame TestFrame;
    TestFrame.arg_size = 12;
    TestFrame.cmd = 25;
    TestFrame.args[0] = 1;
    TestFrame.args[1] = 1;
    TestFrame.args[2] = 8;
    TestFrame.args[11] = state;
    PowerBoardSerial.SendFrame(TestFrame);
}

void PbInfoRequest(void) {
    UartProtocolFrame PbInfoReqFrame;
    PbInfoReqFrame.arg_size = 1;
    PbInfoReqFrame.cmd = 1;
    PbInfoReqFrame.args[0] = 0;
    PowerBoardSerial.SendFrame(PbInfoReqFrame);
}

void BatteryInfoRequest(void) {
    UartProtocolFrame PbInfoReqFrame;
    PbInfoReqFrame.arg_size = 18;
    PbInfoReqFrame.cmd = 2;
    PbInfoReqFrame.args[0] = 0;
    PowerBoardSerial.SendFrame(PbInfoReqFrame);
}

// --- EEPROM Functions using Pico SDK I2C API ---

uint8_t EepromWriteByte(uint8_t BlockAddr, uint8_t ByteAddr, uint8_t Value) {
    uint8_t DataToSend[] = {ByteAddr, Value};
    int result = i2c_write_blocking(i2c0, EEPROM_CONTROL_BYTE(EEPROM_DEV_ID, BlockAddr), DataToSend, 2, false);
    return (result == 2) ? 0 : -1;
}

uint8_t EepromReadByte(uint8_t BlockAddr, uint8_t ByteAddr, uint8_t *Value) {
    int write_result = i2c_write_blocking(i2c0, EEPROM_CONTROL_BYTE(EEPROM_DEV_ID, BlockAddr), &ByteAddr, 1, true);
    if (write_result != 1) return -1;
    int read_result = i2c_read_blocking(i2c0, EEPROM_CONTROL_BYTE(EEPROM_DEV_ID, BlockAddr), Value, 1, false);
    return (read_result == 1) ? 0 : -1;
}

uint8_t EepromWritePage(uint8_t BlockAddr, uint8_t ByteAddr, uint8_t *Value, uint8_t Size) {
    uint8_t DataToSend[Size + 1];
    DataToSend[0] = ByteAddr;
    memcpy(DataToSend + 1, Value, Size);
    int result = i2c_write_blocking(i2c0, EEPROM_CONTROL_BYTE(EEPROM_DEV_ID, BlockAddr), DataToSend, Size + 1, false);
    return (result == (Size + 1)) ? 0 : -1;
}

uint8_t EepromReadPage(uint8_t BlockAddr, uint8_t ByteAddr, uint8_t *Value, uint8_t Size) {
    int write_result = i2c_write_blocking(i2c0, EEPROM_CONTROL_BYTE(EEPROM_DEV_ID, BlockAddr), &ByteAddr, 1, true);
    if (write_result != 1) return -1;
    int read_result = i2c_read_blocking(i2c0, EEPROM_CONTROL_BYTE(EEPROM_DEV_ID, BlockAddr), Value, Size, false);
    return (read_result == Size) ? 0 : -1;
}