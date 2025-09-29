/**
 * @file bsp.h
 * @author Maciej Kurcius
 * @brief Board support package
 * @version 0.1
 * @date 2022-01-20
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef BSP_H
#define BSP_H

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <string>
// #include <motors.h>
#include "UartLib.h"
#include "hardware_cfg.h"

typedef enum { Off = 0, On = 1, Toggle = 2 } SwitchStateTypeDef;

typedef enum { Idle = 0, Shutdown = 1 } PowerOffSignalTypeDef;

void BoardGpioInit(void);
void SetLocalPower(SwitchStateTypeDef State_);
void SetGreenLed(SwitchStateTypeDef State_);
void SetRedLed(SwitchStateTypeDef State_);
void BoardPheripheralsInit(void);
PowerOffSignalTypeDef PowerOffSignalLoopHandler(void);
std::string GetBoardVersion(void);
void I2cBusInit(void);

// POWER BOARD FUNCTIONS

void TestFunction(uint8_t);
void RoboticArmInvReset(void);
void PbInfoRequest(void);
void BatteryInfoRequest(void);

// EEPROM FUNCTIONS

uint8_t EepromWriteByte(uint8_t BlockAddr, uint8_t ByteAddr, uint8_t value);
uint8_t EepromReadByte(uint8_t BlockAddr, uint8_t ByteAddr, uint8_t *value);
uint8_t EepromWritePage(uint8_t BlockAddr, uint8_t ByteAddr, uint8_t *Value, uint8_t Size);
uint8_t EepromReadPage(uint8_t BlockAddr, uint8_t ByteAddr, uint8_t *Value, uint8_t Size);

// Expose the global PowerBoardSerial object for use in main.cpp
extern UartProtocolClass PowerBoardSerial;

#endif /* BSP_H */