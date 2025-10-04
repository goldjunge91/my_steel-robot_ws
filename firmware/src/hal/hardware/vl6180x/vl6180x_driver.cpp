#include "hal/hardware/vl6180x/vl6180x_driver.hpp"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include <cstdio>

// Register Adressen, wie vom Hersteller empfohlen
namespace {
constexpr uint8_t VL6180X_DEFAULT_I2C_ADDR = 0x29;
enum regAddr : uint16_t {
    IDENTIFICATION__MODEL_ID = 0x0000,
    SYSTEM__INTERRUPT_CLEAR = 0x0015,
    SYSTEM__FRESH_OUT_OF_RESET = 0x0016,
    SYSRANGE__START = 0x0018,
    SYSALS__START = 0x0038,
    SYSALS__ANALOGUE_GAIN = 0x003F,
    SYSALS__INTEGRATION_PERIOD_HI = 0x0040,
    SYSALS__INTEGRATION_PERIOD_LO = 0x0041,
    RESULT__RANGE_STATUS = 0x004d,
    RESULT__ALS_STATUS = 0x004e,
    RESULT__INTERRUPT_STATUS_GPIO = 0x004f,
    RESULT__ALS_VAL = 0x0050,
    RESULT__RANGE_VAL = 0x0062,
};
}  // namespace

namespace hal::hardware {

Vl6180x::Vl6180x(const Config& config) : config_(config) {}

bool Vl6180x::initialize() {
    uint8_t chip_id = 0;
    if (!readRegister(IDENTIFICATION__MODEL_ID, chip_id) || chip_id != 0xB4) {
        printf("[Vl6180x] Chip ID mismatch. Found 0x%02X, expected 0xB4.\n", chip_id);
        return false;
    }

    uint8_t reset_status = 0;
    if (!readRegister(SYSTEM__FRESH_OUT_OF_RESET, reset_status)) return false;

    if (reset_status & 0x01) {
        printf("[Vl6180x] Sensor is fresh out of reset. Starting mandatory init sequence.\n");
        // Lade die "tunneled" Einstellungen
        writeRegister(0x0207, 0x01);
        writeRegister(0x0208, 0x01);
        writeRegister(0x0096, 0x00);
        writeRegister(0x0097, 0xfd);
        writeRegister(0x00e3, 0x00);
        writeRegister(0x00e4, 0x04);
        writeRegister(0x00e5, 0x02);
        writeRegister(0x00e6, 0x01);
        writeRegister(0x00e7, 0x03);
        writeRegister(0x00f5, 0x02);
        writeRegister(0x00d9, 0x05);
        writeRegister(0x00db, 0xce);
        writeRegister(0x00dc, 0x03);
        writeRegister(0x00dd, 0xf8);
        writeRegister(0x009f, 0x00);
        writeRegister(0x00a3, 0x3c);
        writeRegister(0x00b7, 0x00);
        writeRegister(0x00bb, 0x3c);
        writeRegister(0x00b2, 0x09);
        writeRegister(0x00ca, 0x09);
        writeRegister(0x0198, 0x01);
        writeRegister(0x01b0, 0x17);
        writeRegister(0x01ad, 0x00);
        writeRegister(0x00ff, 0x05);
        writeRegister(0x0100, 0x05);
        writeRegister(0x0199, 0x05);
        writeRegister(0x01a6, 0x1b);
        writeRegister(0x01ac, 0x3e);
        writeRegister(0x01a7, 0x1f);
        writeRegister(0x0030, 0x00);
        writeRegister(SYSTEM__FRESH_OUT_OF_RESET, 0x00);
    }

    // Standard-Konfigurationen anwenden
    writeRegister(0x010A, 0x30);
    writeRegister(SYSALS__ANALOGUE_GAIN, 0x46);
    writeRegister(0x002d, 0x01);
    writeRegister(SYSALS__INTEGRATION_PERIOD_HI, 0x00);
    writeRegister(SYSALS__INTEGRATION_PERIOD_LO, 0x64);

    return true;
}

uint8_t Vl6180x::readRange() {
    uint8_t status = 0;
    do {  // Warte, bis eine Messung bereit ist
        if (!readRegister(RESULT__RANGE_STATUS, status)) return 255;  // Fehler
    } while (!(status & 0x01));

    writeRegister(SYSRANGE__START, 0x01);  // Starte eine neue Messung

    do {  // Warte, bis die Messung abgeschlossen ist
        if (!readRegister(RESULT__INTERRUPT_STATUS_GPIO, status)) return 255;
    } while (!(status & 0x04));

    uint8_t range = 0;
    readRegister(RESULT__RANGE_VAL, range);
    writeRegister(SYSTEM__INTERRUPT_CLEAR, 0x01);
    return range;
}

float Vl6180x::readLux() {
    uint8_t const gain = 0x40;  // GAIN_20 für hohe Empfindlichkeit
    writeRegister(SYSALS__ANALOGUE_GAIN, gain);

    writeRegister(SYSALS__START, 0x01);  // Starte eine neue Messung

    uint8_t status = 0;
    do {  // Warte, bis die Messung abgeschlossen ist
        if (!readRegister(RESULT__INTERRUPT_STATUS_GPIO, status)) return 0.0f;
    } while (!(status & 0x20));

    uint16_t als = 0;
    readRegister16(RESULT__ALS_VAL, als);
    writeRegister(SYSTEM__INTERRUPT_CLEAR, 0x02);

    float lux = 0.32f * (static_cast<float>(als) / 20.0f);
    return lux;
}

// --- Low-Level I2C Funktionen ---

bool Vl6180x::writeRegister(uint16_t reg, uint8_t value) {
    uint8_t buffer[3];
    buffer[0] = (reg >> 8) & 0xFF;
    buffer[1] = reg & 0xFF;
    buffer[2] = value;
    int result = i2c_write_blocking(config_.bus, VL6180X_DEFAULT_I2C_ADDR, buffer, 3, false);
    return result == 3;
}

bool Vl6180x::readRegister(uint16_t reg, uint8_t& value) {
    uint8_t reg_buffer[2];
    reg_buffer[0] = (reg >> 8) & 0xFF;
    reg_buffer[1] = reg & 0xFF;
    if (i2c_write_blocking(config_.bus, VL6180X_DEFAULT_I2C_ADDR, reg_buffer, 2, true) != 2) {
        return false;
    }
    if (i2c_read_blocking(config_.bus, VL6180X_DEFAULT_I2C_ADDR, &value, 1, false) != 1) {
        return false;
    }
    return true;
}

bool Vl6180x::readRegister16(uint16_t reg, uint16_t& value) {
    uint8_t reg_buffer[2];
    reg_buffer[0] = (reg >> 8) & 0xFF;
    reg_buffer[1] = reg & 0xFF;
    if (i2c_write_blocking(config_.bus, VL6180X_DEFAULT_I2C_ADDR, reg_buffer, 2, true) != 2) {
        return false;
    }
    uint8_t data[2];
    if (i2c_read_blocking(config_.bus, VL6180X_DEFAULT_I2C_ADDR, data, 2, false) != 2) {
        return false;
    }
    value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    return true;
}

}  // namespace hal::hardware
