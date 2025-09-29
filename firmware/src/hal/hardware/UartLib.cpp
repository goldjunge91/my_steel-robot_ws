/**
 * @file UartLib.cpp
 * @author Maciej Kurcius, refactored for Pico SDK by Gemini
 * @brief Native Pico SDK implementation for UART protocol communication.
 * @version 0.2
 * @date 2025-09-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "hardware/UartLib.h"
#include <cstring>
#include <string>
#include <algorithm> // For std::min
#include <math.h>

// These are defined in bsp.cpp, ensure they are std::string
extern std::string PowerBoardFirmwareVersion;
extern std::string PowerBoardVersion;

UartProtocolClass::UartProtocolClass(uart_inst_t* uart_instance, uint32_t baudrate, uint tx_pin, uint rx_pin)
    : _uart(uart_instance) {
    uart_init(_uart, baudrate);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
}

UartProtocolClass::~UartProtocolClass() { 
    if(_uart) uart_deinit(_uart); 
}

void UartProtocolClass::UartProtocolLoopHandler() {
    this->CleanRxBuffer();
    rx_buffer_size = 0;
    while (uart_is_readable(_uart) && rx_buffer_size < RX_BUFF_CAPACITY) {
        rx_buffer[rx_buffer_size++] = uart_getc(_uart);
    }

    if (this->rx_buffer_size != 0) {
        this->StreamParse();
    }
}

int8_t UartProtocolClass::StreamParse() {
    uint16_t frame_data_ptr;
    uint8_t counted_check_sum;
    if (rx_buffer_size >= RX_BUFF_CAPACITY)
        return -1;

    for (uint16_t i = 0; i < rx_buffer_size && i < RX_BUFF_CAPACITY; i++) {
        if (rx_buffer[i] == '<') {
            counted_check_sum = 0;
            if (HexToByte(&rx_buffer[i + 1], &this->processed_frame.cmd) != ConversionError) {
                counted_check_sum ^= this->processed_frame.cmd;
                if (HexToByte(&rx_buffer[i + 3], &this->processed_frame.arg_size) !=
                    ConversionError) {
                    if (this->processed_frame.arg_size <= MAX_ARGS_SIZE) {
                        counted_check_sum ^= this->processed_frame.arg_size;
                        frame_data_ptr = i + 5;
                        if (HexToByte(&rx_buffer[i + this->processed_frame.arg_size * 2 + 5],
                                      &processed_frame.check_sum) !=
                            ConversionError) { 
                            for (uint8_t j = 0; j < this->processed_frame.arg_size; j++) {
                                if (HexToByte(&rx_buffer[frame_data_ptr + j * 2],
                                              &this->processed_frame.args[j]) == ConversionError)
                                    break;
                                counted_check_sum ^= this->processed_frame.args[j];
                            }
                            if (counted_check_sum == this->processed_frame.check_sum) {
                                this->ExecuteFrame();
                                i = i + this->processed_frame.arg_size * 2 +
                                    7; 
                            }
                        }
                    }
                }
            }
        }
    }
    return 0;
}

UartConvStatusTypeDef UartProtocolClass::HexToByte(uint8_t *arg_byte, uint8_t *arg_result) {
    uint8_t value;
    value = DecodeHex(arg_byte[0]);
    if (value < 16) {
        *arg_result = value << 4;
        value = DecodeHex(arg_byte[1]);
        if (value < 16) {
            *arg_result = *arg_result + value;
            return ConversionOk;
        }
        return ConversionError;
    }
    return ConversionError;
}

uint8_t *UartProtocolClass::ByteToHex(uint8_t arg_byte, uint8_t *arg_buffer) {
    arg_buffer[0] = EncodeHex((arg_byte & 0xF0) >> 4);
    arg_buffer[1] = EncodeHex(arg_byte & 0x0F);
    return arg_buffer + 2;
}

uint8_t UartProtocolClass::DecodeHex(uint8_t arg_byte) {
    if (arg_byte >= '0' && arg_byte <= '9')
        return arg_byte - '0';
    else if (arg_byte >= 'a' && arg_byte <= 'f')
        return arg_byte - 'a' + 10;
    else if (arg_byte >= 'A' && arg_byte <= 'F')
        return arg_byte - 'A' + 10;
    else
        return 0xFF;
}

uint8_t UartProtocolClass::EncodeHex(uint8_t arg_byte) {
    if (arg_byte < 10)
        return '0' + arg_byte;
    else if (arg_byte < 16)
        return 'a' + arg_byte - 10;
    else
        return 0xFF;
}

void UartProtocolClass::ExecuteFrame() {
    switch (this->processed_frame.cmd) {
    case 0:
        break;
    case 1: { // Power Board firmware version
        if (processed_frame.arg_size != POWER_BOARD_VERSION_MSG_LENGTH)
            break;
        PowerBoardFirmwareVersion.clear();
        PowerBoardVersion.clear();
        for (uint8_t i = 1; i < 12; i++) {
            PowerBoardFirmwareVersion += (char)processed_frame.args[i];
        }
        for (uint8_t i = 13; i < 25; i++) {
            PowerBoardVersion += (char)processed_frame.args[i];
        }
    } break;
    case 2: { // Battery State
        if (this->processed_frame.arg_size != BATTERY_STATE_MSG_LENGTH) {
            break;
        }
        battery_state_queue_t battery_state;
        battery_state.voltage =
            (float(this->processed_frame.args[1] << 8 | this->processed_frame.args[2])) * 0.001;
        int32_t temperature = (this->processed_frame.args[3] << 8 | this->processed_frame.args[4]);
        if (temperature > 200 || temperature < -100) {
            battery_state.temperature = NAN;
        } else
            battery_state.temperature = (float)temperature;
        battery_state.current =
            (float((this->processed_frame.args[7] << 8 | this->processed_frame.args[8]) -
                   (this->processed_frame.args[5] << 8 | this->processed_frame.args[6]))) *
            0.001;
        battery_state.charge_current = NAN;
        battery_state.capacity = NAN;
        battery_state.design_capacity =
            (float(this->processed_frame.args[11] << 8 | this->processed_frame.args[12])) * 0.001;
        battery_state.percentage = NAN;
        battery_state.status = (BatteryStatusTypeDef)this->processed_frame.args[14];
        battery_state.health = (BatteryHealthTypeDef)this->processed_frame.args[15];
        battery_state.technology = (BatteryTechnologyTypeDef)this->processed_frame.args[16];
        battery_state.present = (bool)this->processed_frame.args[17];
        battery_state.cell_temperature[0] = NAN;
        battery_state.cell_voltage[0] = NAN;
        BatteryStateBuffer = battery_state;
        BatteryStateAvailable = true;
    } break;
    default:
        break;
    }
}

void UartProtocolClass::SendFrame(UartProtocolFrame arg_frame) {
    static uint8_t tx_buff[UART_FRAME_LENGTH(MAX_ARGS_SIZE)];
    uint8_t *tx_buff_ptr = tx_buff;
    arg_frame.check_sum = arg_frame.cmd ^ arg_frame.arg_size;
    *tx_buff_ptr++ = FRAME_START_BIT;
    tx_buff_ptr = ByteToHex(arg_frame.cmd, tx_buff_ptr);
    tx_buff_ptr = ByteToHex(arg_frame.arg_size, tx_buff_ptr);
    for (uint8_t i = 0; i < arg_frame.arg_size; i++) {
        tx_buff_ptr = ByteToHex(arg_frame.args[i], tx_buff_ptr);
        arg_frame.check_sum ^= arg_frame.args[i];
    }
    tx_buff_ptr = ByteToHex(arg_frame.check_sum, tx_buff_ptr);
    *tx_buff_ptr++ = FRAME_STOP_BIT;
    this->SendBuffer(UART_FRAME_LENGTH(arg_frame.arg_size), tx_buff);
}

void UartProtocolClass::SendBuffer(uint8_t arg_size, uint8_t *arg_buffer) {
    uart_write_blocking(_uart, arg_buffer, arg_size);
}

void UartProtocolClass::SendBuffer(uint8_t arg_size, const std::string& arg_buffer) {
    uart_write_blocking(_uart, (const uint8_t*)arg_buffer.c_str(), std::min((size_t)arg_size, arg_buffer.length()));
}

void UartProtocolClass::CleanRxBuffer(void) { memset(this->rx_buffer, 0, RX_BUFF_CAPACITY); }