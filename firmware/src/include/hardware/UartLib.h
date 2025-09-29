#ifndef UARTLIB_H
#define UARTLIB_H

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware_cfg.h"
#include "pico/stdlib.h"
#include <string>

typedef enum {
    ConversionError = 0,
    ConversionOk = 1,
} UartConvStatusTypeDef;

#define RX_BUFF_CAPACITY 255
#define MAX_ARGS_SIZE 32
#define DEFAULT_TIMEOUT 10
#define FRAME_START_BIT '<'
#define FRAME_STOP_BIT '>'
#define UART_FRAME_LENGTH(arg) (arg * 2 + 8)

// POWER BOARD MSG
#define POWER_BOARD_VERSION_MSG_LENGTH 25
#define BATTERY_STATE_MSG_LENGTH 18

extern battery_state_queue_t BatteryStateBuffer;
extern bool BatteryStateAvailable;

typedef struct {
    uint8_t cmd;
    uint8_t arg_size;
    uint8_t args[MAX_ARGS_SIZE];
    uint8_t check_sum;
} UartProtocolFrame;

class UartProtocolClass {
  public:
    UartProtocolClass(uart_inst_t* uart_instance, uint32_t baudrate, uint tx_pin, uint rx_pin);
    ~UartProtocolClass();
    void UartProtocolLoopHandler();
    void SendFrame(UartProtocolFrame arg_frame);

  private:
    uart_inst_t* _uart;
    int8_t StreamParse();
    void ExecuteFrame();
    void SendBuffer(uint8_t arg_size, uint8_t *arg_buffer);
    void SendBuffer(uint8_t arg_size, const std::string& arg_buffer);
    void CleanRxBuffer(void);
    UartConvStatusTypeDef HexToByte(uint8_t *arg_byte, uint8_t *arg_result);
    uint8_t *ByteToHex(uint8_t arg_byte, uint8_t *arg_buffer);
    uint8_t DecodeHex(uint8_t arg_byte);
    uint8_t EncodeHex(uint8_t arg_byte);
    uint8_t rx_buffer[RX_BUFF_CAPACITY];
    uint16_t rx_buffer_size;
    UartProtocolFrame processed_frame;
};

#endif /* UARTLIB_H */