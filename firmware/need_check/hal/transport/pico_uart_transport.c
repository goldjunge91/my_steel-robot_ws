#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>

void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

bool pico_usb_transport_open(struct uxrCustomTransport *transport)
{
    // Ensure that stdio_init_all is only called once on the runtime
    static bool require_init = true;
    if (require_init)
    {
        stdio_init_all();
        require_init = false;
    }

    return true;
}

bool pico_usb_transport_close(struct uxrCustomTransport *transport)
{
    return true;
}

size_t pico_usb_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
    for (size_t i = 0; i < len; i++)
    {
        if (buf[i] != putchar((int)buf[i]))
        {
            *errcode = 1;
            return i;
        }
    }
    return len;
}

size_t pico_usb_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    uint64_t start_time_us = time_us_64();
    for (size_t i = 0; i < len; i++)
    {
        int64_t elapsed_time_us = timeout * 1000 - (time_us_64() - start_time_us);
        if (elapsed_time_us < 0)
        {
            *errcode = 1;
            return i;
        }

        int character = getchar_timeout_us(elapsed_time_us);
        if (character == PICO_ERROR_TIMEOUT)
        {
            *errcode = 1;
            return i;
        }
        buf[i] = character;
    }
    return len;
}

// ---------------------- UART transport implementation ----------------------
// Uses UART0 on GP0 (TX) / GP1 (RX) by default. Adjust pins/baud if needed.
#ifndef PICO_UART_ID
#define PICO_UART_ID uart0
#endif
#ifndef PICO_UART_BAUD
#define PICO_UART_BAUD 115200
#endif
#ifndef PICO_UART_TX_PIN
#define PICO_UART_TX_PIN 0
#endif
#ifndef PICO_UART_RX_PIN
#define PICO_UART_RX_PIN 1
#endif

bool pico_uart_transport_open(struct uxrCustomTransport *transport)
{
    (void)transport;
    static bool uart_initialized = false;
    if (!uart_initialized)
    {
        // Initialize chosen UART
        uart_init(PICO_UART_ID, PICO_UART_BAUD);
        gpio_set_function(PICO_UART_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(PICO_UART_RX_PIN, GPIO_FUNC_UART);
        // Optionally enable FIFO or other settings here
        uart_initialized = true;
    }
    return true;
}

bool pico_uart_transport_close(struct uxrCustomTransport *transport)
{
    (void)transport;
    // Nothing special to do; keep UART initialized for runtime
    return true;
}

size_t pico_uart_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
    (void)transport;
    for (size_t i = 0; i < len; ++i)
    {
        // blocking put
        uart_putc_raw(PICO_UART_ID, buf[i]);
    }
    return len;
}

size_t pico_uart_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    (void)transport;
    uint64_t start_time_us = time_us_64();
    for (size_t i = 0; i < len; ++i)
    {
        int64_t remaining_us = (int64_t)timeout * 1000 - (int64_t)(time_us_64() - start_time_us);
        if (remaining_us <= 0)
        {
            *errcode = 1;
            return i;
        }

        // wait for data until timeout expires
        while (!uart_is_readable(PICO_UART_ID))
        {
            if ((int64_t)(time_us_64() - start_time_us) >= (int64_t)timeout * 1000)
            {
                *errcode = 1;
                return i;
            }
            sleep_us(100);
        }

        int c = uart_getc(PICO_UART_ID);
        if (c == PICO_ERROR_TIMEOUT)
        {
            *errcode = 1;
            return i;
        }
        buf[i] = (uint8_t)c;
    }
    return len;
}
