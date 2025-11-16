/*
MIT License

UART Adapter for SRXL2 using FakeUART

This adapter implements the UART interface expected by spm_srxl_config.h
using the fake UART library.
*/

#ifndef __UART_ADAPTER_H__
#define __UART_ADAPTER_H__

#include <stdint.h>
#include "../fakeuart/fakeuart.h"

// UART handle type (we only support one UART for simplicity)
static inline void uartSetBaud(uint8_t uart, uint32_t baudRate)
{
    // Fake UART doesn't need baud rate changes
    (void)uart;
    (void)baudRate;
}

static inline void uartTransmit(uint8_t uart, uint8_t* pBuffer, uint8_t length)
{
    (void)uart;
    fakeuart_send(pBuffer, length);
}

#endif // __UART_ADAPTER_H__
