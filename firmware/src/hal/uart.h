/*
 * UART0 (USB link to the tuner). Interrupt-driven ring buffers; the ISRs only
 * move single bytes so they never delay the crank decoder by more than ~2 us.
 */
#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>

#define UART_BAUD 115200UL

void     uart_init(void);
uint8_t  uart_read(uint8_t *byte);      /* 1 if a byte was returned */
uint8_t  uart_write(uint8_t byte);      /* 1 if queued, 0 if TX buffer full */
uint16_t uart_rx_overflows(void);

#endif
