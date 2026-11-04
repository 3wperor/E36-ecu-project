#include "hal/uart.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define RX_SIZE 128u   /* power of two */
#define TX_SIZE 256u   /* 256 so uint8_t indices wrap naturally */

static volatile uint8_t rx_buf[RX_SIZE];
static volatile uint8_t rx_head, rx_tail;
static volatile uint8_t tx_buf[TX_SIZE];
static volatile uint8_t tx_head, tx_tail;
static volatile uint16_t rx_overflow;

ISR(USART0_RX_vect)
{
    uint8_t c = UDR0;
    uint8_t next = (uint8_t)((rx_head + 1u) & (RX_SIZE - 1u));
    if (next != rx_tail) {
        rx_buf[rx_head] = c;
        rx_head = next;
    } else {
        rx_overflow++;
    }
}

ISR(USART0_UDRE_vect)
{
    if (tx_head == tx_tail) {
        UCSR0B &= (uint8_t)~(1 << UDRIE0);
    } else {
        UDR0 = tx_buf[tx_tail];
        tx_tail++;
    }
}

void uart_init(void)
{
    /* Double speed mode: UBRR = F_CPU / (8 * baud) - 1, rounded. */
    const uint16_t ubrr = (uint16_t)((F_CPU + 4UL * UART_BAUD) / (8UL * UART_BAUD) - 1UL);
    UBRR0  = ubrr;
    UCSR0A = (1 << U2X0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);           /* 8N1 */
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
}

uint8_t uart_read(uint8_t *byte)
{
    if (rx_head == rx_tail) {
        return 0;
    }
    *byte = rx_buf[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1u) & (RX_SIZE - 1u));
    return 1;
}

uint8_t uart_write(uint8_t byte)
{
    uint8_t next = (uint8_t)(tx_head + 1u);
    if (next == tx_tail) {
        return 0;
    }
    tx_buf[tx_head] = byte;
    tx_head = next;
    UCSR0B |= (1 << UDRIE0);
    return 1;
}

uint16_t uart_rx_overflows(void)
{
    uint16_t v;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = rx_overflow; }
    return v;
}
