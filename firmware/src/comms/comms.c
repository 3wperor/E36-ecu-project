#include "comms/comms.h"
#include "comms/commands.h"
#include "comms/frame.h"
#include "hal/uart.h"
#include "status.h"

static frame_rx_t rx;
static uint8_t  tx[FRAME_MAX_PAYLOAD + FRAME_OVERHEAD];
static uint16_t tx_len, tx_pos;

void comms_init(void)
{
    frame_rx_reset(&rx);
    tx_len = tx_pos = 0;
}

static void send_status_only(uint8_t code)
{
    tx[2] = code;
    tx_len = frame_wrap(tx, 1);
    tx_pos = 0;
}

void comms_poll(uint32_t now_ms)
{
    /* Finish sending the current response before reading the next request. */
    if (tx_pos < tx_len) {
        while (tx_pos < tx_len && uart_write(tx[tx_pos])) {
            tx_pos++;
        }
        return;
    }

    uint8_t b;
    while (uart_read(&b)) {
        frame_result_t r = frame_rx_byte(&rx, b, now_ms);
        if (r == FRAME_OK) {
            uint16_t n = commands_handle(rx.buf, rx.len, &tx[2]);
            tx_len = frame_wrap(tx, n);
            tx_pos = 0;
            return;
        }
        if (r == FRAME_BAD_CRC) {
            rt.frame_errors++;
            send_status_only(FRAME_RC_CRC_ERR);
            return;
        }
        if (r == FRAME_BAD_LENGTH) {
            rt.frame_errors++;
        }
    }
}
