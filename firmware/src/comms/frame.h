/*
 * Serial framing, TunerStudio-compatible ("msEnvelope"):
 *
 *     [length: u16 big-endian][payload: length bytes][CRC32(payload): u32 big-endian]
 *
 * Requests: payload[0] is the command character.
 * Responses: payload[0] is a status code (FRAME_RC_*), followed by data.
 *
 * Portable C (no AVR headers) so it is unit-tested on the host.
 */
#ifndef COMMS_FRAME_H
#define COMMS_FRAME_H

#include <stdint.h>
#include "config.h"

/* Response status codes (same values as the TunerStudio serial protocol). */
#define FRAME_RC_OK         0x00
#define FRAME_RC_CRC_ERR    0x82
#define FRAME_RC_UNKNOWN    0x83
#define FRAME_RC_RANGE_ERR  0x84
#define FRAME_RC_BUSY       0x85

#define FRAME_OVERHEAD 6u   /* 2 length + 4 CRC */

typedef enum {
    FRAME_NONE = 0,     /* need more bytes */
    FRAME_OK,           /* complete, CRC valid: payload in rx->buf, rx->len */
    FRAME_BAD_CRC,      /* complete, CRC mismatch */
    FRAME_BAD_LENGTH,   /* length 0 or > FRAME_MAX_PAYLOAD; parser reset */
} frame_result_t;

typedef struct {
    uint8_t  state;
    uint16_t len;
    uint16_t idx;
    uint32_t crc_rx;
    uint32_t last_ms;
    uint8_t  buf[FRAME_MAX_PAYLOAD];
} frame_rx_t;

void           frame_rx_reset(frame_rx_t *rx);

/* Feed one received byte. now_ms is used to drop stale partial frames. */
frame_result_t frame_rx_byte(frame_rx_t *rx, uint8_t byte, uint32_t now_ms);

/* Wrap a payload that has already been written at out[2 .. 2+len-1].
 * Writes the length header and CRC trailer; returns total bytes to send. */
uint16_t       frame_wrap(uint8_t *out, uint16_t len);

#endif
