#include "comms/frame.h"
#include "comms/crc32.h"

enum { ST_LEN_HI = 0, ST_LEN_LO, ST_PAYLOAD, ST_CRC };

void frame_rx_reset(frame_rx_t *rx)
{
    rx->state = ST_LEN_HI;
    rx->len = 0;
    rx->idx = 0;
    rx->crc_rx = 0;
}

frame_result_t frame_rx_byte(frame_rx_t *rx, uint8_t byte, uint32_t now_ms)
{
    if (rx->state != ST_LEN_HI && (uint32_t)(now_ms - rx->last_ms) > FRAME_TIMEOUT_MS) {
        frame_rx_reset(rx);
    }
    rx->last_ms = now_ms;

    switch (rx->state) {
    case ST_LEN_HI:
        rx->len = (uint16_t)byte << 8;
        rx->state = ST_LEN_LO;
        return FRAME_NONE;

    case ST_LEN_LO:
        rx->len |= byte;
        if (rx->len == 0 || rx->len > FRAME_MAX_PAYLOAD) {
            frame_rx_reset(rx);
            return FRAME_BAD_LENGTH;
        }
        rx->idx = 0;
        rx->state = ST_PAYLOAD;
        return FRAME_NONE;

    case ST_PAYLOAD:
        rx->buf[rx->idx++] = byte;
        if (rx->idx == rx->len) {
            rx->idx = 0;
            rx->crc_rx = 0;
            rx->state = ST_CRC;
        }
        return FRAME_NONE;

    case ST_CRC:
    default:
        rx->crc_rx = (rx->crc_rx << 8) | byte;
        if (++rx->idx < 4) {
            return FRAME_NONE;
        }
        rx->state = ST_LEN_HI;
        return (crc32(rx->buf, rx->len) == rx->crc_rx) ? FRAME_OK : FRAME_BAD_CRC;
    }
}

uint16_t frame_wrap(uint8_t *out, uint16_t len)
{
    out[0] = (uint8_t)(len >> 8);
    out[1] = (uint8_t)len;
    uint32_t crc = crc32(&out[2], len);
    uint8_t *t = &out[2 + len];
    t[0] = (uint8_t)(crc >> 24);
    t[1] = (uint8_t)(crc >> 16);
    t[2] = (uint8_t)(crc >> 8);
    t[3] = (uint8_t)crc;
    return (uint16_t)(len + FRAME_OVERHEAD);
}
