#include "comms/commands.h"
#include "comms/frame.h"
#include "config.h"
#include "decoder/decoder.h"
#include "decoder/decoder_core.h"
#include "sched/sched.h"
#include "status.h"
#include "tune/tune.h"
#include <string.h>

static uint16_t reply_string(uint8_t *resp, const char *s)
{
    uint16_t n = (uint16_t)strlen(s);
    resp[0] = FRAME_RC_OK;
    memcpy(&resp[1], s, n);
    return (uint16_t)(n + 1u);
}

static uint16_t rd16le(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint16_t reply_code(uint8_t *resp, uint8_t code)
{
    resp[0] = code;
    return 1;
}

uint16_t commands_handle(const uint8_t *req, uint16_t req_len, uint8_t *resp)
{
    switch (req[0]) {
    case 'R': {                 /* page read: page, offset, count (u16 LE each) */
        if (req_len != 7u) return reply_code(resp, FRAME_RC_RANGE_ERR);
        uint16_t size;
        uint8_t *pg = tune_page((uint8_t)rd16le(&req[1]), &size);
        uint16_t off = rd16le(&req[3]), cnt = rd16le(&req[5]);
        if (!pg || rd16le(&req[1]) >= TUNE_PAGES || cnt > FRAME_MAX_PAYLOAD - 1u ||
            (uint32_t)off + cnt > size) {
            return reply_code(resp, FRAME_RC_RANGE_ERR);
        }
        resp[0] = FRAME_RC_OK;
        memcpy(&resp[1], pg + off, cnt);
        return (uint16_t)(1u + cnt);
    }

    case 'M': {                 /* page write (RAM): page, offset, count, data */
        if (req_len < 7u) return reply_code(resp, FRAME_RC_RANGE_ERR);
        uint16_t size;
        uint8_t page = (uint8_t)rd16le(&req[1]);
        uint8_t *pg = tune_page(page, &size);
        uint16_t off = rd16le(&req[3]), cnt = rd16le(&req[5]);
        if (!pg || rd16le(&req[1]) >= TUNE_PAGES || req_len != 7u + cnt || (uint32_t)off + cnt > size) {
            return reply_code(resp, FRAME_RC_RANGE_ERR);
        }
        memcpy(pg + off, &req[7], cnt);
        tune_modified(page);
        return reply_code(resp, FRAME_RC_OK);
    }

    case 'B':                   /* burn page to EEPROM (completes in the background) */
        if (req_len != 3u || rd16le(&req[1]) >= TUNE_PAGES || !tune_burn((uint8_t)rd16le(&req[1]))) {
            return reply_code(resp, FRAME_RC_RANGE_ERR);
        }
        return reply_code(resp, FRAME_RC_OK);

    case 'D':                   /* restore a page's defaults into RAM (burn to keep) */
        if (req_len != 3u || rd16le(&req[1]) >= TUNE_PAGES || !tune_restore_defaults((uint8_t)rd16le(&req[1]))) {
            return reply_code(resp, FRAME_RC_RANGE_ERR);
        }
        return reply_code(resp, FRAME_RC_OK);

    case 'O': {                 /* realtime block: offset, count (u16 LE) */
        if (req_len != 5u) return reply_code(resp, FRAME_RC_RANGE_ERR);
        uint16_t off = rd16le(&req[1]), cnt = rd16le(&req[3]);
        if ((uint32_t)off + cnt > sizeof rt || cnt > FRAME_MAX_PAYLOAD - 1u) {
            return reply_code(resp, FRAME_RC_RANGE_ERR);
        }
        resp[0] = FRAME_RC_OK;
        memcpy(&resp[1], (const uint8_t *)&rt + off, cnt);
        return (uint16_t)(1u + cnt);
    }

    case 'Q':
        return reply_string(resp, ECU_SIGNATURE);

    case 'S':
        return reply_string(resp, ECU_VERSION);

    case 'A':
        resp[0] = FRAME_RC_OK;
        memcpy(&resp[1], &rt, sizeof rt);
        return (uint16_t)(sizeof rt + 1u);

    case 'T': {
        resp[0] = FRAME_RC_OK;
        uint8_t count = decoder_take_log(&resp[2]);
        resp[1] = count;
        return (uint16_t)(2u + 4u * count);
    }

    case 'P':
        if (req_len != 1u + SCHED_CFG_SIZE || !sched_set_cfg(&req[1])) {
            resp[0] = FRAME_RC_RANGE_ERR;
            return 1;
        }
        resp[0] = FRAME_RC_OK;
        return 1;

    case 'p':
        resp[0] = FRAME_RC_OK;
        sched_get_cfg(&resp[1]);
        return (uint16_t)(1u + SCHED_CFG_SIZE);

    default:
        resp[0] = FRAME_RC_UNKNOWN;
        return 1;
    }
}
