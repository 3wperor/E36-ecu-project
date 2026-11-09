#include "comms/commands.h"
#include "comms/frame.h"
#include "config.h"
#include "decoder/decoder.h"
#include "decoder/decoder_core.h"
#include "sched/sched.h"
#include "status.h"
#include <string.h>

static uint16_t reply_string(uint8_t *resp, const char *s)
{
    uint16_t n = (uint16_t)strlen(s);
    resp[0] = FRAME_RC_OK;
    memcpy(&resp[1], s, n);
    return (uint16_t)(n + 1u);
}

uint16_t commands_handle(const uint8_t *req, uint16_t req_len, uint8_t *resp)
{
    switch (req[0]) {
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
