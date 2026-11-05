#include "comms/commands.h"
#include "comms/frame.h"
#include "config.h"
#include "decoder/decoder.h"
#include "decoder/decoder_core.h"
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
    (void)req_len;

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

    default:
        resp[0] = FRAME_RC_UNKNOWN;
        return 1;
    }
}
