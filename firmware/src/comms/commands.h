#ifndef COMMS_COMMANDS_H
#define COMMS_COMMANDS_H

#include <stdint.h>

/*
 * Handle one request payload; write the response payload (status byte first)
 * to resp, which has room for FRAME_MAX_PAYLOAD bytes. Returns its length.
 *
 * Phase 1 commands:
 *   'Q'  signature string     (tuner checks it matches its definition)
 *   'S'  version string       (human readable)
 *   'A'  realtime data        (realtime_t)
 *
 * Page read/write/burn and the loggers arrive with the tune storage in
 * Phase 4, together with the generated TunerStudio .ini.
 */
uint16_t commands_handle(const uint8_t *req, uint16_t req_len, uint8_t *resp);

#endif
