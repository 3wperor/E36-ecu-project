#ifndef COMMS_COMMANDS_H
#define COMMS_COMMANDS_H

#include <stdint.h>

/*
 * Handle one request payload; write the response payload (status byte first)
 * to resp, which has room for FRAME_MAX_PAYLOAD bytes. Returns its length.
 *
 * Commands:
 *   'Q'  signature string     (tuner checks it matches its definition)
 *   'S'  version string       (human readable)
 *   'A'  realtime data        (realtime_t)
 *   'T'  tooth log            [count u8][count x (interval u16 LE, pos, flags)]:
 *                             teeth captured since the previous 'T' (max 64),
 *                             oldest first, then a new capture starts.
 *                             See decoder_core.h tooth_log_t.
 *   'P'  + 12 bytes           set output test config (sched_cfg_t); RANGE_ERR if invalid
 *   'p'  output test config   [12 bytes]
 *   'R'  page, offset, count  read tune page bytes (all u16 little-endian)
 *   'M'  page, offset, count, data   write tune page bytes in RAM (live)
 *   'B'  page                 burn page to EEPROM (background; see realtime `engine` bit 6)
 *   'D'  page                 restore the page's defaults in RAM
 *   'O'  offset, count        realtime bytes (TunerStudio output channels)
 *
 * 'R', 'M', 'B', 'O' are the commands named in the generated TunerStudio
 * .ini (tuner/e36ecu.ini), framed with msEnvelope_1.0.
 */
uint16_t commands_handle(const uint8_t *req, uint16_t req_len, uint8_t *resp);

#endif
