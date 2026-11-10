/*
 * Realtime data sent to the tuner. The layout (realtime_t) is generated from
 * definitions/ecu.toml into generated/realtime.h; change it there.
 */
#ifndef STATUS_H
#define STATUS_H

#include <stdint.h>
#include "config.h"
#include "generated/realtime.h"

#define RT_FLAG_TIMERS_SYNCED  0x01

extern realtime_t rt;

#endif
