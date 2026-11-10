/*
 * Engine calculation — main-loop API. Reads the sensors, runs the fuel and
 * ignition calculation (engine_core.h) and fills the scheduler input and the
 * realtime data.
 */
#ifndef ENGINE_H
#define ENGINE_H

#include <stdint.h>
#include "decoder/decoder.h"
#include "sched/sched_types.h"

void engine_init(void);

/* 100 Hz. Fills *out from the tune and sensors. */
void engine_update(const decoder_status_t *ds, sched_input_t *out);

#endif
