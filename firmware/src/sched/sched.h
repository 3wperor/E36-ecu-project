/*
 * Output scheduler — main-loop API (portable declarations).
 */
#ifndef SCHED_H
#define SCHED_H

#include <stdint.h>
#include "sched/sched_types.h"

/* Same layout as sched_cfg_t in sched_core.h (12 bytes, little-endian):
 * enable u8, spare u8, tdc_offset u16, advance s16, dwell_us u16,
 * inj_pw_us u16, inj_end u16. */
#define SCHED_CFG_SIZE 12u

/* Connect the 12 output-compare channels to their pins, all OFF. Call
 * after timers_init(). Outputs stay disabled until a valid config with
 * enable bits arrives. */
void sched_init_hw(void);

/* Bench test settings ('P'/'p' commands). While they enable any output they
 * override the engine calculation ("test mode"). Returns 1 and applies the
 * config if every field is in range, else 0. */
uint8_t sched_set_cfg(const uint8_t *raw12);
void    sched_get_cfg(uint8_t *raw12);
uint8_t sched_test_active(void);
void    sched_test_input(sched_input_t *in);

/* Main loop, 100 Hz: recompute and post the 12 channel schedules.
 * period4 = last 4 tooth periods summed, rev_ticks = last revolution
 * (decoder_status_t). */
void sched_update(uint8_t sync, uint16_t period4, uint32_t rev_ticks, const sched_input_t *in);

typedef struct {
    uint8_t  enable;
    uint16_t late;
    uint16_t overlap;
} sched_stats_t;

void sched_get_stats(sched_stats_t *st);

#endif
