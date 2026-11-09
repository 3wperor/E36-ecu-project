/*
 * Crank/cam decoder — AVR side. Input capture on ICP5 (crank, PL1) and ICP4
 * (cam, PL0); both timers share the synchronised 4 us time base.
 */
#ifndef DECODER_H
#define DECODER_H

#include <stdint.h>

typedef struct {
    uint8_t  sync;          /* SYNC_* */
    uint8_t  rev;
    uint8_t  cam_pos;       /* tooth (+60 for rev 1) where the cam edge falls, 0xFF = none yet */
    uint8_t  loss_reason;   /* LOSS_* of the most recent loss */
    uint16_t rpm;           /* averaged over the last revolution */
    uint16_t rpm_inst;      /* from the last tooth */
    uint16_t sync_losses;
    uint16_t noise_teeth;
    uint16_t cam_errors;
    uint16_t tooth_period;  /* ticks, 0 = unknown */
    uint16_t period4;       /* last 4 tooth periods summed (ticks), 0 = unknown */
    uint32_t rev_ticks;     /* last full revolution (ticks), 0 = unknown */
} decoder_status_t;

/* cam_enabled = 0 runs crank-only (never leaves CRANK sync). */
void decoder_init(uint8_t cam_enabled);

/* Main loop, at least every 50 ms: stall detection + RPM. */
void decoder_poll(decoder_status_t *out);

/* Tooth logger: copy the teeth captured since the previous call (up to
 * DEC_LOG_SIZE), oldest first, as 4-byte entries (interval u16 LE, pos,
 * flags), then start a new capture. Returns the number of entries. */
uint8_t decoder_take_log(uint8_t *out);

#endif
