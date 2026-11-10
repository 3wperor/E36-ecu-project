/*
 * Knock control — core logic (portable, header-only). Chip-independent: it
 * works on one integrated knock level (0..1023) per cylinder firing.
 *
 *  windows   knock_build_windows(): for each cylinder, the crank tooth where
 *            the chip starts integrating (window open, degrees after that
 *            cylinder's compression TDC) and the tooth where it stops. The
 *            crank ISR looks the current tooth up in a 120-entry table.
 *  levels    knock_sample(): after each window, the chip's level for that
 *            cylinder. Each cylinder learns its own background noise (a
 *            running average of non-knocking windows). Knock = level >
 *            noise x threshold(rpm).
 *  response  that cylinder loses `knockStep` degrees (twice for heavy knock,
 *            > 1.5 x threshold), up to `knockMaxRetard`; knock_tick() gives
 *            timing back at `knockRecover` deg/s while there is no knock.
 *  safety    a chip that keeps returning 0 or full scale is a fault: every
 *            cylinder then runs `knockFaultRetard` (or its own retard if
 *            larger) until good readings return.
 *
 * Sensors: cylinders 1-3 on sensor 1 (chip channel 0), 4-6 on sensor 2
 * (channel 1). Firing order 1-5-3-6-2-4 alternates the channel every firing.
 */
#ifndef KNOCK_CORE_H
#define KNOCK_CORE_H

#include <stdint.h>
#include "generated/tune_def.h"

#define KNOCK_INLINE static inline

#define KNOCK_OPEN    0x80      /* window table: start integrating */
#define KNOCK_CLOSE   0x40      /* stop integrating, read the level */
#define KNOCK_CYL     0x07

#define KNOCK_ST_ENABLED 0x01
#define KNOCK_ST_ACTIVE  0x02
#define KNOCK_ST_KNOCK   0x04
#define KNOCK_ST_FAULT   0x08

#define KNOCK_WARMUP_SAMPLES 16u   /* windows per cylinder before detection starts */
#define KNOCK_FAULT_SAMPLES  8u    /* consecutive impossible readings -> fault */
#define KNOCK_SEEN_TICKS     50u   /* "knock" status bit held 0.5 s */

/* Compression TDC of each cylinder relative to cylinder 1, 0.1 deg
 * (same as the scheduler: firing order 1-5-3-6-2-4). */
static const uint16_t knock_tdc_rel[6] = { 0, 4800, 2400, 6000, 1200, 3600 };

/* Firing order, and the cylinder that fires after each (0-based). */
static const uint8_t knock_next_cyl[6] = { 4, 3, 5, 0, 2, 1 };

KNOCK_INLINE uint8_t knock_channel(uint8_t cyl) { return cyl < 3 ? 0 : 1; }

typedef struct {
    uint16_t noise16[6];        /* learned background, x16 */
    uint16_t level[6];          /* last window level */
    uint8_t  samples[6];        /* windows seen, saturates at KNOCK_WARMUP_SAMPLES */
    uint8_t  retard[6];         /* 0.1 deg */
    uint16_t recover_acc[6];
    uint16_t count;             /* knock events (wraps) */
    uint8_t  bad;               /* consecutive impossible readings */
    uint8_t  fault;
    uint8_t  seen;              /* ticks left showing "knock" */
    uint8_t  state;             /* KNOCK_ST_* */
} knock_t;

KNOCK_INLINE void knock_reset(knock_t *k)
{
    uint8_t *p = (uint8_t *)k;
    for (uint16_t i = 0; i < sizeof *k; i++) p[i] = 0;
}

/* Crank position (0..119, tooth + 60 x rev) of the first physical tooth at
 * or after an angle (0.1 deg after decoder tooth 0). Missing teeth 58/59
 * move to the next tooth 0. */
KNOCK_INLINE uint8_t knock_angle_to_pos(uint16_t deg10)
{
    uint8_t pos = (uint8_t)((deg10 + 59u) / 60u);              /* 6 deg per tooth, round up */
    if (pos >= 120u) pos = (uint8_t)(pos - 120u);
    uint8_t tooth = (uint8_t)(pos >= 60u ? pos - 60u : pos);
    if (tooth >= 58u) {
        pos = (uint8_t)(pos + (60u - tooth));                    /* to tooth 0 of the next rev */
        if (pos >= 120u) pos = (uint8_t)(pos - 120u);
    }
    return pos;
}

/* Fill the 120-entry window table. Window length is limited to 96 deg:
 * firings are 120 deg apart, so at least 4 teeth separate a window's close
 * from the next open, enough for the missing-tooth shift (up to 2 teeth) and
 * for reading the chip on the tooth after the close. */
KNOCK_INLINE void knock_build_windows(uint8_t act[120], uint16_t tdc_offset_x10, uint8_t start_deg, uint8_t len_deg)
{
    for (uint8_t i = 0; i < 120; i++) act[i] = 0;
    if (len_deg < 6) len_deg = 6;
    if (len_deg > 96) len_deg = 96;
    for (uint8_t c = 0; c < 6; c++) {
        uint32_t open = (uint32_t)tdc_offset_x10 + knock_tdc_rel[c] + (uint32_t)start_deg * 10u;
        uint32_t close = open + (uint32_t)len_deg * 10u;
        uint8_t po = knock_angle_to_pos((uint16_t)(open % 7200u));
        uint8_t pc = knock_angle_to_pos((uint16_t)(close % 7200u));
        if (pc == po) {
            /* a short window inside the missing-tooth gap: both edges moved to
             * the same tooth; close on the next one instead */
            pc = (uint8_t)(po + 1u >= 120u ? 0u : po + 1u);
        }
        act[po] = (uint8_t)(KNOCK_OPEN | c);
        act[pc] = (uint8_t)(KNOCK_CLOSE | c);
    }
}

/* Two SPI result bytes -> 10-bit level (order: tune knockReadOrder). */
KNOCK_INLINE uint16_t knock_parse(uint8_t b0, uint8_t b1, uint8_t lsb_first)
{
    if (lsb_first) return (uint16_t)(b0 | ((uint16_t)(b1 & 0x03u) << 8));
    return (uint16_t)(((uint16_t)b0 << 2) | (b1 >> 6));
}

/* Knock threshold (x10) at rpm, from the tune curve. */
KNOCK_INLINE uint16_t knock_threshold_x10(uint16_t rpm)
{
    uint16_t r0 = (uint16_t)tu_knockThrRpm(0) * 100u;
    if (rpm <= r0) return tu_knockThr(0);
    for (uint8_t i = 1; i < TU_KNOCK_THR_RPM_N; i++) {
        uint16_t r1 = (uint16_t)tu_knockThrRpm(i) * 100u;
        if (rpm <= r1 && r1 > r0) {
            int32_t t0 = tu_knockThr(i - 1u), t1 = tu_knockThr(i);
            return (uint16_t)(t0 + (t1 - t0) * (int32_t)(rpm - r0) / (int32_t)(r1 - r0));
        }
        r0 = r1;
    }
    return tu_knockThr(TU_KNOCK_THR_RPM_N - 1u);
}

/* One window result for cylinder `cyl`. `active` = enabled and the engine
 * is in the knock-control range (rpm, load, coolant, full sync). */
KNOCK_INLINE void knock_sample(knock_t *k, uint8_t cyl, uint16_t level, uint8_t active, uint16_t rpm)
{
    if (cyl >= 6) return;
    k->level[cyl] = level;

    /* A real sensor always produces some noise and never sits at the rail. */
    if (level == 0 || level >= 1023u) {
        if (k->bad < 255u) k->bad++;
        if (k->bad >= KNOCK_FAULT_SAMPLES) k->fault = 1;
        return;
    }
    k->bad = 0;
    k->fault = 0;

    uint32_t l16 = (uint32_t)level << 4;
    if (k->samples[cyl] < KNOCK_WARMUP_SAMPLES) {
        /* fast start: plain average of the first windows */
        k->samples[cyl]++;
        k->noise16[cyl] = (uint16_t)(k->noise16[cyl] + ((int32_t)l16 - (int32_t)k->noise16[cyl]) / k->samples[cyl]);
        return;
    }

    uint32_t limit = ((uint32_t)k->noise16[cyl] * knock_threshold_x10(rpm)) / 10u;
    uint8_t n = tu_knockLearn();
    if (n < 1) n = 1;
    if (n > 6) n = 6;
    if (l16 > limit && !active) n = (uint8_t)(n + 2u);     /* loud but not controlling: learn slowly */
    if (active && l16 > limit) {
        uint8_t step = tu_knockStep();
        if (l16 * 2u > limit * 3u) step = (uint8_t)(step * 2u);     /* heavy knock */
        uint16_t r = (uint16_t)k->retard[cyl] + step;
        uint8_t max = tu_knockMaxRetard();
        k->retard[cyl] = (uint8_t)(r > max ? max : r);
        k->recover_acc[cyl] = 0;
        k->count++;
        k->seen = KNOCK_SEEN_TICKS;
        return;                         /* knocking windows do not teach the noise */
    }
    /* learn the background: noise += (level - noise) / 2^n */
    k->noise16[cyl] = (uint16_t)((int32_t)k->noise16[cyl] + (((int32_t)l16 - (int32_t)k->noise16[cyl]) >> n));
}

/* Every 10 ms: give timing back, update the status. */
KNOCK_INLINE void knock_tick(knock_t *k, uint8_t enabled, uint8_t active)
{
    if (!enabled) {
        for (uint8_t c = 0; c < 6; c++) { k->retard[c] = 0; k->recover_acc[c] = 0; }
        k->fault = 0;
        k->bad = 0;
    }
    uint8_t rate = tu_knockRecover();               /* 0.1 deg per second */
    for (uint8_t c = 0; c < 6; c++) {
        if (!k->retard[c]) continue;
        k->recover_acc[c] = (uint16_t)(k->recover_acc[c] + rate);
        while (k->recover_acc[c] >= 100u && k->retard[c]) {   /* 100 ticks x 0.1 deg/s = 0.1 deg */
            k->recover_acc[c] = (uint16_t)(k->recover_acc[c] - 100u);
            k->retard[c]--;
        }
    }
    if (k->seen) k->seen--;
    k->state = (uint8_t)((enabled ? KNOCK_ST_ENABLED : 0) | (active ? KNOCK_ST_ACTIVE : 0) |
                         (k->seen ? KNOCK_ST_KNOCK : 0) | (k->fault ? KNOCK_ST_FAULT : 0));
}

/* Timing to remove from cylinder c (0.1 deg). */
KNOCK_INLINE uint8_t knock_retard(const knock_t *k, uint8_t c)
{
    uint8_t r = k->retard[c];
    if (k->fault && tu_knockFaultRetard() > r) r = tu_knockFaultRetard();
    return r;
}

#endif
