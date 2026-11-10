/*
 * Scheduler data types shared by the scheduler, the engine calculation and
 * the protocol (no hardware hooks needed).
 */
#ifndef SCHED_TYPES_H
#define SCHED_TYPES_H

#include <stdint.h>

#define SCHED_DEG10_CYCLE   7200u

/* Bench test configuration ('P' / 'p' commands). While it enables any
 * output it overrides the engine calculation (test mode). */
typedef struct __attribute__((packed)) {
    uint8_t  enable;            /* bit 0 ignition, bit 1 injection; 0 at power-up */
    uint8_t  spare;
    uint16_t tdc_offset;        /* cyl 1 compression TDC after decoder tooth 0, 0.1° */
    int16_t  advance;           /* 0.1° BTDC */
    uint16_t dwell_us;
    uint16_t inj_pw_us;         /* per cycle; halved per revolution in CRANK sync */
    uint16_t inj_end;           /* end of injection, 0.1° before compression TDC */
} sched_cfg_t;                  /* 12 bytes */

#define SCHED_EN_IGN 0x01
#define SCHED_EN_INJ 0x02

/* Limits enforced on the configuration. */
#define SCHED_ADV_MIN      (-200)   /* 20° ATDC */
#define SCHED_ADV_MAX      600      /* 60° BTDC */
#define SCHED_DWELL_MIN    500u
#define SCHED_DWELL_MAX    8000u    /* also hard max dwell */
#define SCHED_PW_MAX       30000u

static inline uint8_t sched_cfg_valid(const sched_cfg_t *c)
{
    return (c->enable & ~(SCHED_EN_IGN | SCHED_EN_INJ)) == 0
        && c->tdc_offset < SCHED_DEG10_CYCLE
        && c->advance >= SCHED_ADV_MIN && c->advance <= SCHED_ADV_MAX
        && c->dwell_us >= SCHED_DWELL_MIN && c->dwell_us <= SCHED_DWELL_MAX
        && c->inj_pw_us <= SCHED_PW_MAX
        && c->inj_end < SCHED_DEG10_CYCLE;
}

/* What the scheduler is asked to do: from the engine calculation, or from
 * the bench test settings (sched_input_from_cfg). Per-cylinder pulse width
 * and advance carry trims (and later knock retard / lambda bank trims). */
typedef struct sched_input {
    uint8_t  enable;            /* SCHED_EN_* */
    uint16_t tdc_offset;        /* 0.1 deg, 0..7199 */
    uint16_t dwell_us;
    uint16_t inj_end;           /* 0.1 deg BTDC, 0..7199 */
    int16_t  advance[6];        /* 0.1 deg BTDC, cylinders 1..6 */
    uint16_t pw_us[6];          /* per cycle (halved per revolution in CRANK sync), 0 = no pulse */
} sched_input_t;

static inline void sched_input_from_cfg(const sched_cfg_t *cfg, sched_input_t *in)
{
    in->enable = sched_cfg_valid(cfg) ? cfg->enable : 0;
    in->tdc_offset = cfg->tdc_offset;
    in->dwell_us = cfg->dwell_us;
    in->inj_end = cfg->inj_end;
    for (uint8_t c = 0; c < 6; c++) {
        in->advance[c] = cfg->advance;
        in->pw_us[c] = cfg->inj_pw_us;
    }
}

#endif
