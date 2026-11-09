/*
 * Injector / ignition output scheduler — core logic (portable, header-only).
 *
 * Division of work
 * ----------------
 *  main loop  sched_compute(): turns angles (advance, dwell, end of injection)
 *             into, per channel, "start N/256 of a tooth after reference tooth
 *             R, stay on for D ticks", and posts it with sched_post().
 *  crank ISR  sched_tooth(): at reference tooth R arms the channel's output
 *             compare for ICR + N/256 x tooth period (latest speed). Uses a
 *             per-tooth bitmask, so teeth without events cost one lookup.
 *  OC ISR     sched_oc(): the hardware has just switched the pin ON; arm the
 *             OFF edge D ticks later. On the OFF edge, go idle.
 *
 * Both pin edges are produced by output-compare hardware, so ISR latency
 * only affects when an edge is armed, never when it happens.
 *
 * Tooth period: delays are scaled with the sum of the last 4 tooth periods
 * (decoder period4). A single 4 us-quantised period is off by up to ~1 % at
 * 3000 rpm (83 vs 83.3 ticks); the 4-tooth sum keeps the fraction.
 *
 * Coils are scheduled from the spark, not from the dwell start: the start
 * reference tooth is chosen with margin (dwell angle x 1.5 + 1 tooth before
 * the spark), and at that tooth the ISR computes
 *     dwell start = predicted spark time - dwell time
 * from the latest speed. A schedule can be up to one cycle old (it is
 * adopted after the channel's own event), which at low rpm under hard
 * acceleration would otherwise shorten the dwell a lot.
 *
 * Spark refinement: each coil also has a reference tooth for the spark
 * itself. If the coil is dwelling when that tooth arrives, the spark edge is
 * re-timed from the latest tooth (bounded to +/- half the dwell, so dwell can
 * never run away). Spark angle error stays within a fraction of a degree
 * even under hard acceleration.
 *
 * Safety rules
 * ------------
 *  - The OFF edge is armed in the same ISR that sees the ON edge, and a dwell
 *    or pulse can never be shorter than SCHED_MIN_LEAD: an output cannot be
 *    left on (a missed compare would otherwise wait a 262 ms timer wrap).
 *  - An edge that would be armed too close to "now" is moved to
 *    now + SCHED_MIN_LEAD (counted in `late`) rather than missed.
 *  - Reference teeth are chosen at least SCHED_LEAD_UNITS before the event,
 *    so the crank ISR's own latency never makes an event late (up to
 *    ~9000 rpm).
 *  - A channel fires at most once per half period (spacing check), so a
 *    schedule change can never double-fire a cylinder in one cycle, and a
 *    channel adopts a new schedule only right after its own event, so a
 *    change can never skip one.
 *  - Sync loss / stall: pending starts are cancelled; outputs that are
 *    already on finish their (bounded) pulse. Cutting a dwell short would fire
 *    the spark at an unknown crank angle.
 *
 * Sync modes
 * ----------
 *  FULL  (720° known): every channel fires once per cycle (sequential).
 *  CRANK (360° only): every channel fires once per revolution: coils give
 *        wasted spark (compression + exhaust TDC), injectors fire at half
 *        pulse width every revolution.
 *
 * Units
 * -----
 *  angle: 0.1° (deg10), 0..7199, measured from decoder tooth 0 of rev 0.
 *  position: 1/256 tooth ("units"), 0..30719 over 720°.
 *  time: 4 us timer ticks.
 *
 * The includer must define, before including this header:
 *   uint16_t sched_hw_now(void);                    current timer count
 *   void sched_hw_arm(uint8_t ch, uint16_t when);   pin ON at `when`, enable OC irq
 *   void sched_hw_end(uint8_t ch, uint16_t when);   pin OFF at `when`
 *   void sched_hw_idle(uint8_t ch);                 pin stays OFF, OC irq off
 */
#ifndef SCHED_CORE_H
#define SCHED_CORE_H

#include <stdint.h>
#include "decoder/decoder_core.h"

#define SCHED_INLINE static inline __attribute__((always_inline))

/* See DEC_COLD in decoder_core.h: the AVR build makes the per-event work a
 * real function so the crank ISR stays light on teeth without events. */
#ifndef SCHED_COLD
#define SCHED_COLD SCHED_INLINE
#endif

#define SCHED_CHANNELS      12u      /* 0..5 injectors cyl 1..6, 6..11 coils cyl 1..6 */
#define SCHED_INJ(c)        (c)
#define SCHED_IGN(c)        (6u + (c))
#define SCHED_NO_REF        0xFFu

#define SCHED_UNITS_TOOTH   256u
#define SCHED_UNITS_CYCLE   (120u * SCHED_UNITS_TOOTH)   /* 30720 */
#define SCHED_DEG10_CYCLE   7200u

#define SCHED_LEAD_UNITS    128u     /* 0.5 tooth: reference tooth at least this far ahead */
#define SCHED_MIN_LEAD      4u       /* 16 us: closest an edge is ever armed to "now" */
#define SCHED_MIN_DUR       25u      /* 100 us: shortest pulse/dwell */

/* Compression TDC of each cylinder relative to cylinder 1 (0.1°), firing
 * order 1-5-3-6-2-4 at 120° intervals. Index = cylinder - 1. */
static const uint16_t sched_tdc_rel[6] = { 0, 4800, 2400, 6000, 1200, 3600 };

enum { CH_IDLE = 0, CH_WAIT_START = 1, CH_ON = 2 };

/* One channel's schedule as computed by the main loop. */
typedef struct {
    uint8_t  ref;               /* start reference position 0..119 (tooth + 60*rev), SCHED_NO_REF = off */
    uint8_t  e_ref;             /* coils: spark reference position (SCHED_NO_REF for injectors) */
    uint16_t frac;              /* injectors: start delay after ref; coils: spark delay after ref (1/256 tooth) */
    uint16_t e_frac;            /* spark delay after e_ref */
    uint16_t dur;               /* ON time, ticks */
} sched_slot_t;

typedef struct {
    uint8_t  state;
    uint8_t  p_valid;           /* pending schedule, adopted after the next event */
    uint16_t last_arm;          /* tooth counter at the last arm */
    uint16_t end_at;            /* armed OFF time (for refinement bounds) */
    sched_slot_t a;             /* active */
    sched_slot_t p;             /* pending */
} sched_chan_t;

typedef struct {
    sched_chan_t ch[SCHED_CHANNELS];
    uint16_t mask[120];         /* bit n: channel n starts from this position */
    uint8_t  emask[120];        /* bit c: coil c's spark reference is this position */
    uint16_t teeth;             /* physical teeth seen while synced (wraps) */
    uint16_t late;              /* edges pulled forward to now + SCHED_MIN_LEAD */
    uint16_t overlap;           /* start skipped: channel still on from the last event */
    uint16_t armed;             /* starts armed (wraps) */
    uint16_t refined;           /* spark edges re-timed (wraps) */
} sched_t;

/* Test configuration (Phase 3). In Phase 4 the values come from the tune
 * tables; the mechanism stays the same. */
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

SCHED_INLINE uint8_t sched_cfg_valid(const sched_cfg_t *c)
{
    return (c->enable & ~(SCHED_EN_IGN | SCHED_EN_INJ)) == 0
        && c->tdc_offset < SCHED_DEG10_CYCLE
        && c->advance >= SCHED_ADV_MIN && c->advance <= SCHED_ADV_MAX
        && c->dwell_us >= SCHED_DWELL_MIN && c->dwell_us <= SCHED_DWELL_MAX
        && c->inj_pw_us <= SCHED_PW_MAX
        && c->inj_end < SCHED_DEG10_CYCLE;
}

/* ---------------------------------------------------------------- setup -- */

SCHED_INLINE void sched_slot_off(sched_slot_t *s)
{
    s->ref = SCHED_NO_REF;
    s->e_ref = SCHED_NO_REF;
    s->frac = 0;
    s->e_frac = 0;
    s->dur = 0;
}

SCHED_INLINE void sched_init(sched_t *s)
{
    uint8_t *p = (uint8_t *)s;
    for (uint16_t i = 0; i < sizeof *s; i++) p[i] = 0;
    for (uint8_t i = 0; i < SCHED_CHANNELS; i++) {
        sched_slot_off(&s->ch[i].a);
        sched_slot_off(&s->ch[i].p);
    }
}

/* ------------------------------------------------------------ ISR side -- */

/* 16 x 16 -> 32 bit unsigned multiply. On AVR: hardware MUL, no libgcc
 * call (a call would make the crank ISR save every call-clobbered register). */
SCHED_INLINE uint32_t sched_mul16(uint16_t a, uint16_t b)
{
#ifdef __AVR__
    uint32_t r;
    uint8_t z;
    __asm__ ("clr  %[z]"              "\n\t"
             "mul  %A[a], %A[b]"     "\n\t"
             "movw %A[r], r0"        "\n\t"
             "mul  %B[a], %B[b]"     "\n\t"
             "movw %C[r], r0"        "\n\t"
             "mul  %B[a], %A[b]"     "\n\t"
             "add  %B[r], r0"        "\n\t"
             "adc  %C[r], r1"        "\n\t"
             "adc  %D[r], %[z]"      "\n\t"
             "mul  %A[a], %B[b]"     "\n\t"
             "add  %B[r], r0"        "\n\t"
             "adc  %C[r], r1"        "\n\t"
             "adc  %D[r], %[z]"      "\n\t"
             "clr  r1"
             : [r] "=&r" (r), [z] "=&r" (z)
             : [a] "r" (a), [b] "r" (b));
    return r;
#else
    return (uint32_t)a * b;
#endif
}

/* delay ticks = (period4 / 4) * frac / 256 */
SCHED_INLINE uint16_t sched_delay(uint16_t period4, uint16_t frac)
{
    return (uint16_t)(sched_mul16(period4, frac) >> 10);
}

SCHED_INLINE void sched_apply(sched_t *s, uint8_t n, const sched_slot_t *ns)
{
    sched_chan_t *c = &s->ch[n];
    uint16_t bit = (uint16_t)(1u << n);
    if (c->a.ref != SCHED_NO_REF) s->mask[c->a.ref] &= (uint16_t)~bit;
    if (c->a.e_ref != SCHED_NO_REF) s->emask[c->a.e_ref] &= (uint8_t)~(1u << (n - 6u));
    c->a = *ns;
    if (ns->ref != SCHED_NO_REF) s->mask[ns->ref] |= bit;
    if (ns->e_ref != SCHED_NO_REF) s->emask[ns->e_ref] |= (uint8_t)(1u << (n - 6u));
    c->p_valid = 0;
}

SCHED_INLINE void sched_arm(sched_t *s, uint8_t n, uint16_t icr, uint16_t period4, uint8_t spacing)
{
    sched_chan_t *c = &s->ch[n];

    if ((uint16_t)(s->teeth - c->last_arm) < spacing) {
        /* already fired this period (schedule just moved later): skip */
    } else if (c->state != CH_IDLE) {
        s->overlap++;
    } else {
        uint16_t when = (uint16_t)(icr + sched_delay(period4, c->a.frac));
        if (n >= 6u) {
            when = (uint16_t)(when - c->a.dur);         /* coil: spark time - dwell */
        }
        uint16_t now = sched_hw_now();
        if ((int16_t)(when - now) < (int16_t)SCHED_MIN_LEAD) {
            when = (uint16_t)(now + SCHED_MIN_LEAD);
            s->late++;
        }
        sched_hw_arm(n, when);
        c->state = CH_WAIT_START;
        c->last_arm = s->teeth;
        s->armed++;
    }
    if (c->p_valid) {
        sched_apply(s, n, &c->p);
    }
}

/* Spark reference tooth: re-time the OFF edge of a dwelling coil. */
SCHED_INLINE void sched_refine(sched_t *s, uint8_t n, uint16_t icr, uint16_t period4)
{
    sched_chan_t *c = &s->ch[n];
    if (c->state != CH_ON) {
        return;
    }
    uint16_t end = (uint16_t)(icr + sched_delay(period4, c->a.e_frac));
    int16_t shift = (int16_t)(end - c->end_at);
    int16_t bound = (int16_t)(c->a.dur >> 1);
    uint16_t now = sched_hw_now();
    if (shift <= bound && shift >= -bound && (int16_t)(end - now) >= (int16_t)SCHED_MIN_LEAD) {
        sched_hw_end(n, end);
        c->end_at = end;
        s->refined++;
    }
}

/* Teeth that carry events (about 15 % of them). */
SCHED_COLD void sched_events(sched_t *s, uint16_t mask, uint8_t emask, uint16_t icr,
                             uint16_t p4, uint8_t spacing)
{
    for (uint8_t b = 0; emask; b++, emask >>= 1) {
        if (emask & 1u) {
            sched_refine(s, (uint8_t)(6u + b), icr, p4);
        }
    }
    for (uint8_t n = 0; mask; n++, mask >>= 1) {
        if (mask & 1u) {
            sched_arm(s, n, icr, p4, spacing);
        }
    }
}

/* Call from the crank ISR after dec_crank_edge(). */
SCHED_INLINE void sched_tooth(sched_t *s, const decoder_t *d, uint8_t ev, uint16_t icr)
{
    if (!(ev & EV_TOOTH) || d->sync < SYNC_CRANK) {
        return;
    }
    s->teeth++;

    uint16_t mask;
    uint8_t emask;
    uint8_t spacing;
    if (d->sync == SYNC_FULL) {
        uint8_t i = (uint8_t)(d->tooth + (d->rev ? 60u : 0u));
        mask = s->mask[i];
        emask = s->emask[i];
        spacing = 58;                       /* once per 720° */
    } else {
        mask = (uint16_t)(s->mask[d->tooth] | s->mask[d->tooth + 60u]);
        emask = (uint8_t)(s->emask[d->tooth] | s->emask[d->tooth + 60u]);
        spacing = 29;                       /* once per 360° */
    }
    if (mask | emask) {
        sched_events(s, mask, emask, icr, d->period4, spacing);
    }
}

/* Call from channel n's output-compare ISR. */
SCHED_INLINE void sched_oc(sched_t *s, uint8_t n, uint16_t ocr)
{
    sched_chan_t *c = &s->ch[n];
    if (c->state == CH_WAIT_START) {
        uint16_t end = (uint16_t)(ocr + c->a.dur);
        uint16_t now = sched_hw_now();
        if ((int16_t)(end - now) < (int16_t)SCHED_MIN_LEAD) {
            end = (uint16_t)(now + SCHED_MIN_LEAD);
            s->late++;
        }
        sched_hw_end(n, end);
        c->end_at = end;
        c->state = CH_ON;
    } else {
        sched_hw_idle(n);
        c->state = CH_IDLE;
    }
}

/* Sync lost or engine stopped: cancel starts that have not happened yet.
 * Outputs already on run to their armed end. Call with interrupts off. */
SCHED_COLD void sched_cancel_pending(sched_t *s)
{
    for (uint8_t n = 0; n < SCHED_CHANNELS; n++) {
        if (s->ch[n].state == CH_WAIT_START) {
            sched_hw_idle(n);
            s->ch[n].state = CH_IDLE;
        }
    }
}

/* ------------------------------------------------------ main-loop side -- */

/* 0.1° -> 1/256 tooth (6° = 60 deg10 = 256 units): x 64/15, done as a
 * multiply-shift (a 32-bit division costs ~40 us on AVR). */
SCHED_INLINE uint16_t sched_deg10_to_units(uint16_t deg10)
{
    return (uint16_t)(((uint32_t)deg10 * 279621UL + 32768UL) >> 16);
}

/* Callers only pass values within two cycles of the range, so add/subtract
 * replaces the (slow) modulo. */
SCHED_INLINE uint16_t sched_wrap_units(int32_t u)
{
    while (u < 0) u += SCHED_UNITS_CYCLE;
    while (u >= (int32_t)SCHED_UNITS_CYCLE) u -= SCHED_UNITS_CYCLE;
    return (uint16_t)u;
}

/* Event position -> reference tooth (a physical tooth at least
 * SCHED_LEAD_UNITS earlier) and delay after it. */
SCHED_INLINE void sched_to_ref(uint16_t pos, uint8_t *ref, uint16_t *frac)
{
    uint16_t p = sched_wrap_units((int32_t)pos - (int32_t)SCHED_LEAD_UNITS);
    uint8_t r = (uint8_t)(p >> 8);              /* 0..119 */
    uint8_t tooth = (uint8_t)(r >= 60 ? r - 60 : r);
    if (tooth > DEC_LAST_TOOTH) {               /* inside the gap: back to tooth 57 */
        r = (uint8_t)(r - (tooth - DEC_LAST_TOOTH));
    }
    *ref = r;
    *frac = sched_wrap_units((int32_t)pos - (int32_t)r * SCHED_UNITS_TOOTH);
}

/* Main loop: post a new schedule for channel n. Call with interrupts off. */
SCHED_INLINE void sched_post(sched_t *s, uint8_t n, const sched_slot_t *ns, uint8_t synced)
{
    sched_chan_t *c = &s->ch[n];
    const sched_slot_t *a = &c->a;
    if (a->ref == ns->ref && a->frac == ns->frac && a->dur == ns->dur &&
        a->e_ref == ns->e_ref && a->e_frac == ns->e_frac) {
        c->p_valid = 0;
        return;
    }
    /* Adopt at once when it cannot cause a skip or double fire: channel not
     * scheduled yet, being switched off, or engine not synced. */
    if (a->ref == SCHED_NO_REF || ns->ref == SCHED_NO_REF || !synced) {
        sched_apply(s, n, ns);
    } else {
        c->p = *ns;
        c->p_valid = 1;
    }
}

/* Time (ticks) -> angle (units). Uses the last full revolution when known
 * (60 tooth periods: much finer than 4), else the 4-tooth sum. */
SCHED_INLINE uint32_t sched_ticks_to_units(uint32_t ticks, uint16_t period4, uint32_t rev_ticks)
{
    if (rev_ticks) {
        return (ticks * (60u * SCHED_UNITS_TOOTH)) / rev_ticks;
    }
    return (ticks * 1024u) / period4;
}

/*
 * Compute all 12 channels from the configuration, period4 (sum of the last 4
 * tooth periods, ticks) and rev_ticks (last revolution, ticks; 0 = unknown).
 * Pure function, so the main loop can run it outside the atomic block and
 * then post the results.
 */
SCHED_INLINE void sched_compute(const sched_cfg_t *cfg, uint8_t sync, uint16_t period4,
                                uint32_t rev_ticks, sched_slot_t out[SCHED_CHANNELS])
{
    for (uint8_t n = 0; n < SCHED_CHANNELS; n++) {
        sched_slot_off(&out[n]);
    }
    if (sync < SYNC_CRANK || period4 == 0 || !sched_cfg_valid(cfg)) {
        return;
    }

    /* Period of one channel's events: 720° (FULL) or 360° (CRANK), in ticks
     * (= 120 or 60 tooth periods = period4 x 30 or x 15). */
    if (rev_ticks > 60u * 65535u) rev_ticks = 0;
    uint32_t rev = rev_ticks ? rev_ticks : (uint32_t)period4 * 15u;
    uint32_t period = sync == SYNC_FULL ? rev * 2u : rev;

    /* Dwell: time-based, as an angle at the current speed. */
    uint32_t dwell = cfg->dwell_us / 4u;
    if (dwell > period / 2) dwell = period / 2;                  /* never > half the period */
    if (dwell < SCHED_MIN_DUR) dwell = SCHED_MIN_DUR;
    uint32_t dwell_u = sched_ticks_to_units(dwell, period4, rev_ticks);

    /* Injection: per revolution at half width in CRANK sync; duty <= 90 %. */
    uint32_t pw = cfg->inj_pw_us / 4u;
    if (sync != SYNC_FULL) pw /= 2u;
    if (pw > period * 9u / 10u) pw = period * 9u / 10u;
    uint32_t pw_u = sched_ticks_to_units(pw, period4, rev_ticks);

    /* advance (may be negative) and end of injection, in units */
    int32_t adv_u = cfg->advance >= 0 ? (int32_t)sched_deg10_to_units((uint16_t)cfg->advance)
                                      : -(int32_t)sched_deg10_to_units((uint16_t)-cfg->advance);
    uint16_t eoi_u = sched_deg10_to_units(cfg->inj_end);

    for (uint8_t c = 0; c < 6; c++) {
        uint16_t tdc = (uint16_t)(cfg->tdc_offset + sched_tdc_rel[c]);
        if (tdc >= SCHED_DEG10_CYCLE) tdc = (uint16_t)(tdc - SCHED_DEG10_CYCLE);
        uint16_t tdc_u = sched_deg10_to_units(tdc);

        if ((cfg->enable & SCHED_EN_IGN)) {
            sched_slot_t *o = &out[SCHED_IGN(c)];
            uint16_t spark = sched_wrap_units((int32_t)tdc_u - adv_u);
            /* start reference: dwell angle x 1.5 + 1 tooth before the spark */
            uint16_t margin = (uint16_t)(dwell_u + dwell_u / 2u + SCHED_UNITS_TOOTH);
            uint16_t start = sched_wrap_units((int32_t)spark - (int32_t)margin);
            uint16_t unused;
            sched_to_ref(start, &o->ref, &unused);
            o->frac = sched_wrap_units((int32_t)spark - (int32_t)o->ref * SCHED_UNITS_TOOTH);
            sched_to_ref(spark, &o->e_ref, &o->e_frac);
            o->dur = (uint16_t)dwell;
        }
        if ((cfg->enable & SCHED_EN_INJ) && pw >= SCHED_MIN_DUR) {
            sched_slot_t *o = &out[SCHED_INJ(c)];
            uint16_t end_u = sched_wrap_units((int32_t)tdc_u - (int32_t)eoi_u);
            uint16_t start = sched_wrap_units((int32_t)end_u - (int32_t)pw_u);
            sched_to_ref(start, &o->ref, &o->frac);
            o->dur = (uint16_t)pw;
        }
    }
}

#endif
