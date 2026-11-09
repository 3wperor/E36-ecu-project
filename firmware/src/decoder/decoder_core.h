/*
 * M52 60-2 crank + cam decoder — core logic.
 *
 * Portable C, header-only and force-inlined: the AVR input-capture ISRs call
 * these directly (no function call, so the ISR prologue stays small), and the
 * host unit tests call exactly the same code.
 *
 * Inputs are 16-bit timestamps in 4 us ticks from the hardware input-capture
 * registers (ICR5 crank, ICR4 cam), so ISR latency never affects accuracy.
 *
 * Sync states:
 *   NONE     no gap found yet (or lost) — outputs must be off
 *   PENDING  one gap found; counting teeth to confirm the next gap
 *   CRANK    two consecutive gaps exactly 58 teeth apart: 360° position known
 *            (wasted spark + paired injection possible)
 *   FULL     cam phase confirmed: 720° position known (full sequential)
 *
 * Cam phase rule (works for a single-pulse or a half-moon cam wheel, since
 * both give one active edge per 720°): the revolution that STARTS at the gap
 * following the cam edge is rev 0, the next is rev 1. Entering FULL needs two
 * consecutive revolutions that alternate (cam seen / not seen). In FULL every
 * revolution must keep alternating, otherwise the decoder drops back to CRANK.
 * The cam edge must sit well away from the gap across the VANOS range;
 * cam_pos reports where it is seen, for checking that on the car.
 */
#ifndef DECODER_CORE_H
#define DECODER_CORE_H

#include <stdint.h>

#define DEC_INLINE static inline __attribute__((always_inline))

/* Rarely taken paths (once per revolution). The AVR build defines this as a
 * real function before including, so the crank ISR's register save covers
 * only the per-tooth path; the cold function saves its own registers only
 * when it runs. Host tests just inline it. */
#ifndef DEC_COLD
#define DEC_COLD DEC_INLINE
#endif

#define DEC_LAST_TOOTH     57u          /* physical teeth 0..57, then the gap */
#define DEC_TICKS_PER_MIN  15000000UL   /* 60 s / 4 us */
#define DEC_STALL_TICKS    50000u       /* 200 ms without a tooth = stopped */
#define DEC_LOG_SIZE       64u          /* power of two */

enum { SYNC_NONE = 0, SYNC_PENDING = 1, SYNC_CRANK = 2, SYNC_FULL = 3 };
enum { LOSS_NONE = 0, LOSS_COUNT = 1, LOSS_NO_GAP = 2, LOSS_STALL = 3 };

/* Events returned by dec_crank_edge(). */
#define EV_TOOTH      0x01
#define EV_GAP        0x02
#define EV_SYNC_LOST  0x04
#define EV_FULL       0x08
#define EV_CAM_ERR    0x10
#define EV_NOISE      0x20

/* Tooth-log entry flags (low nibble), sync state in the high nibble. */
#define LOG_GAP    0x01
#define LOG_CAM    0x02
#define LOG_NOISE  0x04

typedef struct {
    uint16_t interval;      /* ticks since previous accepted tooth */
    uint8_t  pos;           /* tooth index, bit 7 = rev (valid in FULL) */
    uint8_t  flags;         /* LOG_* | sync << 4 */
} tooth_log_t;

typedef struct {
    /* configuration */
    uint8_t  cam_enabled;

    /* position */
    uint8_t  sync;
    uint8_t  tooth;
    uint8_t  rev;
    uint8_t  has_time;
    uint8_t  has_interval;
    uint16_t last_time;
    uint16_t last_interval;     /* previous interval, for gap/noise tests (the gap itself after a gap) */
    uint16_t tooth_period;      /* last normal (non-gap) tooth interval, for RPM */
    uint16_t period4;           /* sum of the last 4 normal intervals: tooth period x 4,
                                 * keeps the fraction a single 4 us-quantised interval loses */
    uint16_t ring[4];
    uint8_t  ring_i;
    uint32_t rev_acc;
    uint32_t rev_ticks;         /* duration of the last complete revolution */

    /* cam */
    uint8_t  cam_seen;          /* cam edge during the current revolution */
    uint8_t  cam_obs;           /* previous revolution was a valid observation */
    uint8_t  cam_last;          /* cam_seen of that revolution */
    uint8_t  cam_pos;           /* where the last cam edge fell: tooth + 60*rev */
    uint8_t  cam_flag;          /* mark next log entry */

    /* diagnostics */
    uint8_t  loss_reason;
    uint16_t sync_losses;
    uint16_t noise_teeth;
    uint16_t cam_errors;

    /* tooth logger: one-shot capture of the next DEC_LOG_SIZE teeth. Costs
     * the ISR a single flag test when not capturing. */
    uint8_t  log_active;
    uint8_t  log_count;
    tooth_log_t log[DEC_LOG_SIZE];
} decoder_t;

/* Forget position (engine stopped or lost). Keeps config, counters and log. */
DEC_INLINE void dec_reset_position(decoder_t *d)
{
    d->sync = SYNC_NONE;
    d->tooth = 0;
    d->rev = 0;
    d->has_time = 0;
    d->has_interval = 0;
    d->rev_acc = 0;
    d->rev_ticks = 0;
    d->cam_seen = 0;
    d->cam_obs = 0;
    d->cam_flag = 0;
}

DEC_INLINE void dec_init(decoder_t *d, uint8_t cam_enabled)
{
    uint8_t *p = (uint8_t *)d;
    for (uint16_t i = 0; i < sizeof *d; i++) p[i] = 0;
    d->cam_enabled = cam_enabled;
    d->cam_pos = 0xFF;
}

DEC_INLINE void dec_log(decoder_t *d, uint16_t interval, uint8_t flags)
{
    if (!d->log_active) {
        return;
    }
    tooth_log_t *e = &d->log[d->log_count];
    e->interval = interval;
    e->pos = (uint8_t)(d->tooth | (d->rev << 7));
    if (d->cam_flag) {
        flags |= LOG_CAM;
        d->cam_flag = 0;
    }
    e->flags = (uint8_t)(flags | (d->sync << 4));
    if (++d->log_count >= DEC_LOG_SIZE) {
        d->log_active = 0;          /* buffer full: capture complete */
    }
}

/* Start a new capture (discarding the old one). */
DEC_INLINE void dec_log_start(decoder_t *d)
{
    d->log_count = 0;
    d->cam_flag = 0;
    d->log_active = 1;
}

/* The edge that ends the gap = tooth 0 of a new revolution. */
DEC_COLD uint8_t dec_gap(decoder_t *d, uint16_t interval)
{
    uint8_t ev = EV_TOOTH | EV_GAP;
    uint8_t count_ok = (d->tooth == DEC_LAST_TOOTH);

    d->rev_acc += interval;
    /* Compare the next tooth against the gap itself. Using the pre-gap tooth
     * instead can lock up: after a false gap (e.g. a noise pulse accepted as a
     * tooth) every real tooth would look like a gap again. */
    d->last_interval = interval;

    switch (d->sync) {
    case SYNC_NONE:
        d->sync = SYNC_PENDING;
        count_ok = 0;                       /* partial revolution before the first gap */
        break;
    case SYNC_PENDING:
        if (count_ok) {
            d->sync = SYNC_CRANK;
            d->rev_ticks = d->rev_acc;
        }
        break;
    default:                                /* CRANK, FULL */
        if (count_ok) {
            d->rev_ticks = d->rev_acc;
        } else {
            d->sync_losses++;
            d->loss_reason = LOSS_COUNT;
            d->sync = SYNC_PENDING;         /* this gap is the new candidate */
            ev |= EV_SYNC_LOST;
        }
        break;
    }

    /* Cam phase: one observation per correctly counted revolution. */
    uint8_t seen = d->cam_seen;
    d->cam_seen = 0;
    if (count_ok && d->cam_enabled) {
        if (d->sync == SYNC_FULL) {
            uint8_t new_rev = seen ? 0 : 1;
            if (new_rev != (uint8_t)(d->rev ^ 1u)) {
                d->cam_errors++;
                d->sync = SYNC_CRANK;
                d->rev = 0;
                ev |= EV_CAM_ERR;
            } else {
                d->rev = new_rev;
            }
        } else if (d->cam_obs && d->cam_last != seen) {
            d->sync = SYNC_FULL;
            d->rev = seen ? 0 : 1;
            ev |= EV_FULL;
        }
        d->cam_last = seen;
        d->cam_obs = 1;
    } else {
        d->cam_obs = 0;
        if (d->sync != SYNC_FULL) d->rev = 0;
    }

    d->tooth = 0;
    d->rev_acc = 0;
    dec_log(d, interval, LOG_GAP);
    return ev;
}

DEC_INLINE uint8_t dec_crank_edge(decoder_t *d, uint16_t t)
{
    if (!d->has_time) {
        d->has_time = 1;
        d->last_time = t;
        return 0;
    }

    uint16_t interval = (uint16_t)(t - d->last_time);

    /* Reject edges that come impossibly early; they are ignored completely
     * (the next real tooth is measured from the last real one). Always: under
     * 1/4 of the previous interval. Once synced, also under 5/8 of the tooth
     * period: a running engine cannot speed up that much in one tooth (worst
     * cranking ripple is ~11 % per tooth), while before sync the starter's
     * initial acceleration must not be mistaken for noise. */
    uint16_t floor = (uint16_t)(d->last_interval >> 2);
    if (d->sync >= SYNC_CRANK) {
        uint16_t f2 = (uint16_t)((d->tooth_period >> 1) + (d->tooth_period >> 3));
        if (f2 > floor) floor = f2;
    }
    if (d->has_interval && interval < floor) {
        d->noise_teeth++;
        dec_log(d, interval, LOG_NOISE);
        return EV_NOISE;
    }
    d->last_time = t;

    if (!d->has_interval) {
        d->has_interval = 1;
        d->last_interval = interval;
        d->tooth_period = interval;
        uint16_t iv = interval > 16383u ? 16383u : interval;
        d->ring[0] = d->ring[1] = d->ring[2] = d->ring[3] = iv;
        d->period4 = (uint16_t)(iv * 4u);
        return 0;
    }

    /* Gap = 3 tooth periods; anything over 1.5x the previous tooth counts.
     * 16-bit maths (saturating) keeps the ISR short. */
    uint16_t li = d->last_interval;
    uint16_t gap_thr = (uint16_t)(li + (li >> 1));
    if (gap_thr < li) gap_thr = 0xFFFFu;
    if (interval > gap_thr) {
        return dec_gap(d, interval);
    }
    d->last_interval = interval;
    d->tooth_period = interval;
    {
        uint16_t iv = interval > 16383u ? 16383u : interval;   /* sum stays 16-bit */
        d->period4 = (uint16_t)(d->period4 - d->ring[d->ring_i] + iv);
        d->ring[d->ring_i] = iv;
        d->ring_i = (uint8_t)((d->ring_i + 1u) & 3u);
    }

    if (d->sync == SYNC_NONE) {
        dec_log(d, interval, 0);
        return 0;
    }
    if (++d->tooth > DEC_LAST_TOOTH) {
        /* More than 58 teeth without a gap. */
        d->sync_losses++;
        d->loss_reason = LOSS_NO_GAP;
        d->sync = SYNC_NONE;
        d->tooth = 0;
        d->rev = 0;
        d->cam_obs = 0;
        dec_log(d, interval, 0);
        return EV_SYNC_LOST;
    }
    d->rev_acc += interval;
    dec_log(d, interval, 0);
    return EV_TOOTH;
}

DEC_INLINE void dec_cam_edge(decoder_t *d)
{
    d->cam_seen = 1;
    d->cam_flag = 1;
    d->cam_pos = (uint8_t)(d->tooth + ((d->sync == SYNC_FULL && d->rev) ? 60u : 0u));
}

/* Call with the current tick count at least every ~60 ms. Returns 1 if the
 * engine was declared stopped. */
DEC_INLINE uint8_t dec_check_stall(decoder_t *d, uint16_t now)
{
    if (d->has_time && (uint16_t)(now - d->last_time) > DEC_STALL_TICKS) {
        if (d->sync != SYNC_NONE) {
            d->loss_reason = LOSS_STALL;
        }
        dec_reset_position(d);
        return 1;
    }
    return 0;
}

/* RPM from one full revolution of ticks (0 if unknown). */
DEC_INLINE uint16_t dec_rpm(uint32_t rev_ticks)
{
    if (rev_ticks == 0) return 0;
    uint32_t r = (DEC_TICKS_PER_MIN + rev_ticks / 2) / rev_ticks;
    return r > 0xFFFFu ? 0xFFFFu : (uint16_t)r;
}

/* Instantaneous RPM from one tooth interval (6° = 1/60 rev). */
DEC_INLINE uint16_t dec_rpm_tooth(uint16_t interval)
{
    if (interval == 0) return 0;
    uint32_t r = (DEC_TICKS_PER_MIN / 60u + interval / 2u) / interval;
    return r > 0xFFFFu ? 0xFFFFu : (uint16_t)r;
}

#endif
