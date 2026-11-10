/*
 * Phase 9a controls — portable, header-only, integer maths. Called every
 * 10 ms after the fuel/ignition calculation:
 *
 *   overrun fuel cut   closed throttle above dfcoRpm (after dfcoDelay), fuel
 *                      back below dfcoResumeRpm or when the throttle opens
 *   lambda             narrowband: per bank (cyl 1-3 / 4-6) switching PI, only
 *                      near stoichiometric; wideband: PI to the AFR target;
 *                      both: wideband, falling back to narrowband when the
 *                      wideband reading sits at a rail (warming up / faulty)
 *   idle               idle valve: base(CLT) + PI on rpm error + dashpot;
 *                      optional ignition idle control (fast)
 *   VANOS              on/off in an rpm window above a throttle/coolant minimum
 *   heaters            narrowband heaters a set time after the engine starts
 *
 * Every correction is clamped; nothing integrates while cold, cranking,
 * enriching for acceleration, cutting fuel or at the rev limiter.
 */
#ifndef CONTROL_CORE_H
#define CONTROL_CORE_H

#include <stdint.h>
#include "generated/tune_def.h"

#define CTL_INLINE static inline

/* realtime `engine2` bits */
#define CTL_IDLE     0x01
#define CTL_DFCO     0x02
#define CTL_VANOS    0x04
#define CTL_HEATERS  0x08
#define CTL_CLOSED   0x10
#define CTL_WB_BAD   0x20
#define CTL_NB_A_RICH 0x40
#define CTL_NB_B_RICH 0x80

typedef struct {
    uint16_t rpm;
    uint16_t tps_x10;
    int16_t  clt_x10;
    uint16_t air_mg;
    uint16_t afr_target_c;
    uint16_t afr_c;             /* wideband */
    uint16_t nb_a_mv, nb_b_mv;
    uint8_t  running, cranking, accel, revcut;
} ctl_in_t;

typedef struct {
    /* state */
    uint16_t run_ticks;         /* 10 ms ticks since the engine started (saturates) */
    uint16_t dfco_timer;
    int32_t  wb_i;              /* wideband integral, 0.1 % / 1024 (trim = wb_i >> 10) */
    int32_t  nb_a, nb_b;        /* narrowband integrators, 0.1 % / 1024 */
    uint8_t  nb_rich_a, nb_rich_b;
    uint8_t  mode_prev;
    int32_t  idle_i;            /* idle integral, 0.01 % / 128 */
    uint16_t dashpot_x100;      /* remaining dashpot, 0.01 % */
    uint8_t  throttle_was_open;
    uint8_t  idle_div;
    int16_t  idle_p_x100;
    /* cached slow values (recomputed only when their input changes) */
    uint8_t  cache_ok;
    int16_t  clt_cached;
    uint16_t base_x100;         /* idle valve base opening, 0.01 % */
    uint16_t target_cached;
    uint32_t target_recip;      /* 2^26 / AFR target (0.01 AFR) */

    /* outputs */
    uint8_t  flags;             /* CTL_* */
    int16_t  trim_a, trim_b;    /* 0.1 %, applied to cylinders 1-3 / 4-6 */
    uint8_t  iac_duty;          /* % */
    uint16_t idle_target;       /* rpm */
    int16_t  idle_ign_x10;      /* added to the advance */
} ctl_t;

CTL_INLINE void ctl_init(ctl_t *c)
{
    uint8_t *p = (uint8_t *)c;
    for (uint16_t i = 0; i < sizeof *c; i++) p[i] = 0;
}

/* 6-point curve, U08 temperature axis (raw = C + 40), U08 values. */
CTL_INLINE int32_t ctl_curve6(const uint8_t *ax, const uint8_t *val, int16_t t_x10, int32_t mul)
{
    int32_t x = (int32_t)t_x10 + 400;                  /* (C + 40) x 10 */
    if (x <= (int32_t)ax[0] * 10) return (int32_t)val[0] * mul;
    for (uint8_t i = 1; i < 6; i++) {
        int32_t x1 = (int32_t)ax[i] * 10;
        if (x <= x1) {
            int32_t x0 = (int32_t)ax[i - 1] * 10;
            int32_t y0 = (int32_t)val[i - 1] * mul, y1 = (int32_t)val[i] * mul;
            return x1 > x0 ? y0 + (y1 - y0) * (x - x0) / (x1 - x0) : y1;
        }
    }
    return (int32_t)val[5] * mul;
}

CTL_INLINE int32_t ctl_clamp(int32_t v, int32_t lim)
{
    return v > lim ? lim : (v < -lim ? -lim : v);
}

/* x / 10 for |x| < 2^20 without a library division (205 / 2048 = 0.1001) */
CTL_INLINE int32_t ctl_div10(int32_t x)
{
    return x >= 0 ? (x * 205) >> 11 : -((-x * 205) >> 11);
}

/* Narrowband switching PI for one bank; accumulator in 0.1 % / 1024. */
CTL_INLINE void ctl_nb(int32_t *acc, uint8_t *rich_prev, uint16_t mv, int32_t lim)
{
    uint8_t rich = mv > tu_nbThreshold();
    int32_t ramp = ((int32_t)tu_nbRate() * 1311) >> 7;  /* 0.1 %/s per 10 ms = 0.001 % = 10.24 units */
    int32_t step = (int32_t)tu_nbStep() << 10;          /* 0.1 % */
    if (rich != *rich_prev) {
        *acc += rich ? -step : step;                    /* jump back on each switch */
        *rich_prev = rich;
    }
    *acc += rich ? -ramp : ramp;
    *acc = ctl_clamp(*acc, lim);
}

CTL_INLINE void ctl_update(ctl_t *c, const ctl_in_t *in)
{
    uint8_t flags = 0;
    if (!in->running) c->run_ticks = 0;
    else if (c->run_ticks < 0xFFFFu) c->run_ticks++;

    /* ---- overrun fuel cut */
    uint8_t closed = in->tps_x10 < tu_dfcoTps();
    uint8_t warm_dfco = in->clt_x10 >= ((int16_t)tu_dfcoMinClt() - 40) * 10;
    if (tu_dfcoEnable() && in->running && !in->cranking && warm_dfco && closed &&
        in->rpm > (uint16_t)tu_dfcoRpm() * 100u) {
        uint16_t delay = (uint16_t)tu_dfcoDelay() * 10u;           /* 0.1 s -> 10 ms ticks */
        if (c->dfco_timer < delay) c->dfco_timer++;
        else flags |= CTL_DFCO;
    } else if ((c->flags & CTL_DFCO) && tu_dfcoEnable() && in->running && closed &&
               in->rpm >= (uint16_t)tu_dfcoResumeRpm() * 100u) {
        flags |= CTL_DFCO;                                          /* stay cut until the resume speed */
    } else {
        c->dfco_timer = 0;
    }

    /* ---- closed-loop lambda */
    uint8_t mode = tu_lambdaMode();
    if (mode != c->mode_prev) {
        c->wb_i = 0; c->nb_a = 0; c->nb_b = 0;
        c->mode_prev = mode;
    }
    int32_t lim_x10 = (int32_t)tu_lambdaMaxTrim() * 10;             /* % -> 0.1 % */
    int32_t lim = lim_x10 << 10;                                    /* integrator units */
    int16_t a0 = (int16_t)(tu_afrAt0V() * 10), a5 = (int16_t)(tu_afrAt5V() * 10);
    uint8_t wb_bad = (int16_t)in->afr_c <= a0 + 30 || (int16_t)in->afr_c >= a5 - 30;
    if (wb_bad) flags |= CTL_WB_BAD;
    if (in->running) {                                              /* sensor state is meaningless stopped */
        if (in->nb_a_mv > tu_nbThreshold()) flags |= CTL_NB_A_RICH;
        if (in->nb_b_mv > tu_nbThreshold()) flags |= CTL_NB_B_RICH;
    }

    uint8_t cl = mode != 0 && in->running && !in->cranking && !in->accel && !in->revcut &&
                 !(flags & CTL_DFCO) &&
                 in->clt_x10 >= ((int16_t)tu_lambdaMinClt() - 40) * 10 &&
                 c->run_ticks >= (uint16_t)tu_lambdaDelay() * 100u &&
                 in->rpm >= (uint16_t)tu_lambdaMinRpm() * 100u && in->rpm <= (uint16_t)tu_lambdaMaxRpm() * 100u &&
                 in->air_mg <= tu_lambdaMaxLoad();
    uint8_t use_wb = (mode == 2 || mode == 3) && !wb_bad;
    uint8_t use_nb = mode == 1 || (mode == 3 && wb_bad);
    uint8_t near_stoich = in->afr_target_c >= 1440u && in->afr_target_c <= 1500u;
    if (use_nb && !near_stoich) cl = 0;                              /* narrowbands only know lambda 1 */
    if (mode == 2 && wb_bad) cl = 0;

    if (cl && use_wb) {
        /* error in 0.1 %: positive = leaner than the target = more fuel.
         * The division by the target is a cached reciprocal (the target
         * changes far less often than every 10 ms). */
        if (!(c->cache_ok & 2) || in->afr_target_c != c->target_cached) {
            c->target_cached = in->afr_target_c;
            c->target_recip = in->afr_target_c ? (1UL << 26) / in->afr_target_c : 0;
            c->cache_ok |= 2;
        }
        int32_t diff = ctl_clamp((int32_t)in->afr_c - (int32_t)in->afr_target_c, 1000);
        int32_t err = (diff * 1000 >= 0)
            ? (int32_t)(((uint32_t)(diff * 1000) >> 4) * (c->target_recip >> 6) >> 16)
            : -(int32_t)(((uint32_t)(-diff * 1000) >> 4) * (c->target_recip >> 6) >> 16);
        err = ctl_clamp(err, 300);                                  /* +-30 % */
        /* Ki raw x0.1 %/s per %, 10 ms ticks: err * Ki / 1000 in 0.1 %
         * = err * Ki * 1.024 units; Kp raw x0.1 %/% */
        int32_t ki = err * tu_wbKi();
        c->wb_i = ctl_clamp(c->wb_i + ki + ((ki * 3) >> 7), lim);
        int32_t p = ctl_div10(err * tu_wbKp());                     /* 0.1 % */
        int16_t t = (int16_t)ctl_clamp((c->wb_i >> 10) + p, lim_x10);
        c->trim_a = t;
        c->trim_b = t;
    } else if (cl && use_nb) {
        ctl_nb(&c->nb_a, &c->nb_rich_a, in->nb_a_mv, lim);
        ctl_nb(&c->nb_b, &c->nb_rich_b, in->nb_b_mv, lim);
        c->trim_a = (int16_t)(c->nb_a >> 10);
        c->trim_b = (int16_t)(c->nb_b >> 10);
    } else {
        c->trim_a = 0;                                              /* open loop; integrators held */
        c->trim_b = 0;
    }
    if (cl) flags |= CTL_CLOSED;
    if (!in->running) { c->wb_i = 0; c->nb_a = 0; c->nb_b = 0; }

    /* ---- idle */
    if (!(c->cache_ok & 1) || in->clt_x10 != c->clt_cached) {    /* curves only when the temperature moves */
        c->clt_cached = in->clt_x10;
        c->idle_target = (uint16_t)ctl_curve6(tu_idleTargetT_ptr(), tu_idleTarget_ptr(), in->clt_x10, 10);
        c->base_x100 = (uint16_t)ctl_curve6(tu_iacBaseT_ptr(), tu_iacBase_ptr(), in->clt_x10, 100);
        c->cache_ok |= 1;
    }
    int32_t base_x100 = c->base_x100;
    uint8_t throttle_closed = in->tps_x10 < tu_idleTps();
    if (throttle_closed && c->throttle_was_open && in->running) {
        c->dashpot_x100 = (uint16_t)tu_dashpot() * 100u;
    }
    c->throttle_was_open = !throttle_closed;
    uint16_t decay = tu_dashpotDecay();                             /* %/s -> 0.01 % per 10 ms */
    c->dashpot_x100 = c->dashpot_x100 > decay ? (uint16_t)(c->dashpot_x100 - decay) : 0;

    uint8_t idling = in->running && !in->cranking && throttle_closed && in->rpm < c->idle_target + 600u;
    int32_t err = (int32_t)c->idle_target - (int32_t)in->rpm;        /* + = too slow = open the valve */
    if (idling) {
        flags |= CTL_IDLE;
        if (++c->idle_div >= 5) {                                   /* PI at 20 Hz */
            c->idle_div = 0;
            int32_t ilim = ((int32_t)tu_idleMaxTrim() * 100) << 7;
            /* Ki raw x0.01 %/s per rpm at 20 Hz: err * Ki / 20 in 0.01 % = err * Ki * 6.4 units */
            int32_t ki = err * tu_idleKi();
            c->idle_i = ctl_clamp(c->idle_i + ki * 6 + ((ki * 51) >> 7), ilim);
            c->idle_p_x100 = (int16_t)ctl_clamp(err * tu_idleKp(), (int32_t)tu_idleMaxTrim() * 100);
        }
    } else {
        c->idle_p_x100 = 0;
    }
    if (!in->running) c->idle_i = 0;

    int32_t duty_x100;
    if (in->cranking) duty_x100 = (int32_t)tu_iacCrank() * 100;
    else duty_x100 = base_x100 + c->idle_p_x100 + (c->idle_i >> 7) + c->dashpot_x100;
    if (duty_x100 < 0) duty_x100 = 0;
    if (duty_x100 > 10000) duty_x100 = 10000;
    c->iac_duty = tu_iacEnable() ? (uint8_t)(((uint16_t)duty_x100 + 50u) / 100u) : 0;

    c->idle_ign_x10 = 0;
    if (idling && tu_idleIgnEnable()) {
        int32_t ign = ctl_div10(ctl_clamp(err, 2000) * tu_idleIgnGain()); /* 0.01 deg/rpm -> 0.1 deg */
        c->idle_ign_x10 = (int16_t)ctl_clamp(ign, tu_idleIgnMax());
    }

    /* ---- VANOS: on in an rpm window, with hysteresis */
    uint16_t on = (uint16_t)tu_vanosRpmOn() * 100u, off = (uint16_t)tu_vanosRpmOff() * 100u;
    uint16_t hy = (uint16_t)tu_vanosHyst() * 10u;
    uint16_t min_tps = (uint16_t)tu_vanosMinTps() * 10u;
    uint8_t vok = tu_vanosEnable() && in->running && !in->cranking &&
                  in->clt_x10 >= ((int16_t)tu_vanosMinClt() - 40) * 10;
    uint8_t vanos;
    if (c->flags & CTL_VANOS) {
        vanos = vok && in->rpm + hy >= on && in->rpm <= off + hy && in->tps_x10 + 20u >= min_tps;
    } else {
        vanos = vok && in->rpm >= on && in->rpm <= off && in->tps_x10 >= min_tps;
    }
    if (vanos) flags |= CTL_VANOS;

    /* ---- narrowband heaters */
    if (in->running && c->run_ticks >= (uint16_t)tu_nbHeatDelay() * 100u) flags |= CTL_HEATERS;

    c->flags = flags;
}

#endif
