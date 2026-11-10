/*
 * Sensor conversion and fuel / ignition calculation — core logic (portable,
 * header-only, integer fixed point; float costs ~150 us per operation on AVR).
 *
 * Called at 100 Hz from the main loop with raw ADC values and the decoder
 * state; produces per-cylinder pulse widths and advance for the scheduler.
 *
 * Units used throughout (suffix):
 *   _x10   tenths      (tps_x10: 0.1 %, clt_x10: 0.1 C, adv: 0.1 deg)
 *   _c     hundredths  (vbat_c: 0.01 V, afr_c: 0.01 AFR, maf_c: 0.01 g/s, fuel_c: 0.01 mg)
 *   air_d  air mass per cylinder per intake stroke, 0.1 mg
 *
 * Raw tune encodings are documented in generated/tune_def.h.
 *
 * Cost matters: a 32-bit division is ~40 us on AVR, so the hot paths use
 * multiply-shift, 16-bit divisions, and skip work that does nothing (trims at
 * 0 %, Alpha-N maths in MAF mode). Budget: well under 1 ms per update.
 */
#ifndef ENGINE_CORE_H
#define ENGINE_CORE_H

#include <stdint.h>
#include "generated/tune_def.h"

#define ENG_INLINE static inline

/* engine state bits (realtime `engine`) */
#define ENG_RUNNING    0x01
#define ENG_CRANKING   0x02
#define ENG_REVLIMIT   0x04
#define ENG_AE         0x08
#define ENG_AFTERSTART 0x10
#define ENG_TESTMODE   0x20
#define ENG_BURNING    0x40
#define ENG_PUMP       0x80

/* fault bits (realtime `faults`) */
#define FLT_TPS1       0x01
#define FLT_TPS2       0x02
#define FLT_TPS_DIFF   0x04
#define FLT_CLT        0x08
#define FLT_IAT        0x10
#define FLT_MAF        0x20
#define FLT_VBAT       0x40
#define FLT_DEFAULTS   0x80

#define ENG_CLT_FALLBACK_X10   600     /* 60 C if the coolant sensor fails */
#define ENG_IAT_FALLBACK_X10   300     /* 30 C */
#define ENG_ADC_OPEN           1013    /* above: sensor open circuit */
#define ENG_ADC_SHORT          10      /* below: sensor shorted */

/* ------------------------------------------------------------ lookups -- */

/* Find x on an ascending axis: index i (0..n-2) and fraction f (0..256)
 * between ax[i] and ax[i+1]. Clamps outside the axis. */
ENG_INLINE void eng_axis(const uint16_t *ax, uint8_t n, uint16_t x, uint8_t *i, uint16_t *f)
{
    if (x <= ax[0]) { *i = 0; *f = 0; return; }
    if (x >= ax[n - 1]) { *i = (uint8_t)(n - 2); *f = 256; return; }
    uint8_t k = 1;
    while (x > ax[k]) k++;
    *i = (uint8_t)(k - 1);
    uint16_t span = (uint16_t)(ax[k] - ax[k - 1]);
    uint16_t d = (uint16_t)(x - ax[k - 1]);
    while (span > 255u) {           /* keep (d << 8) in 16 bits: 16-bit divide */
        span >>= 1;
        d >>= 1;
    }
    *f = span ? (uint16_t)((uint16_t)(d << 8) / span) : 0;
}

/* Linear interpolation between a and b, f in 0..256. */
ENG_INLINE int32_t eng_lerp(int32_t a, int32_t b, uint16_t f)
{
    return a + (((b - a) * (int32_t)f) >> 8);
}

/* Bilinear lookup in a U08 table z[y][x]; result = raw value x 256. */
ENG_INLINE uint16_t eng_table8(const uint8_t *z, uint8_t nx, uint8_t ix, uint16_t fx, uint8_t iy, uint16_t fy)
{
    const uint8_t *r0 = z + (uint16_t)iy * nx;
    const uint8_t *r1 = r0 + nx;
    int32_t a = eng_lerp((int32_t)r0[ix] << 8, (int32_t)r0[ix + 1] << 8, fx);
    int32_t b = eng_lerp((int32_t)r1[ix] << 8, (int32_t)r1[ix + 1] << 8, fx);
    return (uint16_t)eng_lerp(a, b, fy);
}

/* Axis helpers: convert a tune axis to a common uint16 scale. */
ENG_INLINE void eng_axis_u8(const uint8_t *raw, uint8_t n, uint16_t mul, uint16_t *out)
{
    for (uint8_t k = 0; k < n; k++) out[k] = (uint16_t)(raw[k] * mul);
}

ENG_INLINE void eng_axis_u16(const uint8_t *raw, uint8_t n, uint16_t *out)
{
    for (uint8_t k = 0; k < n; k++) out[k] = tune_rd16(raw + 2u * k);
}

/* Curve with a U08 axis (x in axis raw units x mul) and U16 values. */
ENG_INLINE uint16_t eng_curve_u8_u16(const uint8_t *xr, const uint8_t *yr, uint8_t n, uint16_t mul, uint16_t x)
{
    uint16_t ax[16];
    eng_axis_u8(xr, n, mul, ax);
    uint8_t i; uint16_t f;
    eng_axis(ax, n, x, &i, &f);
    return (uint16_t)eng_lerp(tune_rd16(yr + 2u * i), tune_rd16(yr + 2u * (i + 1u)), f);
}

/* Curve with a U08 axis and U08 values; result x 256. */
ENG_INLINE uint16_t eng_curve_u8_u8(const uint8_t *xr, const uint8_t *yr, uint8_t n, uint16_t mul, uint16_t x)
{
    uint16_t ax[16];
    eng_axis_u8(xr, n, mul, ax);
    uint8_t i; uint16_t f;
    eng_axis(ax, n, x, &i, &f);
    return (uint16_t)eng_lerp((int32_t)yr[i] << 8, (int32_t)yr[i + 1] << 8, f);
}

/* ------------------------------------------------------------ sensors -- */

typedef struct {
    uint16_t adc[10];           /* raw, index = enum adc_channel (board.h) */
} eng_adc_t;

typedef struct {
    uint16_t tps_x10;           /* 0..1000 */
    int16_t  clt_x10;
    int16_t  iat_x10;
    uint16_t vbat_c;
    uint16_t maf_mv;
    uint16_t maf_c;             /* 0.01 g/s */
    uint16_t afr_c;
    uint16_t nbo2a_mv, nbo2b_mv;
    uint8_t  faults;
} eng_sensors_t;

/* ADC counts -> mV (x 5000 / 1023, as multiply-shift: 20020 / 4096 = 4.8877). */
ENG_INLINE uint16_t eng_adc_mv(uint16_t adc)
{
    return (uint16_t)(((uint32_t)adc * 20020u + 2048u) >> 12);
}

/* TPS 0.1 % from one sensor; returns 0xFFFF if the reading is impossible. */
ENG_INLINE uint16_t eng_tps_one(uint16_t adc, uint16_t lo, uint16_t hi)
{
    if (adc < ENG_ADC_SHORT || adc > ENG_ADC_OPEN || hi <= lo) return 0xFFFFu;
    if (adc <= lo) return 0;
    if (adc >= hi) return 1000;
    return (uint16_t)(((uint32_t)(adc - lo) * 1000u) / (uint16_t)(hi - lo));
}

/* Thermistor (shared CLT/IAT curve): 0.1 C, or INT16_MIN if out of range. */
ENG_INLINE int16_t eng_therm(uint16_t adc)
{
    if (adc < ENG_ADC_SHORT || adc > ENG_ADC_OPEN) return INT16_MIN;
    uint16_t ax[TU_THERM_ADC_N];
    eng_axis_u16(tu_thermAdc_ptr(), TU_THERM_ADC_N, ax);
    uint8_t i; uint16_t f;
    eng_axis(ax, TU_THERM_ADC_N, adc, &i, &f);
    int32_t t0 = ((int32_t)tu_thermC(i) - 40) * 10;
    int32_t t1 = ((int32_t)tu_thermC(i + 1u) - 40) * 10;
    return (int16_t)eng_lerp(t0, t1, f);
}

#define FLT_FAST (FLT_TPS1 | FLT_TPS2 | FLT_TPS_DIFF | FLT_MAF)

/* Every 10 ms: throttle and MAF. */
ENG_INLINE void eng_sensors_fast(const eng_adc_t *a, eng_sensors_t *s)
{
    s->faults &= (uint8_t)~FLT_FAST;

    /* throttle: one or two sensors (two banks of ITBs) */
    uint16_t t1 = eng_tps_one(a->adc[1], tu_tps1Min(), tu_tps1Max());
    uint16_t t2 = tu_tpsCount() ? eng_tps_one(a->adc[2], tu_tps2Min(), tu_tps2Max()) : t1;
    if (t1 == 0xFFFFu) s->faults |= FLT_TPS1;
    if (tu_tpsCount() && t2 == 0xFFFFu) s->faults |= FLT_TPS2;
    if (t1 != 0xFFFFu && t2 != 0xFFFFu) {
        uint16_t d = t1 > t2 ? (uint16_t)(t1 - t2) : (uint16_t)(t2 - t1);
        if (d > 100u) {
            s->faults |= FLT_TPS_DIFF;
            s->tps_x10 = t1 < t2 ? t1 : t2;          /* disagree: trust the lower one */
        } else {
            s->tps_x10 = (uint16_t)((t1 + t2) / 2u);
        }
    } else if (t1 != 0xFFFFu) {
        s->tps_x10 = t1;
    } else if (t2 != 0xFFFFu) {
        s->tps_x10 = t2;
    } else {
        s->tps_x10 = 0;                                 /* both failed: closed throttle */
    }

    /* MAF: volts -> g/s on the transfer curve (axis raw x 20 mV) */
    s->maf_mv = eng_adc_mv(a->adc[0]);
    s->maf_c = eng_curve_u8_u16(tu_mafV_ptr(), tu_mafFlow_ptr(), TU_MAF_V_N, 20, s->maf_mv);
    if (a->adc[0] > ENG_ADC_OPEN) s->faults |= FLT_MAF;
}

/* Every 100 ms: slow-changing sensors. */
ENG_INLINE void eng_sensors_slow(const eng_adc_t *a, eng_sensors_t *s)
{
    s->faults &= (uint8_t)FLT_FAST;

    int16_t c = eng_therm(a->adc[3]);
    if (c == INT16_MIN) { s->faults |= FLT_CLT; c = ENG_CLT_FALLBACK_X10; }
    s->clt_x10 = c;
    int16_t t = eng_therm(a->adc[4]);
    if (t == INT16_MIN) { s->faults |= FLT_IAT; t = ENG_IAT_FALLBACK_X10; }
    s->iat_x10 = t;

    /* battery: ADC volts x divider ratio (raw x0.01); 0.01 V =
     * adc x scale x 5000 / 1023 / 1000 = adc x scale x 320.3 / 65536 */
    s->vbat_c = (uint16_t)(((uint32_t)a->adc[8] * tu_vbatScale() * 320u + 32768u) >> 16);
    if (s->vbat_c < 600u || s->vbat_c > 2000u) s->faults |= FLT_VBAT;

    /* wideband: linear between the controller's 0 V and 5 V AFR */
    uint16_t wmv = eng_adc_mv(a->adc[5]);
    int32_t a0 = (int32_t)tu_afrAt0V() * 10, a5 = (int32_t)tu_afrAt5V() * 10;
    s->afr_c = (uint16_t)(a0 + (a5 - a0) * (int32_t)wmv / 5000);

    s->nbo2a_mv = eng_adc_mv(a->adc[6]);
    s->nbo2b_mv = eng_adc_mv(a->adc[7]);
}

ENG_INLINE void eng_sensors(const eng_adc_t *a, eng_sensors_t *s)
{
    s->faults = 0;
    eng_sensors_fast(a, s);
    eng_sensors_slow(a, s);
}

/* Temperature (0.1 C) -> raw axis of the temperature curves (C + 40),
 * clamped at 0 (below -40 C). */
ENG_INLINE uint16_t eng_temp_axis(int16_t t_x10)
{
    int16_t v = (int16_t)((t_x10 + 405) / 10);
    return v < 0 ? 0 : (uint16_t)v;
}

/* --------------------------------------------------------- fuel / ign -- */

typedef struct {
    /* persistent state */
    uint8_t  primed;             /* tps_prev holds a real reading */
    uint16_t tps_prev;
    uint8_t  ae_peak;            /* % */
    uint16_t ae_left;            /* 10 ms ticks */
    uint16_t as_left;            /* 10 ms ticks */
    uint8_t  was_cranking;
    uint8_t  revcut;
    uint16_t rpm;                /* last rpm, for the slow step */

    /* cached by eng_calc_slow() (tune + slow sensors, every 100 ms) */
    uint32_t k_flow;             /* 1e7 / injector flow (ug/ms) */
    uint32_t an_k;               /* Alpha-N: cylinder cc x air density, x 65536 / 100000 */
    int16_t  clt_adv_x10;
    uint16_t crank_pw_us;

    /* results */
    uint8_t  state;              /* ENG_* */
    int16_t  tps_dot;            /* %/s */
    uint16_t air_d;              /* 0.1 mg per cylinder */
    uint16_t ve_x10;
    uint8_t  blend;              /* % MAF */
    uint16_t afr_target_c;
    uint8_t  warmup;             /* % */
    uint8_t  ae;                 /* % */
    uint8_t  as;                 /* % */
    uint16_t pw_base_us;         /* before cylinder trim and dead time */
    uint16_t dead_us;
    uint8_t  duty;
    int16_t  adv_x10;            /* before cylinder trim */
    uint16_t dwell_us;
    uint16_t eoi_x10;
    uint16_t pw_us[6];           /* to the scheduler: per cycle, incl. dead time */
    int16_t  advance[6];
} eng_calc_t;

ENG_INLINE void eng_calc_init(eng_calc_t *e)
{
    uint8_t *p = (uint8_t *)e;
    for (uint16_t i = 0; i < sizeof *e; i++) p[i] = 0;
}

/* rpm/tps/load axes of the main tables, in common units:
 * rpm (rpm), tps (0.5 %, the raw axis), load (mg) */
typedef struct {
    uint8_t  ir, il, it;
    uint16_t fr, fl, ft;
} eng_pos_t;

/* Every 100 ms (and before the first eng_calc): values that depend only on
 * the tune and slow sensors. Holds the 32-bit divisions that would otherwise
 * run 100 times a second. */
ENG_INLINE void eng_calc_slow(eng_calc_t *e, const eng_sensors_t *s)
{
    uint32_t flow = ((uint32_t)tu_injFlow() * tu_fuelDensity()) / 60u;                 /* ug per ms */
    if (flow < 100u) flow = 100u;
    e->k_flow = 10000000UL / flow;

    uint16_t t_k = (uint16_t)((s->iat_x10 + 2732) / 10);
    uint32_t rho_ug = (3484UL * tu_baro()) / t_k;                                       /* air density, ug/cc */
    uint32_t x = (uint32_t)(tu_displacement() / 6u) * rho_ug;                           /* ug of air per cylinder */
    e->an_k = ((x << 8) / 100000UL) << 8;

    uint16_t ta = eng_temp_axis(s->clt_x10);
    e->warmup = (uint8_t)(eng_curve_u8_u8(tu_warmupT_ptr(), tu_warmup_ptr(), TU_WARMUP_T_N, 1, ta) >> 8);
    e->crank_pw_us = eng_curve_u8_u16(tu_crankPwT_ptr(), tu_crankPw_ptr(), TU_CRANK_PW_T_N, 1, ta);
    uint16_t ct = eng_curve_u8_u8(tu_cltAdvT_ptr(), tu_cltAdv_ptr(), TU_CLT_ADV_T_N, 1, ta);
    e->clt_adv_x10 = (int16_t)((((int32_t)ct * 5) >> 8) - 100);                          /* (raw - 20) x 0.5 */

    uint16_t vb = (uint16_t)(s->vbat_c / 10u);                                           /* axis raw: 0.1 V */
    e->dead_us = eng_curve_u8_u16(tu_injDeadV_ptr(), tu_injDead_ptr(), TU_INJ_DEAD_V_N, 1, vb);
    uint16_t dw = eng_curve_u8_u16(tu_dwellV_ptr(), tu_dwell_ptr(), TU_DWELL_V_N, 1, vb);
    e->dwell_us = dw < 500u ? 500u : (dw > 8000u ? 8000u : dw);

    uint32_t duty = e->rpm ? ((uint32_t)e->pw_us[0] * e->rpm) / 1200000UL : 0;
    e->duty = (uint8_t)(duty > 100u ? 100u : duty);
}

/*
 * One calculation step (every 10 ms). Call eng_calc_slow() first, and then
 * every 100 ms.
 *   rpm      averaged engine speed (0 = not turning / unknown)
 *   synced   decoder has at least crank sync
 */
ENG_INLINE void eng_calc(eng_calc_t *e, const eng_sensors_t *s, uint16_t rpm, uint8_t synced)
{
    uint16_t ax[16];
    eng_pos_t p;

    /* ---- engine state */
    uint16_t crank_rpm = (uint16_t)tu_crankRpm() * 10u;
    uint8_t running = synced && rpm > 0;
    uint8_t cranking = running && rpm < crank_rpm;
    e->state = (uint8_t)((running ? ENG_RUNNING : 0) | (cranking ? ENG_CRANKING : 0));
    e->rpm = rpm;

    /* ---- acceleration enrichment from throttle speed (the first reading only
     * seeds tps_prev: a jump from "0" at power-up is not a throttle snap) */
    if (!e->primed) {
        e->tps_prev = s->tps_x10;
        e->primed = 1;
    }
    e->tps_dot = (int16_t)(((int16_t)s->tps_x10 - (int16_t)e->tps_prev) * 10);   /* 0.1 % per 10 ms -> %/s */
    e->tps_prev = s->tps_x10;
    if (running && !cranking && e->tps_dot > (int16_t)(tu_aeRate(0) * 10u)) {
        uint8_t add = (uint8_t)(eng_curve_u8_u8(tu_aeRate_ptr(), tu_aeAdd_ptr(), TU_AE_RATE_N, 10,
                                                (uint16_t)e->tps_dot) >> 8);
        if (add >= e->ae || !e->ae_left) {
            e->ae_peak = add;
            e->ae_left = tu_aeTime();                  /* raw x10 ms = 10 ms ticks */
        }
    }
    if (e->ae_left) {
        e->ae = (uint8_t)(((uint16_t)e->ae_peak * e->ae_left) / (tu_aeTime() ? tu_aeTime() : 1u));
        e->ae_left--;
        e->state |= ENG_AE;
    } else {
        e->ae = 0;
    }

    /* ---- after-start enrichment */
    if (e->was_cranking && running && !cranking) {
        e->as_left = (uint16_t)tu_afterStartTime() * 100u;
    }
    e->was_cranking = cranking;
    if (!running) e->as_left = 0;
    if (e->as_left) {
        uint16_t total = (uint16_t)tu_afterStartTime() * 100u;
        e->as = (uint8_t)(((uint32_t)tu_afterStart() * e->as_left) / (total ? total : 1u));
        e->as_left--;
        e->state |= ENG_AFTERSTART;
    } else {
        e->as = 0;
    }

    /* ---- table positions */
    eng_axis_u8(tu_rpmBins_ptr(), TU_RPM_BINS_N, 100, ax);
    eng_axis(ax, TU_RPM_BINS_N, rpm, &p.ir, &p.fr);
    eng_axis_u8(tu_tpsBins_ptr(), TU_TPS_BINS_N, 1, ax);
    eng_axis(ax, TU_TPS_BINS_N, (uint16_t)(s->tps_x10 / 5u), &p.it, &p.ft);   /* axis raw: 0.5 % */

    /* ---- air mass per cylinder */
    uint16_t maf_air = 0, an_air = 0;
    uint8_t src = tu_loadSource();
    if (src != 1 && rpm > 0) {
        /* mg = g/s x 1000 x 120 / (rpm x 6); 0.1 mg = maf_c x 2000 / rpm */
        uint32_t m = ((uint32_t)s->maf_c * 2000u) / rpm;
        uint16_t trim = eng_table8(tu_mafTrim_ptr(), TU_MAF_TRIM_X, p.ir, p.fr, p.it, p.ft);   /* raw x256, 100 = 0 % */
        if (trim != 100u * 256u) m = (((m * trim) >> 8) * 655u) >> 16;                    /* x trim / 100 */
        maf_air = (uint16_t)(m > 6500u ? 6500u : m);
    }
    {
        uint16_t ve_x256 = eng_table8(tu_veTable_ptr(), TU_VE_TABLE_X, p.ir, p.fr, p.it, p.ft);
        e->ve_x10 = (uint16_t)(((uint32_t)ve_x256 * 5u) >> 8);                         /* raw 0.5 % -> 0.1 % */
    }
    if (src != 0) {
        uint32_t a = ((uint32_t)e->ve_x10 * e->an_k) >> 16;                             /* 0.1 mg */
        an_air = (uint16_t)(a > 6500u ? 6500u : a);
    }
    if (src == 0) {
        e->blend = 100;
        e->air_d = maf_air;
    } else if (src == 1) {
        e->blend = 0;
        e->air_d = an_air;
    } else {
        e->blend = (uint8_t)(eng_table8(tu_blendTable_ptr(), TU_BLEND_TABLE_X, p.ir, p.fr, p.it, p.ft) >> 8);
        if (e->blend > 100) e->blend = 100;
        uint32_t mix = (uint32_t)maf_air * e->blend + (uint32_t)an_air * (100u - e->blend);
        e->air_d = (uint16_t)((mix * 655u + 32768u) >> 16);                             /* / 100 */
    }

    eng_axis_u16(tu_loadBins_ptr(), TU_LOAD_BINS_N, ax);
    eng_axis(ax, TU_LOAD_BINS_N, (uint16_t)(e->air_d / 10u), &p.il, &p.fl);

    /* ---- fuel */
    e->afr_target_c = (uint16_t)(((uint32_t)eng_table8(tu_afrTable_ptr(), TU_AFR_TABLE_X, p.ir, p.fr, p.il, p.fl) * 10u) >> 8);
    if (e->afr_target_c < 600u) e->afr_target_c = 600u;
    /* pulse (us) = air (mg) / AFR / flow (mg/ms) x 1000 = air_d x (1e7 / flow_ug) / afr_c */
    uint32_t pw = ((uint32_t)e->air_d * e->k_flow) / e->afr_target_c;
    if (pw > 65535u) pw = 65535u;
    uint32_t w = (uint32_t)e->warmup * (100u + e->ae + e->as);                          /* % x %, 10000 = 1.0 */
    uint32_t f_q10 = (w * 6711u) >> 16;                                                 /* x 1024 / 10000 */
    if (f_q10 != 1024u) pw = (pw * f_q10) >> 10;

    if (cranking) {
        pw = e->crank_pw_us;
    }

    /* rev limiter: fuel cut with hysteresis */
    uint16_t lim = (uint16_t)tu_revLimit() * 100u;
    uint16_t hyst = (uint16_t)tu_revHyst() * 10u;
    if (rpm > lim) e->revcut = 1;
    else if (rpm + hyst < lim) e->revcut = 0;
    if (e->revcut) e->state |= ENG_REVLIMIT;

    if (!running || e->revcut || pw == 0) pw = 0;
    if (pw > 60000u) pw = 60000u;
    e->pw_base_us = (uint16_t)pw;

    for (uint8_t c = 0; c < 6; c++) {
        uint8_t trim = tu_cylFuelTrim(c);                                               /* raw 100 = 0 % */
        uint32_t pc = trim == 100u ? pw : (pw * trim) / 100u;
        e->pw_us[c] = pc ? (uint16_t)(pc + e->dead_us > 60000u ? 60000u : pc + e->dead_us) : 0;
    }

    /* ---- ignition */
    int16_t adv;
    if (cranking) {
        adv = (int16_t)(((int16_t)tu_crankAdvance() - 40) * 5);
    } else {
        uint16_t z = eng_table8(tu_ignTable_ptr(), TU_IGN_TABLE_X, p.ir, p.fr, p.il, p.fl);
        adv = (int16_t)((((int32_t)z * 5) >> 8) - 200);                                  /* (raw - 40) x 0.5 deg */
        adv = (int16_t)(adv + e->clt_adv_x10);
    }
    e->adv_x10 = adv;
    for (uint8_t c = 0; c < 6; c++) {
        int16_t a = (int16_t)(adv + ((int16_t)tu_cylIgnTrim(c) - 20) * 5);
        if (a < -200) a = -200;
        if (a > 600) a = 600;
        e->advance[c] = a;
    }

    /* ---- end of injection (8x8 table, axes rpm and load) */
    {
        uint16_t ax2[8];
        uint8_t ir, il; uint16_t fr, fl;
        eng_axis_u8(tu_eoiRpm_ptr(), TU_EOI_RPM_N, 100, ax2);
        eng_axis(ax2, TU_EOI_RPM_N, rpm, &ir, &fr);
        eng_axis_u16(tu_eoiLoad_ptr(), TU_EOI_LOAD_N, ax2);
        eng_axis(ax2, TU_EOI_LOAD_N, (uint16_t)(e->air_d / 10u), &il, &fl);
        uint32_t eoi = ((uint32_t)eng_table8(tu_eoiTable_ptr(), TU_EOI_TABLE_X, ir, fr, il, fl) * 30u) >> 8;
        e->eoi_x10 = (uint16_t)(eoi > 7199u ? 7199u : eoi);
    }
}

#endif
