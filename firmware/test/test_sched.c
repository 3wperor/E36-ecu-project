/*
 * Host tests for the output scheduler, end to end:
 *   simulated engine (tools/stim pattern) -> real decoder core -> real
 *   scheduler core -> model of the AVR output-compare hardware
 * The model switches pins at exact timer ticks (as the hardware does) and
 * runs the OC "ISR" with a realistic latency. Every pin edge is converted
 * back to a true crank angle and checked.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stdint.h>

/* ---- output-compare hardware model (hooks for sched_core.h) ------------- */

#define ISR_LAT 4                   /* ticks (16 us) from event to ISR code */

static uint64_t g_tick;             /* absolute time, 4 us ticks */

typedef struct { uint8_t set_mode, ie, pin; uint16_t ocr; } hwch_t;
static hwch_t hw[12];

static inline uint16_t sched_hw_now(void) { return (uint16_t)(g_tick + ISR_LAT); }
static inline void sched_hw_arm(uint8_t n, uint16_t when) { hw[n].set_mode = 1; hw[n].ocr = when; hw[n].ie = 1; }
static inline void sched_hw_end(uint8_t n, uint16_t when) { hw[n].set_mode = 0; hw[n].ocr = when; }
static inline void sched_hw_idle(uint8_t n) { hw[n].set_mode = 0; hw[n].ie = 0; }

#include "decoder/decoder_core.h"
#include "sched/sched_core.h"
#include "pattern.h"

static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("  FAIL line %d: %s\n", __LINE__, #c); } } while (0)

/* ---- simulation state ------------------------------------------------------ */

#define MAX_PULSES 6000
#define MAX_STEPS  600000

typedef struct { uint64_t on, off; } pulse_t;

static decoder_t D;
static sched_t S;
static sched_cfg_t cfg;

static pulse_t pulses[12][MAX_PULSES];
static int npulse[12];
static uint64_t on_since[12];

/* engine */
static double eng_rpm;              /* current speed (0 = stopped) */
static double eng_next;             /* tick time of the next half-step */
static uint64_t eng_steps;
static uint8_t eng_step, eng_out, eng_cam_mode = CAM_HALFMOON;
static int eng_drop;
static double step_t[MAX_STEPS];    /* time of half-step k: angle 3k degrees */
static double ramp_from, ramp_to, ramp_t0, ramp_len;

static void sim_reset(void)
{
    g_tick = 1000;
    memset(hw, 0, sizeof hw);
    memset(npulse, 0, sizeof npulse);
    dec_init(&D, 1);
    sched_init(&S);
    eng_rpm = 0;
    eng_next = 1e300;
    eng_steps = 0;
    eng_step = 0;
    eng_cam_mode = CAM_HALFMOON;
    eng_out = stim_pattern(0, eng_cam_mode, 41);
    eng_drop = 0;
    ramp_len = 0;
    step_t[0] = (double)g_tick;
}

static void engine_set(double rpm)
{
    if (rpm > 0 && eng_rpm <= 0) {
        eng_next = (double)g_tick + 10;
        step_t[eng_steps] = (double)g_tick;
    }
    if (rpm <= 0) eng_next = 1e300;
    eng_rpm = rpm;
    ramp_len = 0;
}

static void engine_ramp(double to, double seconds)
{
    ramp_from = eng_rpm;
    ramp_to = to;
    ramp_t0 = (double)g_tick;
    ramp_len = seconds / 4e-6;
}

static void engine_step(void)
{
    double t = eng_next;
    eng_steps++;
    if (eng_steps < MAX_STEPS) step_t[eng_steps] = t;
    eng_step = (uint8_t)((eng_step + 1) % STIM_STEPS);
    uint8_t n = stim_pattern(eng_step, eng_cam_mode, 41);
    uint16_t icr = (uint16_t)(uint64_t)floor(t);

    if ((n & STIM_OUT_CRANK) && !(eng_out & STIM_OUT_CRANK)) {
        if (eng_drop) {
            eng_drop = 0;
        } else {
            uint8_t ev = dec_crank_edge(&D, icr);
            sched_tooth(&S, &D, ev, icr);
            if (ev & EV_SYNC_LOST) sched_cancel_pending(&S);
        }
    }
    if ((n & STIM_OUT_CAM) && !(eng_out & STIM_OUT_CAM)) dec_cam_edge(&D);
    eng_out = n;

    if (ramp_len > 0) {
        double x = (t - ramp_t0) / ramp_len;
        if (x >= 1) { x = 1; ramp_len = 0; }
        eng_rpm = ramp_from + (ramp_to - ramp_from) * x;
    }
    eng_next = t + 125000.0 / eng_rpm;      /* 3° in 4 us ticks */
}

static void main_task(void)
{
    if (dec_check_stall(&D, (uint16_t)g_tick)) sched_cancel_pending(&S);
    sched_slot_t sl[SCHED_CHANNELS];
    sched_input_t in;
    sched_input_from_cfg(&cfg, &in);
    sched_compute(&in, D.sync, D.has_interval ? D.period4 : 0, D.rev_ticks, sl);
    for (uint8_t n = 0; n < SCHED_CHANNELS; n++) {
        sched_post(&S, n, &sl[n], D.sync >= SYNC_CRANK);
    }
}

static void run_ticks(uint64_t n)
{
    uint64_t end = g_tick + n;
    while (g_tick < end) {
        while (eng_next <= (double)g_tick) engine_step();

        for (uint8_t c = 0; c < SCHED_CHANNELS; c++) {
            if ((uint16_t)g_tick == hw[c].ocr) {
                uint8_t lvl = hw[c].set_mode;
                if (lvl && !hw[c].pin) on_since[c] = g_tick;
                if (!lvl && hw[c].pin && npulse[c] < MAX_PULSES) {
                    pulses[c][npulse[c]].on = on_since[c];
                    pulses[c][npulse[c]].off = g_tick;
                    npulse[c]++;
                }
                hw[c].pin = lvl;
                if (hw[c].ie) sched_oc(&S, c, hw[c].ocr);
            }
        }
        if (g_tick % 2500 == 0) main_task();        /* 100 Hz */
        g_tick++;
    }
}

static void run_s(double s) { run_ticks((uint64_t)(s / 4e-6)); }

/* true crank angle (degrees, cumulative) at tick time t */
static double angle_at(double t)
{
    uint64_t lo = 0, hi = eng_steps < MAX_STEPS ? eng_steps : MAX_STEPS - 1;
    if (t <= step_t[0]) return 0;
    if (t >= step_t[hi]) return 3.0 * (double)hi;
    while (hi - lo > 1) {
        uint64_t mid = (lo + hi) / 2;
        if (step_t[mid] <= t) lo = mid; else hi = mid;
    }
    return 3.0 * ((double)lo + (t - step_t[lo]) / (step_t[lo + 1] - step_t[lo]));
}

static double wrap(double a, double period)
{
    a = fmod(a, period);
    if (a < -period / 2) a += period;
    if (a >= period / 2) a -= period;
    return a;
}

/* ---- analysis -------------------------------------------------------------- */

typedef struct {
    int    sparks, injections;
    double spark_err, inj_err;          /* worst angle error, degrees */
    double dwell_err_us, pw_err_us;     /* worst duration error */
    int    bad_count;                   /* channels not firing exactly once per period */
    int    order_ok;
    double max_on_us;                   /* longest any output was on */
} result_t;

/* Expected event angles are in decoder coordinates; decoder angle 0 = true
 * angle 360 (cam at half-step 41, see test_decoder.c). */
static result_t analyse(uint64_t from, uint64_t to, double period_deg,
                        double dwell_us, double pw_us)
{
    result_t r;
    memset(&r, 0, sizeof r);
    r.order_ok = 1;
    double first_spark[6];
    for (int c = 0; c < 6; c++) {
        first_spark[c] = -1;
        double tdc = 360.0 + (cfg.tdc_offset + sched_tdc_rel[c]) / 10.0;

        int count = 0;
        for (int i = 0; i < npulse[SCHED_IGN(c)]; i++) {
            pulse_t p = pulses[SCHED_IGN(c)][i];
            double on_us = (double)(p.off - p.on) * 4.0;
            if (on_us > r.max_on_us) r.max_on_us = on_us;
            if (p.off < from || p.off > to) continue;
            count++;
            double a = angle_at((double)p.off);
            double e = fabs(wrap(a - (tdc - cfg.advance / 10.0), period_deg));
            if (e > r.spark_err) r.spark_err = e;
            double de = fabs(on_us - dwell_us);
            if (de > r.dwell_err_us) r.dwell_err_us = de;
            if (first_spark[c] < 0) first_spark[c] = (double)p.off;
        }
        r.sparks += count;

        int icount = 0;
        for (int i = 0; i < npulse[SCHED_INJ(c)]; i++) {
            pulse_t p = pulses[SCHED_INJ(c)][i];
            double on_us = (double)(p.off - p.on) * 4.0;
            if (on_us > r.max_on_us) r.max_on_us = on_us;
            if (p.off < from || p.off > to) continue;
            icount++;
            double a = angle_at((double)p.off);
            double e = fabs(wrap(a - (tdc - cfg.inj_end / 10.0), period_deg));
            if (e > r.inj_err) r.inj_err = e;
            double pe = fabs(on_us - pw_us);
            if (pe > r.pw_err_us) r.pw_err_us = pe;
        }
        r.injections += icount;

        /* firing exactly once per period over the window */
        double periods = (angle_at((double)to) - angle_at((double)from)) / period_deg;
        if ((cfg.enable & SCHED_EN_IGN) && fabs(count - periods) > 1.01) r.bad_count++;
        if ((cfg.enable & SCHED_EN_INJ) && fabs(icount - periods) > 1.01) r.bad_count++;
    }

    /* firing order 1-5-3-6-2-4: sort cylinders by first spark time */
    if (cfg.enable & SCHED_EN_IGN) {
        const int order[6] = {0, 4, 2, 5, 1, 3};
        int first = 0;
        for (int c = 1; c < 6; c++) if (first_spark[c] < first_spark[first]) first = c;
        int k = 0;
        while (order[k] != first) k++;
        for (int j = 1; j < 6; j++) {
            int a = order[(k + j - 1) % 6], b = order[(k + j) % 6];
            if (!(first_spark[b] > first_spark[a])) r.order_ok = 0;
        }
    }
    return r;
}

static int all_off(void)
{
    for (int c = 0; c < 12; c++) if (hw[c].pin) return 0;
    return 1;
}

static void set_cfg(uint8_t en, uint16_t tdc_off, int16_t adv, uint16_t dwell, uint16_t pw, uint16_t eoi)
{
    cfg.enable = en;
    cfg.spare = 0;
    cfg.tdc_offset = tdc_off;
    cfg.advance = adv;
    cfg.dwell_us = dwell;
    cfg.inj_pw_us = pw;
    cfg.inj_end = eoi;
}

/* ---- tests ------------------------------------------------------------------ */

static void test_steady(void)
{
    printf("sequential at steady speed (FULL sync)\n");
    const double speeds[] = {200, 800, 3000, 6000, 7200, 9000};
    for (unsigned i = 0; i < 6; i++) {
        sim_reset();
        set_cfg(3, 600, 100, 3000, 2500, 3800);
        engine_set(speeds[i]);
        run_s(0.5 + 1200.0 / speeds[i]);          /* sync + settle */
        CHECK(D.sync == SYNC_FULL);
        uint64_t from = g_tick;
        run_s(10 * 120.0 / speeds[i]);           /* 10 cycles */
        result_t r = analyse(from, g_tick, 720, 3000, 2500);
        printf("  %4.0f rpm: %d sparks, %d injections, spark angle err %.3f°, injection end err %.3f°, "
               "dwell err %.0f us, pw err %.0f us, late %u\n",
               speeds[i], r.sparks, r.injections, r.spark_err, r.inj_err, r.dwell_err_us, r.pw_err_us, S.late);
        CHECK(r.bad_count == 0 && r.order_ok);
        CHECK(r.sparks >= 54 && r.injections >= 54);
        /* 0.25°, or 2.5 timer ticks where that is coarser (1 tick = 0.17° at 7200 rpm, 0.22° at
         * 9000): an event referenced across the missing-tooth gap scales the 4-tooth period
         * estimate over up to 3.5 teeth. */
        double tol = fmax(0.25, 2.5 * speeds[i] * 6.0 * 4e-6);
        CHECK(r.spark_err < tol && r.inj_err < tol);
        CHECK(r.dwell_err_us <= 50 && r.pw_err_us <= 8);   /* dwell: spark refinement, < 2 % */
        CHECK(S.overlap == 0);
        CHECK(S.late == 0);
        CHECK(r.max_on_us <= 3050);
    }
}

static void test_crank_sync(void)
{
    printf("crank-only sync: wasted spark + per-revolution injection\n");
    sim_reset();
    eng_cam_mode = CAM_NONE;
    set_cfg(3, 600, 100, 3000, 5000, 3800);
    engine_set(1500);
    run_s(0.3);
    CHECK(D.sync == SYNC_CRANK);
    uint64_t from = g_tick;
    run_s(20 * 40e-3);                         /* 20 revolutions at 1500 rpm */
    result_t r = analyse(from, g_tick, 360, 3000, 2500);
    printf("  %d sparks, %d injections (per rev), spark err %.3f°, pw err %.0f us\n",
           r.sparks, r.injections, r.spark_err, r.pw_err_us);
    CHECK(r.bad_count == 0);
    CHECK(r.sparks >= 6 * 19 && r.injections >= 6 * 19);
    CHECK(r.spark_err < 0.25 && r.inj_err < 0.25 && r.pw_err_us <= 8);
}

static void test_start_from_rest(void)
{
    printf("start from rest: nothing fires before sync, then wasted spark -> sequential\n");
    sim_reset();
    set_cfg(3, 600, 100, 3000, 6000, 3800);
    engine_set(60);
    engine_ramp(250, 0.8);
    run_s(0.25);
    int early = 0;
    for (int c = 0; c < 12; c++) early += npulse[c];
    run_s(2.0);
    CHECK(D.sync == SYNC_FULL);
    result_t r = analyse(0, g_tick, 720, 3000, 6000);
    printf("  outputs before sync: %d, longest on %.0f us, sync %d\n", early, r.max_on_us, D.sync);
    CHECK(r.max_on_us <= 6010);
    /* no coil ever fired twice within 180° */
    int dbl = 0;
    for (int c = 0; c < 6; c++)
        for (int i = 1; i < npulse[SCHED_IGN(c)]; i++)
            if (angle_at((double)pulses[SCHED_IGN(c)][i].off) - angle_at((double)pulses[SCHED_IGN(c)][i - 1].off) < 180) dbl++;
    CHECK(dbl == 0);
}

static void test_acceleration(void)
{
    printf("acceleration 1000 -> 7000 rpm in 0.5 s\n");
    sim_reset();
    set_cfg(3, 600, 200, 3000, 2500, 3800);
    engine_set(1000);
    run_s(0.6);
    uint64_t from = g_tick;
    engine_ramp(7000, 0.5);
    run_s(0.55);
    result_t r = analyse(from, g_tick, 720, 3000, 2500);
    printf("  spark angle err %.2f°, dwell err %.0f us, injection end err %.1f° (not refined; pulse width is exact), "
           "missed/double %d, losses %u, late %u\n",
           r.spark_err, r.dwell_err_us, r.inj_err, r.bad_count, D.sync_losses, S.late);
    CHECK(r.bad_count == 0 && D.sync_losses == 0);
    CHECK(r.spark_err < 0.5);
    CHECK(r.dwell_err_us <= 300);       /* 10 % of the dwell at 12,000 rpm/s */
    CHECK(r.pw_err_us <= 8);
}

static void test_schedule_change(void)
{
    printf("big schedule changes while running: no missed or double events\n");
    sim_reset();
    set_cfg(3, 600, 100, 3000, 2500, 3800);
    engine_set(3000);
    run_s(0.5);
    uint64_t from = g_tick;
    const int16_t adv[] = {300, 50, 450, 100};
    const uint16_t eoi[] = {3600, 4100, 3300, 3800};
    for (int k = 0; k < 4; k++) {
        cfg.advance = adv[k];
        cfg.inj_end = eoi[k];
        run_s(0.2);
    }
    int ign = 0, inj = 0;
    for (int c = 0; c < 6; c++) {
        for (int i = 0; i < npulse[SCHED_IGN(c)]; i++) if (pulses[SCHED_IGN(c)][i].off > from) ign++;
        for (int i = 0; i < npulse[SCHED_INJ(c)]; i++) if (pulses[SCHED_INJ(c)][i].off > from) inj++;
    }
    /* 0.8 s at 3000 rpm = 20 cycles -> 120 events each, +/- one per channel at the window edges */
    printf("  sparks %d, injections %d (expect 120 +/- 6), late %u, overlap %u\n", ign, inj, S.late, S.overlap);
    CHECK(S.overlap == 0);
    CHECK(abs(ign - 120) <= 6 && abs(inj - 120) <= 6);
    int dbl = 0;                    /* (wasted spark before the change is 360° apart: excluded) */
    for (int c = 0; c < 12; c++)
        for (int i = 1; i < npulse[c]; i++)
            if (pulses[c][i - 1].off > from &&
                angle_at((double)pulses[c][i].off) - angle_at((double)pulses[c][i - 1].off) < 360) dbl++;
    CHECK(dbl == 0);
    from = g_tick;
    run_s(0.1);
    result_t r = analyse(from, g_tick, 720, 3000, 2500);
    CHECK(r.spark_err < 0.25 && r.inj_err < 0.25);
}

static void test_sync_loss(void)
{
    printf("sync loss (dropped tooth): pending starts cancelled, running pulses complete\n");
    sim_reset();
    set_cfg(3, 600, 100, 3000, 2500, 3800);
    engine_set(3000);
    run_s(0.5);
    eng_drop = 1;
    run_s(0.02);
    CHECK(D.sync < SYNC_CRANK && D.sync_losses == 1);
    int before[12];
    for (int c = 0; c < 12; c++) before[c] = npulse[c];
    uint64_t t_loss = g_tick;
    run_s(0.01);                                  /* still unsynced */
    int started = 0;
    for (int c = 0; c < 12; c++)
        for (int i = before[c]; i < npulse[c]; i++)
            if (pulses[c][i].on > t_loss) started++;
    result_t r = analyse(0, g_tick, 720, 3000, 2500);
    printf("  outputs started while unsynced: %d, longest on %.0f us\n", started, r.max_on_us);
    CHECK(started == 0);
    CHECK(r.max_on_us <= 3010);
    run_s(0.3);
    CHECK(D.sync == SYNC_FULL);
    uint64_t from = g_tick;
    run_s(0.2);
    r = analyse(from, g_tick, 720, 3000, 2500);
    CHECK(r.bad_count == 0 && r.spark_err < 0.25);
}

static void test_stall(void)
{
    printf("engine stops: everything off, nothing left dwelling\n");
    sim_reset();
    set_cfg(3, 600, 100, 6000, 8000, 3800);
    engine_set(400);
    run_s(1.0);
    engine_set(0);
    run_s(0.5);
    result_t r = analyse(0, g_tick, 720, 6000, 8000);
    printf("  sync %d, all outputs off %d, longest on %.0f us\n", D.sync, all_off(), r.max_on_us);
    CHECK(D.sync == SYNC_NONE && all_off());
    for (int c = 0; c < 12; c++) CHECK(S.ch[c].state == CH_IDLE);
    CHECK(r.max_on_us <= 8010);
}

static void test_disable(void)
{
    printf("outputs disabled while running\n");
    sim_reset();
    set_cfg(3, 600, 100, 3000, 2500, 3800);
    engine_set(3000);
    run_s(0.5);
    cfg.enable = 0;
    run_s(0.05);
    int n0 = 0, n1 = 0;
    for (int c = 0; c < 12; c++) n0 += npulse[c];
    run_s(0.2);
    for (int c = 0; c < 12; c++) n1 += npulse[c];
    printf("  pulses after disable: %d\n", n1 - n0);
    CHECK(n1 == n0 && all_off());
}

static void test_limits(void)
{
    printf("limits: injector duty cap, dwell cap at high rpm\n");
    sim_reset();
    set_cfg(2, 600, 100, 3000, 30000, 3800);      /* 30 ms request at 7200 rpm (16.7 ms cycle) */
    engine_set(7200);
    run_s(0.5);
    uint64_t from = g_tick;
    run_s(0.2);
    double cycle_us = 60e6 / 7200 * 2;
    result_t r = analyse(from, g_tick, 720, 3000, cycle_us * 0.9);
    printf("  injector on %.0f us of a %.0f us cycle (90 %% cap), overlap %u\n", r.max_on_us, cycle_us, S.overlap);
    CHECK(fabs(r.max_on_us - cycle_us * 0.9) <= cycle_us * 0.005);
    CHECK(S.overlap == 0 && r.bad_count == 0);

    sim_reset();
    set_cfg(1, 600, 100, 8000, 2500, 3800);       /* 8 ms dwell at 9000 rpm: period 13.3 ms */
    engine_set(9000);
    run_s(0.5);
    from = g_tick;
    run_s(0.2);
    r = analyse(from, g_tick, 720, 0, 0);
    printf("  dwell request 8000 us at 9000 rpm -> %.0f us (half the 13333 us period)\n", r.max_on_us);
    CHECK(r.max_on_us <= 13333.0 / 2 * 1.01);
    CHECK(r.bad_count == 0);

    /* invalid configs are rejected */
    sched_cfg_t bad = cfg;
    bad.dwell_us = 9000; CHECK(!sched_cfg_valid(&bad));
    bad = cfg; bad.advance = 700; CHECK(!sched_cfg_valid(&bad));
    bad = cfg; bad.tdc_offset = 7200; CHECK(!sched_cfg_valid(&bad));
    bad = cfg; bad.enable = 4; CHECK(!sched_cfg_valid(&bad));
}

static void test_to_ref(void)
{
    printf("angle -> reference tooth conversion\n");
    int bad = 0;
    for (uint32_t pos = 0; pos < SCHED_UNITS_CYCLE; pos += 7) {
        uint8_t ref; uint16_t frac;
        sched_to_ref((uint16_t)pos, &ref, &frac);
        uint8_t tooth = (uint8_t)(ref % 60);
        if (tooth > DEC_LAST_TOOTH) bad++;                          /* must be a real tooth */
        if (frac < SCHED_LEAD_UNITS) bad++;                         /* enough lead */
        if (frac > 3u * 256u + SCHED_LEAD_UNITS) bad++;             /* at most across the gap */
        if ((ref * 256u + frac) % SCHED_UNITS_CYCLE != pos) bad++;  /* exact */
    }
    CHECK(bad == 0);
    CHECK(sched_deg10_to_units(60) == 256 && sched_deg10_to_units(7199) == 30716);
}

int main(void)
{
    test_to_ref();
    test_steady();
    test_crank_sync();
    test_start_from_rest();
    test_acceleration();
    test_schedule_change();
    test_sync_loss();
    test_stall();
    test_disable();
    test_limits();
    printf("scheduler: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
