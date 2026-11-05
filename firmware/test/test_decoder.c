/*
 * Host tests for the crank/cam decoder core.
 *
 * A simulated engine produces edges with the same pattern code as the
 * hardware trigger simulator (tools/stim/pattern.h), converted to the ECU's
 * 16-bit, 4 us timestamps (so timer wrap-around is exercised constantly).
 * The test knows the true tooth and revolution of every edge and checks the
 * decoder never reports a wrong position while it claims sync.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "decoder/decoder_core.h"
#include "pattern.h"

static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("  FAIL line %d: %s\n", __LINE__, #c); } } while (0)

/* ---- simulated engine ----------------------------------------------------- */

typedef double (*rpm_fn)(double t_s, double crank_deg, void *arg);

typedef struct {
    double   t;             /* time, 4 us ticks */
    uint64_t steps;         /* half-steps since start */
    uint8_t  step;          /* 0..239 */
    uint8_t  out;
    uint8_t  cam_mode, cam_start;
    int      drop_tooth;    /* skip the next crank edge */
    double   spike_frac;    /* >0: add a spurious crank edge this fraction of a tooth later */
    int      extra_cam;     /* >0: add a spurious cam edge on the next crank edge */
    int      cam_off;       /* stop producing cam edges */

    /* results */
    int      pos_errors;
    int      map;           /* decoder rev XOR true rev, -1 = not seen yet */
    int      map_errors;
    int      full_at_rev;   /* revolutions until FULL first reached, -1 = never */
    int      crank_at_rev;
    uint32_t edges;
} engine_t;

static void engine_init(engine_t *e, uint8_t cam_mode, uint8_t cam_start, uint8_t start_step)
{
    memset(e, 0, sizeof *e);
    e->cam_mode = cam_mode;
    e->cam_start = cam_start;
    e->step = start_step;
    e->out = stim_pattern(start_step, cam_mode, cam_start);
    e->t = 12345.0;
    e->map = -1;
    e->full_at_rev = -1;
    e->crank_at_rev = -1;
}

static uint16_t ticks16(double t) { return (uint16_t)(uint64_t)llround(t); }

static void check_position(engine_t *e, decoder_t *d)
{
    uint8_t true_tooth = (uint8_t)((e->step >> 1) % 60);
    uint8_t true_rev = e->step >= 120;
    int revs = (int)(e->steps / 120);

    if (d->sync >= SYNC_CRANK && e->crank_at_rev < 0) e->crank_at_rev = revs;
    if (d->sync == SYNC_FULL && e->full_at_rev < 0) e->full_at_rev = revs;

    if (d->sync >= SYNC_CRANK && d->tooth != true_tooth) e->pos_errors++;
    if (d->sync == SYNC_FULL) {
        int m = d->rev ^ true_rev;
        if (e->map < 0) e->map = m;
        else if (m != e->map) e->map_errors++;
    }
}

/* Advance one half-step (3°) at the speed given by f. */
static void engine_step(engine_t *e, decoder_t *d, rpm_fn f, void *arg)
{
    double t_s = e->t * 4e-6;
    double deg = (double)e->steps * 3.0;
    double rpm = f(t_s, deg, arg);
    double step_ticks = 125000.0 / rpm;         /* 3° at rpm, in 4 us ticks */

    e->t += step_ticks;
    e->steps++;
    e->step = (uint8_t)((e->step + 1) % STIM_STEPS);
    uint8_t n = stim_pattern(e->step, e->cam_mode, e->cam_start);
    if (e->cam_off) n &= (uint8_t)~STIM_OUT_CAM;

    if ((n & STIM_OUT_CRANK) && !(e->out & STIM_OUT_CRANK)) {
        if (e->drop_tooth) {
            e->drop_tooth = 0;
        } else {
            dec_crank_edge(d, ticks16(e->t));
            e->edges++;
            check_position(e, d);
            if (e->spike_frac > 0) {
                dec_crank_edge(d, ticks16(e->t + e->spike_frac * 2.0 * step_ticks));
                e->spike_frac = 0;
            }
            if (e->extra_cam) {
                dec_cam_edge(d);
                e->extra_cam = 0;
            }
        }
    }
    if ((n & STIM_OUT_CAM) && !(e->out & STIM_OUT_CAM)) {
        dec_cam_edge(d);
    }
    e->out = n;
}

static void engine_run_revs(engine_t *e, decoder_t *d, double revs, rpm_fn f, void *arg)
{
    uint64_t n = (uint64_t)(revs * 120.0);
    for (uint64_t i = 0; i < n; i++) engine_step(e, d, f, arg);
}

static void engine_run_s(engine_t *e, decoder_t *d, double seconds, rpm_fn f, void *arg)
{
    double end = e->t + seconds / 4e-6;
    while (e->t < end) engine_step(e, d, f, arg);
}

/* Run until true position is the given tooth of the given true revolution. */
static void engine_run_to(engine_t *e, decoder_t *d, uint8_t rev, uint8_t tooth, rpm_fn f, void *arg)
{
    uint8_t target = (uint8_t)(rev * 120 + tooth * 2);
    do { engine_step(e, d, f, arg); } while (e->step != target);
}

/* ---- speed profiles -------------------------------------------------------- */

static double rpm_const(double t, double deg, void *arg) { (void)t; (void)deg; return *(double *)arg; }

typedef struct { double from, to, t0, dur; } ramp_t;
static double rpm_ramp(double t, double deg, void *arg)
{
    (void)deg;
    ramp_t *r = arg;
    double x = (t - r->t0) / r->dur;
    if (x < 0) x = 0;
    if (x > 1) x = 1;
    return r->from + (r->to - r->from) * x;
}

typedef struct { double base, amp; } crank_t;
/* Cranking: speed dips at each of the 3 compression strokes per revolution. */
static double rpm_cranking(double t, double deg, void *arg)
{
    (void)t;
    crank_t *c = arg;
    return c->base * (1.0 + c->amp * sin(3.0 * deg * M_PI / 180.0));
}

/* ---- tests ------------------------------------------------------------------ */

static int expected_map(uint8_t cam_start) { return cam_start < 120 ? 1 : 0; }

static void test_steady_speeds(void)
{
    printf("steady speeds, half-moon cam\n");
    const double speeds[] = {100, 250, 800, 1000, 3000, 6000, 7200, 8000, 9000};
    for (unsigned i = 0; i < sizeof speeds / sizeof speeds[0]; i++) {
        double rpm = speeds[i];
        decoder_t d; dec_init(&d, 1);
        engine_t e; engine_init(&e, CAM_HALFMOON, 41, 0);
        engine_run_revs(&e, &d, 10, rpm_const, &rpm);

        uint16_t r = dec_rpm(d.rev_ticks);
        double err = fabs(r - rpm) / rpm * 100.0;
        printf("  %5.0f rpm: reads %u (%.2f %%), CRANK after %d rev, FULL after %d rev\n",
               rpm, r, err, e.crank_at_rev, e.full_at_rev);
        CHECK(d.sync == SYNC_FULL);
        CHECK(e.pos_errors == 0 && e.map_errors == 0);
        CHECK(e.map == expected_map(41));
        CHECK(err < 0.2);
        CHECK(e.crank_at_rev <= 2 && e.full_at_rev <= 3);
        CHECK(d.sync_losses == 0 && d.noise_teeth == 0 && d.cam_errors == 0);
        uint16_t ri = dec_rpm_tooth(d.tooth_period);
        CHECK(fabs(ri - rpm) / rpm < 0.05);      /* 4 us tick quantisation per tooth */
    }
}

static void test_cam_patterns(void)
{
    printf("cam patterns and positions\n");
    const uint8_t modes[] = {CAM_PULSE, CAM_HALFMOON};
    const uint8_t starts[] = {11, 81, 131, 201};
    double rpm = 2000;
    for (unsigned m = 0; m < 2; m++) {
        for (unsigned s = 0; s < 4; s++) {
            decoder_t d; dec_init(&d, 1);
            engine_t e; engine_init(&e, modes[m], starts[s], 0);
            engine_run_revs(&e, &d, 8, rpm_const, &rpm);
            CHECK(d.sync == SYNC_FULL);
            CHECK(e.pos_errors == 0 && e.map_errors == 0);
            CHECK(e.map == expected_map(starts[s]));
            /* cam_pos reports the tooth just before the cam edge. */
            uint8_t cam_tooth = (uint8_t)(((starts[s] % 120) - 1) / 2);
            uint8_t want = (uint8_t)(cam_tooth + ((starts[s] >= 120) ^ e.map ? 60 : 0));
            CHECK(d.cam_pos == want);
        }
    }
}

static void test_start_from_rest(void)
{
    printf("start from rest, every starting position\n");
    int worst_full = 0;
    for (uint8_t start = 0; start < STIM_STEPS; start += 7) {
        decoder_t d; dec_init(&d, 1);
        engine_t e; engine_init(&e, CAM_HALFMOON, 41, start);
        ramp_t r = {60, 250, e.t * 4e-6, 0.6};
        engine_run_s(&e, &d, 1.5, rpm_ramp, &r);
        CHECK(d.sync == SYNC_FULL);
        CHECK(e.pos_errors == 0 && e.map_errors == 0);
        CHECK(d.sync_losses == 0);
        if (e.full_at_rev > worst_full) worst_full = e.full_at_rev;
    }
    printf("  worst case: FULL sync after %d revolutions\n", worst_full);
    CHECK(worst_full <= 4);
}

static void test_cranking_fluctuation(void)
{
    printf("cranking with compression-stroke speed fluctuation\n");
    const double amps[] = {0.10, 0.25, 0.35};
    for (unsigned i = 0; i < 3; i++) {
        decoder_t d; dec_init(&d, 1);
        engine_t e; engine_init(&e, CAM_HALFMOON, 41, 0);
        crank_t c = {200, amps[i]};
        engine_run_revs(&e, &d, 20, rpm_cranking, &c);
        printf("  200 rpm +/-%2.0f %%: sync %d, losses %u, noise %u\n",
               amps[i] * 100, d.sync, d.sync_losses, d.noise_teeth);
        CHECK(d.sync == SYNC_FULL);
        CHECK(e.pos_errors == 0 && e.map_errors == 0);
        CHECK(d.sync_losses == 0);
    }
}

static void test_acceleration(void)
{
    printf("acceleration / deceleration\n");
    struct { double from, to, dur; } cases[] = {
        {800, 7200, 2.0}, {7200, 800, 1.0}, {1000, 7000, 0.4}, {7000, 1000, 0.4},
    };
    for (unsigned i = 0; i < 4; i++) {
        decoder_t d; dec_init(&d, 1);
        engine_t e; engine_init(&e, CAM_HALFMOON, 41, 0);
        double start = cases[i].from;
        engine_run_revs(&e, &d, 6, rpm_const, &start);
        ramp_t r = {cases[i].from, cases[i].to, e.t * 4e-6, cases[i].dur};
        engine_run_s(&e, &d, cases[i].dur + 0.1, rpm_ramp, &r);
        uint16_t rr = dec_rpm(d.rev_ticks);
        printf("  %4.0f -> %4.0f rpm in %.1f s: sync %d, losses %u, reads %u\n",
               cases[i].from, cases[i].to, cases[i].dur, d.sync, d.sync_losses, rr);
        CHECK(d.sync == SYNC_FULL);
        CHECK(e.pos_errors == 0 && e.map_errors == 0);
        CHECK(d.sync_losses == 0 && d.cam_errors == 0);
        CHECK(fabs(rr - cases[i].to) / cases[i].to < 0.01);
    }
}

static void test_dropped_tooth(void)
{
    printf("dropped tooth -> sync lost, recovered\n");
    double rpm = 3000;
    decoder_t d; dec_init(&d, 1);
    engine_t e; engine_init(&e, CAM_HALFMOON, 41, 0);
    engine_run_revs(&e, &d, 6, rpm_const, &rpm);
    CHECK(d.sync == SYNC_FULL);
    engine_run_to(&e, &d, 1, 29, rpm_const, &rpm);
    e.drop_tooth = 1;
    engine_run_revs(&e, &d, 1, rpm_const, &rpm);
    CHECK(d.sync_losses == 1 && d.loss_reason == LOSS_COUNT);
    CHECK(d.sync < SYNC_CRANK);
    engine_run_revs(&e, &d, 4, rpm_const, &rpm);
    CHECK(d.sync == SYNC_FULL);
    CHECK(e.pos_errors == 0 && e.map_errors == 0);
    printf("  losses %u, now sync %d, position errors while synced %d\n", d.sync_losses, d.sync, e.pos_errors);
}

static void test_noise(void)
{
    printf("electrical noise on the crank signal\n");
    double rpm = 3000;
    decoder_t d; dec_init(&d, 1);
    engine_t e; engine_init(&e, CAM_HALFMOON, 41, 0);
    engine_run_revs(&e, &d, 6, rpm_const, &rpm);

    /* Short spike (10 % of a tooth): rejected, sync kept. */
    e.spike_frac = 0.10;
    engine_run_revs(&e, &d, 2, rpm_const, &rpm);
    CHECK(d.noise_teeth == 1);
    CHECK(d.sync_losses == 0 && d.sync == SYNC_FULL);

    /* Spike at half a tooth: also rejected once synced. */
    e.spike_frac = 0.5;
    engine_run_revs(&e, &d, 2, rpm_const, &rpm);
    CHECK(d.noise_teeth == 2);
    CHECK(d.sync_losses == 0 && d.sync == SYNC_FULL);

    /* Spike at 70 % of a tooth is accepted as a tooth and the real tooth 30 %
     * later is rejected. The interval after that then looks like a gap, so the
     * tooth count is wrong at the next gap check: sync is dropped (outputs
     * off) and re-acquired. The decoder never claims a wrong position. */
    e.spike_frac = 0.7;
    engine_run_revs(&e, &d, 1, rpm_const, &rpm);
    CHECK(d.noise_teeth == 3);
    CHECK(d.sync_losses == 1 && d.loss_reason == LOSS_COUNT);
    engine_run_revs(&e, &d, 4, rpm_const, &rpm);
    CHECK(d.sync == SYNC_FULL);
    CHECK(e.pos_errors == 0 && e.map_errors == 0);

    /* Before sync the looser rule applies: a half-tooth spike while PENDING is
     * counted as a tooth, so the count is wrong and sync is only declared
     * after a clean revolution. */
    dec_init(&d, 1);
    engine_init(&e, CAM_HALFMOON, 41, 0);
    engine_run_to(&e, &d, 1, 30, rpm_const, &rpm);   /* first gap seen: PENDING */
    CHECK(d.sync == SYNC_PENDING);
    e.spike_frac = 0.5;
    engine_run_revs(&e, &d, 1, rpm_const, &rpm);
    CHECK(d.sync == SYNC_PENDING);
    engine_run_revs(&e, &d, 4, rpm_const, &rpm);
    CHECK(d.sync == SYNC_FULL);
    CHECK(e.pos_errors == 0 && e.map_errors == 0);
    printf("  rejected spikes %u, sync losses %u, position errors %d\n", d.noise_teeth, d.sync_losses, e.pos_errors);
}

static void test_cam_faults(void)
{
    printf("cam faults\n");
    double rpm = 2500;

    /* No cam signal at all: runs on CRANK sync (wasted spark), never FULL. */
    decoder_t d; dec_init(&d, 1);
    engine_t e; engine_init(&e, CAM_NONE, 0, 0);
    engine_run_revs(&e, &d, 10, rpm_const, &rpm);
    CHECK(d.sync == SYNC_CRANK && e.full_at_rev < 0 && e.pos_errors == 0);

    /* Cam disabled in config: same, even with a cam signal. */
    dec_init(&d, 0);
    engine_init(&e, CAM_HALFMOON, 41, 0);
    engine_run_revs(&e, &d, 10, rpm_const, &rpm);
    CHECK(d.sync == SYNC_CRANK && e.full_at_rev < 0);

    /* Spurious cam edge in the wrong revolution: drop to CRANK, recover with
     * the same (correct) phase. */
    dec_init(&d, 1);
    engine_init(&e, CAM_PULSE, 41, 0);
    engine_run_revs(&e, &d, 6, rpm_const, &rpm);
    CHECK(d.sync == SYNC_FULL);
    engine_run_to(&e, &d, 1, 30, rpm_const, &rpm);   /* true rev 1: no cam here */
    e.extra_cam = 1;
    engine_run_revs(&e, &d, 1, rpm_const, &rpm);
    CHECK(d.cam_errors == 1 && d.sync == SYNC_CRANK);
    engine_run_revs(&e, &d, 3, rpm_const, &rpm);
    CHECK(d.sync == SYNC_FULL && e.pos_errors == 0 && e.map_errors == 0);

    /* Cam signal disappears: drop to CRANK and stay there, engine keeps running. */
    e.cam_off = 1;
    engine_run_revs(&e, &d, 4, rpm_const, &rpm);
    CHECK(d.sync == SYNC_CRANK && d.cam_errors == 2 && d.sync_losses == 0);
    printf("  cam errors %u, final sync %d\n", d.cam_errors, d.sync);
}

static void test_stall_and_restart(void)
{
    printf("engine stops and restarts\n");
    double rpm = 900;
    decoder_t d; dec_init(&d, 1);
    engine_t e; engine_init(&e, CAM_HALFMOON, 41, 0);
    engine_run_revs(&e, &d, 6, rpm_const, &rpm);
    CHECK(d.sync == SYNC_FULL);

    uint16_t last = d.last_time;
    CHECK(dec_check_stall(&d, (uint16_t)(last + 40000u)) == 0);
    CHECK(dec_check_stall(&d, (uint16_t)(last + DEC_STALL_TICKS + 1u)) == 1);
    CHECK(d.sync == SYNC_NONE && d.loss_reason == LOSS_STALL && d.rev_ticks == 0);
    CHECK(d.sync_losses == 0);          /* stopping is not a fault */

    /* Restart after a pause, from wherever the engine stopped. */
    e.t += 2.0 / 4e-6;
    ramp_t r = {60, 250, e.t * 4e-6, 0.5};
    engine_run_s(&e, &d, 1.5, rpm_ramp, &r);
    CHECK(d.sync == SYNC_FULL && e.pos_errors == 0 && e.map_errors == 0);
}

static void test_tooth_log(void)
{
    printf("tooth logger\n");
    double rpm = 1500;
    decoder_t d; dec_init(&d, 1);
    engine_t e; engine_init(&e, CAM_HALFMOON, 41, 0);
    engine_run_revs(&e, &d, 6, rpm_const, &rpm);
    CHECK(d.log_count == 0);            /* idle until a capture is requested */
    dec_log_start(&d);
    engine_run_revs(&e, &d, 0.5, rpm_const, &rpm);
    CHECK(d.log_active && d.log_count > 25 && d.log_count < 35);
    engine_run_revs(&e, &d, 2, rpm_const, &rpm);
    CHECK(!d.log_active && d.log_count == DEC_LOG_SIZE);

    int gaps = 0, cams = 0, full = 0;
    for (unsigned i = 0; i < DEC_LOG_SIZE; i++) {
        gaps += !!(d.log[i].flags & LOG_GAP);
        cams += !!(d.log[i].flags & LOG_CAM);
        full += (d.log[i].flags >> 4) == SYNC_FULL;
        if (d.log[i].flags & LOG_GAP) {
            CHECK(d.log[i].interval > 2 * 167 && d.log[i].interval < 4 * 167);   /* 3 teeth at 1500 rpm */
            CHECK((d.log[i].pos & 0x7F) == 0);
        }
    }
    CHECK(gaps == 1 || gaps == 2);      /* 64 entries span ~1.1 revolutions */
    CHECK(cams <= 1 && full == DEC_LOG_SIZE);
}

int main(void)
{
    test_steady_speeds();
    test_cam_patterns();
    test_start_from_rest();
    test_cranking_fluctuation();
    test_acceleration();
    test_dropped_tooth();
    test_noise();
    test_cam_faults();
    test_stall_and_restart();
    test_tooth_log();
    printf("decoder: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
