/*
 * Host tests for the Phase 9a controls (control_core.h) against simple
 * plant models: an engine whose fuel is off by a known amount (lambda), an
 * idle valve -> rpm model with lag, and the switching narrowband sensors.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "engine/control_core.h"

uint8_t tune_page0[TUNE_PAGE0_SIZE];
uint8_t tune_page1[TUNE_PAGE1_SIZE];

static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("  FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static void defaults(void)
{
    memcpy(tune_page0, tune_default_page0, TUNE_PAGE0_SIZE);
    memcpy(tune_page1, tune_default_page1, TUNE_PAGE1_SIZE);
}

static ctl_in_t warm_cruise(void)
{
    ctl_in_t in;
    memset(&in, 0, sizeof in);
    in.rpm = 2500;
    in.tps_x10 = 150;
    in.clt_x10 = 850;
    in.air_mg = 250;
    in.afr_target_c = 1470;
    in.afr_c = 1470;
    in.nb_a_mv = 450;
    in.nb_b_mv = 450;
    in.running = 1;
    return in;
}

static void test_dfco(void)
{
    printf("overrun fuel cut\n");
    defaults();
    ctl_t c;
    ctl_init(&c);
    ctl_in_t in = warm_cruise();
    in.rpm = 3000;
    in.tps_x10 = 0;
    int t = 0;
    while (!(c.flags & CTL_DFCO) && t < 200) { ctl_update(&c, &in); t++; }
    printf("  closed throttle at 3000 rpm: fuel cut after %d ms\n", t * 10);
    CHECK(t >= 50 && t <= 52);                          /* dfcoDelay 0.5 s */
    in.rpm = 1500; ctl_update(&c, &in);
    CHECK(c.flags & CTL_DFCO);                          /* still above the resume speed */
    in.rpm = 1390; ctl_update(&c, &in);
    CHECK(!(c.flags & CTL_DFCO));
    in.rpm = 3000;
    for (int i = 0; i < 60; i++) ctl_update(&c, &in);
    CHECK(c.flags & CTL_DFCO);
    in.tps_x10 = 50; ctl_update(&c, &in);               /* throttle opens: fuel back at once */
    CHECK(!(c.flags & CTL_DFCO));
    in.tps_x10 = 0; in.clt_x10 = 300;                   /* cold engine: no cut */
    for (int i = 0; i < 100; i++) ctl_update(&c, &in);
    CHECK(!(c.flags & CTL_DFCO));
}

/* lambda plant: measured AFR = target x (1 + bias) / (1 + trim) */
static uint16_t plant_afr(const ctl_t *c, uint16_t target, double bias)
{
    return (uint16_t)lround(target * (1 + bias) / (1 + c->trim_a / 1000.0));
}

static void test_wideband(void)
{
    printf("wideband closed loop\n");
    defaults();
    tune_page0[TU_LAMBDA_MODE_OFF] = 2;
    ctl_t c;
    ctl_init(&c);
    ctl_in_t in = warm_cruise();
    for (int i = 0; i < 3000; i++) ctl_update(&c, &in);          /* 30 s start delay */
    CHECK(c.flags & CTL_CLOSED);
    double t_settle = -1;
    for (int i = 0; i < 2000; i++) {
        in.afr_c = plant_afr(&c, in.afr_target_c, 0.08);           /* fuel 8 % lean */
        ctl_update(&c, &in);
        if (t_settle < 0 && fabs(c.trim_a / 10.0 - 8.0) < 0.5) t_settle = i / 100.0;
    }
    printf("  8 %% lean engine: trim %.1f %% (settled within 0.5 %% after %.1f s), AFR %.2f\n",
           c.trim_a / 10.0, t_settle, in.afr_c / 100.0);
    CHECK(fabs(c.trim_a / 10.0 - 8.0) < 0.5 && c.trim_b == c.trim_a);
    CHECK(t_settle > 0 && t_settle < 10);

    /* out of the closed-loop range: no trim applied, learned value kept */
    in.air_mg = 600;
    ctl_update(&c, &in);
    CHECK(c.trim_a == 0 && !(c.flags & CTL_CLOSED));
    CHECK(fabs(c.wb_i / 10240.0 - 8.0) < 0.5);              /* integral kept while open loop */
    in.air_mg = 250;
    for (int i = 0; i < 300; i++) {
        in.afr_c = plant_afr(&c, in.afr_target_c, 0.08);
        ctl_update(&c, &in);
    }
    CHECK(fabs(c.trim_a / 10.0 - 8.0) < 0.5);
    /* accel enrichment / overrun / cold: no closed loop */
    in.accel = 1; ctl_update(&c, &in); CHECK(c.trim_a == 0); in.accel = 0;
    in.clt_x10 = 300; ctl_update(&c, &in); CHECK(c.trim_a == 0); in.clt_x10 = 850;

    /* 25 % rich engine: correction stops at the 15 % limit */
    for (int i = 0; i < 3000; i++) {
        in.afr_c = plant_afr(&c, in.afr_target_c, -0.25);
        ctl_update(&c, &in);
    }
    printf("  25 %% rich engine: trim %.1f %% (limit 15 %%)\n", c.trim_a / 10.0);
    CHECK(c.trim_a == -150);

    /* wideband at its 0 V rail (warming up / unplugged): mode 2 opens the loop */
    in.afr_c = 1000;
    ctl_update(&c, &in);
    CHECK((c.flags & CTL_WB_BAD) && !(c.flags & CTL_CLOSED) && c.trim_a == 0);
}

static void test_narrowband(void)
{
    printf("narrowband closed loop, per bank\n");
    defaults();
    tune_page0[TU_LAMBDA_MODE_OFF] = 1;
    ctl_t c;
    ctl_init(&c);
    ctl_in_t in = warm_cruise();
    for (int i = 0; i < 3000; i++) ctl_update(&c, &in);
    /* banks: A 5 % lean, B 3 % rich; sensor sees the exhaust 100 ms later */
    double hist_a[10] = {0}, hist_b[10] = {0};
    double sum_a = 0, sum_b = 0;
    int n = 0;
    for (int i = 0; i < 4000; i++) {
        double lam_a = 1.05 / (1 + c.trim_a / 1000.0), lam_b = 0.97 / (1 + c.trim_b / 1000.0);
        double da = hist_a[i % 10], db = hist_b[i % 10];
        hist_a[i % 10] = lam_a;
        hist_b[i % 10] = lam_b;
        in.nb_a_mv = da < 1.0 && i >= 10 ? 800 : 100;
        in.nb_b_mv = db < 1.0 && i >= 10 ? 800 : 100;
        ctl_update(&c, &in);
        if (i >= 2000) { sum_a += c.trim_a; sum_b += c.trim_b; n++; }
    }
    printf("  bank A (5 %% lean) average trim %.1f %%, bank B (3 %% rich) %.1f %%\n", sum_a / n / 10, sum_b / n / 10);
    CHECK(fabs(sum_a / n / 10 - 5.0) < 1.0);
    CHECK(fabs(sum_b / n / 10 + 3.0) < 1.0);
    CHECK(c.flags & CTL_CLOSED);

    /* target away from 14.7 (power enrichment): narrowbands cannot control */
    in.afr_target_c = 1300;
    ctl_update(&c, &in);
    CHECK(!(c.flags & CTL_CLOSED) && c.trim_a == 0);

    /* mode 3: wideband in charge, narrowbands take over if it is implausible */
    tune_page0[TU_LAMBDA_MODE_OFF] = 3;
    ctl_init(&c);
    in = warm_cruise();
    for (int i = 0; i < 3000; i++) ctl_update(&c, &in);
    in.afr_c = 1550;                                    /* lean on the wideband */
    for (int i = 0; i < 100; i++) ctl_update(&c, &in);
    CHECK((c.flags & CTL_CLOSED) && c.trim_a > 0 && c.trim_a == c.trim_b);
    in.afr_c = 1990;                                    /* at the 5 V rail: implausible */
    in.nb_a_mv = 800;                                   /* bank A rich, bank B lean */
    in.nb_b_mv = 100;
    for (int i = 0; i < 300; i++) ctl_update(&c, &in);
    CHECK((c.flags & CTL_CLOSED) && (c.flags & CTL_WB_BAD) && c.trim_a < 0 && c.trim_b > 0);
}

static void test_idle(void)
{
    printf("idle control\n");
    defaults();
    ctl_t c;
    ctl_init(&c);
    ctl_in_t in = warm_cruise();
    in.tps_x10 = 0;
    in.clt_x10 = 800;                                   /* target 800 rpm, base 30 % */
    in.rpm = 1000;
    double rpm = 1000, load = 0;
    for (int i = 0; i < 2000; i++) {                    /* 20 s */
        double want = 400 + 20.0 * c.iac_duty - load;   /* valve -> speed, with lag */
        rpm += (want - rpm) * 0.05;
        in.rpm = (uint16_t)lround(rpm);
        if (i == 1200) load = 150;                      /* A/C switches on */
        ctl_update(&c, &in);
    }
    printf("  warm idle: %.0f rpm (target %u), valve %u %%, timing %+.1f deg\n",
           rpm, c.idle_target, c.iac_duty, c.idle_ign_x10 / 10.0);
    CHECK(c.idle_target == 800);
    CHECK(fabs(rpm - 800) < 25);
    CHECK(c.flags & CTL_IDLE);

    /* timing idle control pushes the right way */
    in.rpm = 900; ctl_update(&c, &in);
    CHECK(c.idle_ign_x10 < 0);
    in.rpm = 650; ctl_update(&c, &in);
    CHECK(c.idle_ign_x10 > 0 && c.idle_ign_x10 <= 80);

    /* cold: higher target; cranking: fixed opening */
    in.clt_x10 = -100;
    ctl_update(&c, &in);
    CHECK(c.idle_target == 1250);
    in.cranking = 1;
    ctl_update(&c, &in);
    CHECK(c.iac_duty == 45 && !(c.flags & CTL_IDLE));

    /* dashpot: throttle closes -> +6 % fading at 10 %/s */
    ctl_init(&c);
    in = warm_cruise();
    in.clt_x10 = 800;
    in.tps_x10 = 200; in.rpm = 2500;
    ctl_update(&c, &in);
    uint8_t open_duty = c.iac_duty;
    in.tps_x10 = 0;
    ctl_update(&c, &in);
    CHECK(c.iac_duty >= open_duty + 5);
    for (int i = 0; i < 70; i++) ctl_update(&c, &in);
    CHECK(c.iac_duty <= open_duty + 1);
}

static void test_vanos_heaters(void)
{
    printf("VANOS and heaters\n");
    defaults();
    ctl_t c;
    ctl_init(&c);
    ctl_in_t in = warm_cruise();
    in.tps_x10 = 200;
    in.rpm = 1400; ctl_update(&c, &in); CHECK(!(c.flags & CTL_VANOS));
    in.rpm = 1600; ctl_update(&c, &in); CHECK(c.flags & CTL_VANOS);
    in.rpm = 1450; ctl_update(&c, &in); CHECK(c.flags & CTL_VANOS);      /* hysteresis */
    in.rpm = 1350; ctl_update(&c, &in); CHECK(!(c.flags & CTL_VANOS));
    in.rpm = 4000; ctl_update(&c, &in); CHECK(c.flags & CTL_VANOS);
    in.rpm = 5080; ctl_update(&c, &in); CHECK(c.flags & CTL_VANOS);
    in.rpm = 5120; ctl_update(&c, &in); CHECK(!(c.flags & CTL_VANOS));
    in.rpm = 3000; in.tps_x10 = 50; ctl_update(&c, &in); CHECK(!(c.flags & CTL_VANOS));
    in.tps_x10 = 200; in.clt_x10 = 200; ctl_update(&c, &in); CHECK(!(c.flags & CTL_VANOS));

    ctl_init(&c);
    in = warm_cruise();
    for (int i = 0; i < 999; i++) ctl_update(&c, &in);
    CHECK(!(c.flags & CTL_HEATERS));
    ctl_update(&c, &in);
    CHECK(c.flags & CTL_HEATERS);
    in.running = 0; ctl_update(&c, &in);
    CHECK(!(c.flags & CTL_HEATERS));
}

int main(void)
{
    test_dfco();
    test_wideband();
    test_narrowband();
    test_idle();
    test_vanos_heaters();
    printf("control: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
