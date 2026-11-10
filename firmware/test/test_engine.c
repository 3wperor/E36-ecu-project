/*
 * Host tests for sensor conversion and the fuel / ignition calculation.
 * Results are compared with an independent floating-point calculation.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "engine/engine_core.h"

uint8_t tune_page0[TUNE_PAGE0_SIZE];
uint8_t tune_page1[TUNE_PAGE1_SIZE];

static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("  FAIL line %d: %s\n", __LINE__, #c); } } while (0)
#define NEAR(a, b, tol) (fabs((double)(a) - (double)(b)) <= (tol))

static void defaults(void)
{
    memcpy(tune_page0, tune_default_page0, TUNE_PAGE0_SIZE);
    memcpy(tune_page1, tune_default_page1, TUNE_PAGE1_SIZE);
}
static void set8(uint8_t *pg, unsigned off, uint8_t v) { pg[off] = v; }
static void fill8(uint8_t *pg, unsigned off, unsigned n, uint8_t v) { memset(pg + off, v, n); }

static uint16_t adc_for_mv(double mv) { return (uint16_t)lround(mv * 1023.0 / 5000.0); }

/* NTC model the default thermistor curve was generated from */
static uint16_t adc_for_temp(double t)
{
    double beta = log(2500.0 / 330.0) / (1 / 293.15 - 1 / 353.15);
    double r = 2500.0 * exp(beta * (1 / (t + 273.15) - 1 / 293.15));
    return (uint16_t)lround(1023.0 * r / (r + 2490.0));
}

static eng_adc_t base_adc(void)
{
    eng_adc_t a;
    memset(&a, 0, sizeof a);
    a.adc[0] = adc_for_mv(2000);        /* MAF 2.0 V */
    a.adc[1] = 300;                     /* TPS */
    a.adc[2] = 300;
    a.adc[3] = adc_for_temp(90);        /* CLT warm */
    a.adc[4] = adc_for_temp(20);        /* IAT */
    a.adc[5] = adc_for_mv(2500);        /* wideband 2.5 V */
    a.adc[6] = adc_for_mv(450);
    a.adc[7] = adc_for_mv(800);
    a.adc[8] = adc_for_mv(14000.0 / 4.0);   /* 14 V through a /4 divider */
    return a;
}

static void test_sensors(void)
{
    printf("sensors\n");
    defaults();
    eng_adc_t a = base_adc();
    eng_sensors_t s;

    a.adc[1] = 510;                     /* (510-100)/(920-100) = 50 % */
    eng_sensors(&a, &s);
    CHECK(s.tps_x10 == 500 && s.faults == 0);
    a.adc[1] = 60;  eng_sensors(&a, &s); CHECK(s.tps_x10 == 0);
    a.adc[1] = 990; eng_sensors(&a, &s); CHECK(s.tps_x10 == 1000);
    a.adc[1] = 3;   eng_sensors(&a, &s); CHECK((s.faults & FLT_TPS1) && s.tps_x10 == 0);

    /* two throttle sensors (one per ITB bank) */
    set8(tune_page0, TU_TPS_COUNT_OFF, 1);
    a.adc[1] = 346; a.adc[2] = 510;     /* 30 % and 50 %: disagree -> lower one */
    eng_sensors(&a, &s);
    CHECK((s.faults & FLT_TPS_DIFF) && s.tps_x10 == 300);
    a.adc[1] = 500; a.adc[2] = 520;     /* 48.8 % and 51.2 % -> average */
    eng_sensors(&a, &s);
    CHECK(!(s.faults & FLT_TPS_DIFF) && NEAR(s.tps_x10, 500, 1));
    set8(tune_page0, TU_TPS_COUNT_OFF, 0);

    /* thermistor across the range */
    double worst = 0;
    for (int t = -30; t <= 120; t += 5) {
        a.adc[3] = adc_for_temp(t);
        eng_sensors(&a, &s);
        double e = fabs(s.clt_x10 / 10.0 - t);
        if (e > worst) worst = e;
    }
    printf("  thermistor -30..120 C: worst error %.1f C\n", worst);
    CHECK(worst < 2.0);
    a.adc[3] = 1020; eng_sensors(&a, &s);
    CHECK((s.faults & FLT_CLT) && s.clt_x10 == ENG_CLT_FALLBACK_X10);
    a.adc[3] = adc_for_temp(90);

    eng_sensors(&a, &s);
    CHECK(NEAR(s.vbat_c, 1400, 5));
    CHECK(NEAR(s.maf_c, 1400, 10));         /* 2.0 V -> 14.0 g/s on the default curve */
    CHECK(NEAR(s.afr_c, 1500, 3));          /* 2.5 V -> 15.0 AFR (10 + 10 x 0.5) */
    CHECK(NEAR(s.nbo2a_mv, 450, 3) && NEAR(s.nbo2b_mv, 800, 3));
    CHECK(eng_temp_axis(-500) == 0 && eng_temp_axis(200) == 60);
}

/* Run the calculation a few times (state settles). */
static void calc(eng_calc_t *e, const eng_adc_t *a, uint16_t rpm, int steps)
{
    eng_sensors_t s;
    eng_sensors(a, &s);
    for (int i = 0; i < steps; i++) {
        if (i % 10 == 0) eng_calc_slow(e, &s);      /* as the firmware: every 100 ms */
        eng_calc(e, &s, rpm, 1);
    }
}

static void test_air_and_fuel(void)
{
    printf("air mass and fuel\n");
    defaults();
    eng_calc_t e;
    eng_adc_t a = base_adc();

    /* MAF: 14 g/s at 3000 rpm = 14 x 20000 / 3000 = 93.3 mg per cylinder */
    eng_calc_init(&e);
    calc(&e, &a, 3000, 3);
    printf("  MAF 14 g/s at 3000 rpm: air %.1f mg (expect 93.3)\n", e.air_d / 10.0);
    CHECK(NEAR(e.air_d, 933, 3) && e.blend == 100);

    /* Alpha-N: VE 90 %, 20 C, 101 kPa: 0.9 x 415.67 cc x rho */
    set8(tune_page0, TU_LOAD_SOURCE_OFF, 1);
    fill8(tune_page1, TU_VE_TABLE_OFF, 256, 180);           /* 90 % */
    fill8(tune_page1, TU_AFR_TABLE_OFF, 256, 147);          /* 14.7 */
    eng_calc_init(&e);
    calc(&e, &a, 3000, 3);
    double rho = 101.0 * 1000.0 / (287.05 * 293.15);        /* kg/m3 = mg/cc */
    double air = 0.9 * 2494.0 / 6.0 * rho;
    printf("  Alpha-N VE 90 %%: air %.1f mg (float %.1f)\n", e.air_d / 10.0, air);
    CHECK(NEAR(e.air_d / 10.0, air, air * 0.005));

    /* fuel: air / 14.7 / (250 cc/min x 0.745 g/cc) + dead time at 14 V */
    double flow = 250.0 * 0.745 / 60.0;                     /* mg per ms */
    double dead = e.dead_us;                                /* default curve at ~14 V */
    double pw = (e.air_d / 10.0) / 14.7 / flow * 1000.0;
    printf("  pulse width %u us (float %.0f + %.0f dead time)\n", e.pw_us[0], pw, dead);
    CHECK(NEAR(e.pw_us[0], pw + dead, (pw + dead) * 0.005));
    /* 14 V reads as 13.9-14.0 V after ADC rounding: dead time 700-708 us on the default curve */
    CHECK(NEAR(e.dead_us, 704, 5) && e.warmup == 100 && e.afr_target_c == 1470);

    /* warm-up at 20 C: x1.40 */
    a.adc[3] = adc_for_temp(20);
    eng_calc_init(&e);
    calc(&e, &a, 3000, 3);
    CHECK(NEAR(e.warmup, 140, 1));
    CHECK(NEAR(e.pw_base_us, pw * e.warmup / 100.0, pw * 0.01));
    a.adc[3] = adc_for_temp(90);

    /* blend 50 %: half MAF, half Alpha-N */
    set8(tune_page0, TU_LOAD_SOURCE_OFF, 2);
    fill8(tune_page1, TU_BLEND_TABLE_OFF, 256, 50);
    eng_calc_init(&e);
    calc(&e, &a, 3000, 3);
    CHECK(e.blend == 50 && NEAR(e.air_d, (933 + air * 10) / 2, 15));

    /* MAF trim +10 % */
    set8(tune_page0, TU_LOAD_SOURCE_OFF, 0);
    fill8(tune_page1, TU_MAF_TRIM_OFF, 256, 110);
    eng_calc_init(&e);
    calc(&e, &a, 3000, 3);
    CHECK(NEAR(e.air_d, 933 * 1.10, 4));
    fill8(tune_page1, TU_MAF_TRIM_OFF, 256, 100);

    /* per-cylinder fuel trim: +10 % on cylinder 3 */
    set8(tune_page0, TU_CYL_FUEL_TRIM_OFF + 2, 110);
    eng_calc_init(&e);
    calc(&e, &a, 3000, 3);
    CHECK(NEAR(e.pw_us[2] - e.dead_us, (e.pw_us[0] - e.dead_us) * 1.10, 3));
    set8(tune_page0, TU_CYL_FUEL_TRIM_OFF + 2, 100);
}

static void test_cranking_and_limits(void)
{
    printf("cranking, after-start, rev limiter, acceleration enrichment\n");
    defaults();
    eng_calc_t e;
    eng_adc_t a = base_adc();
    a.adc[3] = adc_for_temp(20);

    /* cranking at 200 rpm: cranking pulse (9.0 ms at 20 C) + dead time, fixed 10 deg */
    eng_calc_init(&e);
    calc(&e, &a, 200, 1);
    CHECK((e.state & ENG_CRANKING) && NEAR(e.pw_us[0], 9000 + e.dead_us, 30) && e.advance[0] == 100);

    /* starts: after-start enrichment 30 %, fading over 15 s */
    calc(&e, &a, 900, 1);
    CHECK(!(e.state & ENG_CRANKING) && (e.state & ENG_AFTERSTART) && e.as == 30);
    calc(&e, &a, 900, 750);                                 /* 7.5 s */
    CHECK(NEAR(e.as, 15, 1));
    calc(&e, &a, 900, 760);
    CHECK(e.as == 0 && !(e.state & ENG_AFTERSTART));

    /* rev limiter 6500 rpm, 200 rpm hysteresis */
    calc(&e, &a, 6600, 1);
    CHECK((e.state & ENG_REVLIMIT) && e.pw_us[0] == 0);
    calc(&e, &a, 6400, 1);
    CHECK((e.state & ENG_REVLIMIT) && e.pw_us[0] == 0);     /* still inside the hysteresis */
    calc(&e, &a, 6200, 1);
    CHECK(!(e.state & ENG_REVLIMIT) && e.pw_us[0] > 0);

    /* not turning: no fuel */
    eng_sensors_t s;
    eng_sensors(&a, &s);
    eng_calc_slow(&e, &s);
    eng_calc(&e, &s, 0, 0);
    CHECK(e.pw_us[0] == 0 && !(e.state & ENG_RUNNING));

    /* acceleration enrichment: throttle 10 % -> 30 % in 10 ms = 2000 %/s -> 50 % (top of curve) */
    eng_calc_init(&e);
    a.adc[1] = 182;                                         /* 10 % */
    calc(&e, &a, 2500, 5);
    CHECK(e.ae == 0);
    a.adc[1] = 346;                                         /* 30 % */
    calc(&e, &a, 2500, 1);
    printf("  throttle snap: %d %%/s -> +%u %% fuel\n", e.tps_dot, e.ae);
    CHECK(NEAR(e.tps_dot, 2000, 15) && e.ae == 50 && (e.state & ENG_AE));
    calc(&e, &a, 2500, 10);                                 /* 100 ms of the 200 ms */
    CHECK(NEAR(e.ae, 25, 3));
    calc(&e, &a, 2500, 15);
    CHECK(e.ae == 0 && !(e.state & ENG_AE));
}

static void test_ignition(void)
{
    printf("ignition, dwell, end of injection, table interpolation\n");
    defaults();
    eng_calc_t e;
    eng_adc_t a = base_adc();
    fill8(tune_page1, TU_IGN_TABLE_OFF, 256, 100);          /* (100 - 40) x 0.5 = 30 deg */
    eng_calc_init(&e);
    calc(&e, &a, 3000, 2);                                  /* 90 C: no coolant trim */
    CHECK(e.adv_x10 == 300 && e.advance[5] == 300);
    a.adc[3] = adc_for_temp(20);                            /* +2 deg at 20 C */
    calc(&e, &a, 3000, 1);
    CHECK(NEAR(e.adv_x10, 320, 2));
    set8(tune_page0, TU_CYL_IGN_TRIM_OFF + 1, 23);          /* cylinder 2: +1.5 deg */
    calc(&e, &a, 3000, 1);
    CHECK(NEAR(e.advance[1] - e.advance[0], 15, 0));
    CHECK(NEAR(e.dwell_us, 2800, 20));                      /* ~14 V */
    CHECK(e.eoi_x10 == 3810);                               /* default 381 deg at low rpm */

    /* bilinear interpolation: z = 10 x (x index) + (y index) */
    defaults();
    for (int y = 0; y < 16; y++)
        for (int x = 0; x < 16; x++)
            tune_page1[TU_IGN_TABLE_OFF + y * 16 + x] = (uint8_t)(40 + 10 * x + y);
    a = base_adc();
    /* rpm 2250 is half way between bins 2000 (x=5) and 2500 (x=6); MAF load 14 g/s at 2250 =
     * 124.4 mg: between 110 (y=2) and 140 (y=3) at 0.48 */
    eng_calc_init(&e);
    calc(&e, &a, 2250, 2);
    double xf = 5.5, yf = 2 + (124.44 - 110) / 30.0;
    double expect = (10 * xf + yf) * 0.5 * 10;              /* deg x10 */
    printf("  interpolated advance %.1f deg (float %.2f)\n", e.adv_x10 / 10.0, expect / 10);
    CHECK(NEAR(e.adv_x10, expect, 1.5));
}

int main(void)
{
    test_sensors();
    test_air_and_fuel();
    test_cranking_and_limits();
    test_ignition();
    printf("engine: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
