#include "engine/engine.h"
#include "engine/engine_core.h"

#include "board.h"
#include "decoder/decoder_core.h"
#include "hal/adc.h"
#include "knock/knock.h"
#include "engine/aux.h"
#include "engine/control_core.h"
#include "status.h"
#include "tune/tune.h"

static eng_calc_t calc;
static ctl_t ctl;
static eng_sensors_t sens;
static uint8_t slow_div;            /* every 10th update runs the slow part */
static uint16_t pump_hold;          /* 10 ms ticks the pump stays on after the engine stops */

void engine_init(void)
{
    eng_calc_init(&calc);
    ctl_init(&ctl);
    aux_init();
}

void engine_update(const decoder_status_t *ds, sched_input_t *out)
{
    eng_adc_t a;
    for (uint8_t ch = 0; ch < 10; ch++) a.adc[ch] = adc_get(ch);
    eng_sensors_fast(&a, &sens);
    if (slow_div == 0) {
        slow_div = 10;
        eng_sensors_slow(&a, &sens);
        eng_calc_slow(&calc, &sens);
    }
    slow_div--;
    const eng_sensors_t *s = &sens;

    uint8_t synced = ds->sync >= SYNC_CRANK;
    eng_calc(&calc, s, synced ? ds->rpm : 0, synced);

    /* scheduler input */
    out->enable = (uint8_t)((tu_ignEnable() ? SCHED_EN_IGN : 0) | (tu_injEnable() ? SCHED_EN_INJ : 0));
    out->tdc_offset = tu_tdcOffset();
    out->dwell_us = calc.dwell_us;
    out->inj_end = calc.eoi_x10;
    uint16_t air_mg = (uint16_t)(calc.air_d / 10u);      /* one 32-bit division, used twice */
    knock_update(calc.rpm, air_mg, s->clt_x10, ds->sync == SYNC_FULL,
                 (calc.state & ENG_CRANKING) != 0);

    /* Phase 9a controls: overrun cut, lambda trims, idle, VANOS, heaters */
    ctl_in_t ci = {
        .rpm = calc.rpm, .tps_x10 = s->tps_x10, .clt_x10 = s->clt_x10,
        .air_mg = air_mg, .afr_target_c = calc.afr_target_c, .afr_c = s->afr_c,
        .nb_a_mv = s->nbo2a_mv, .nb_b_mv = s->nbo2b_mv,
        .running = (calc.state & ENG_RUNNING) != 0, .cranking = (calc.state & ENG_CRANKING) != 0,
        .accel = (calc.state & ENG_AE) != 0, .revcut = (calc.state & ENG_REVLIMIT) != 0,
    };
    ctl_update(&ctl, &ci);
    /* bank trims as Q10 factors: (1000 + trim) / 1000 x 1024 */
    uint16_t fa = (uint16_t)(((uint32_t)(1000 + ctl.trim_a) * 1049u) >> 10);
    uint16_t fb = (uint16_t)(((uint32_t)(1000 + ctl.trim_b) * 1049u) >> 10);
    for (uint8_t c = 0; c < 6; c++) {
        uint16_t pw = calc.pw_us[c];
        uint16_t f = c < 3 ? fa : fb;
        if (ctl.flags & CTL_DFCO) {
            pw = 0;
        } else if (pw > calc.dead_us && f != 1024u) {
            uint32_t open = (uint32_t)(pw - calc.dead_us) * f >> 10;
            pw = (uint16_t)(open + calc.dead_us > 60000u ? 60000u : open + calc.dead_us);
        }
        calc.pw_us[c] = pw;
    }
    aux_iac(ctl.iac_duty, tu_iacFreq(), tu_iacEnable());
    aux_vanos((ctl.flags & CTL_VANOS) != 0);
    aux_heaters((ctl.flags & CTL_HEATERS) != 0);
    aux_tach_mode(tu_tachMode(), ds->sync >= SYNC_CRANK);

    for (uint8_t c = 0; c < 6; c++) {
        out->advance[c] = (int16_t)(calc.advance[c] - knock_get_retard(c) + ctl.idle_ign_x10);
        out->pw_us[c] = calc.pw_us[c];
    }

    /* fuel pump: prime at power-up, on while the engine turns, 1 s after */
    uint8_t turning = ds->sync != SYNC_NONE;
    if (turning) pump_hold = 100;
    else if (pump_hold) pump_hold--;
    uint8_t pump = turning || pump_hold || rt.uptime_ms < (uint32_t)tu_fuelPumpPrime() * 1000u;
    if (pump) AUX_PORT |= (1 << AUX_FUEL_PUMP);
    else AUX_PORT &= (uint8_t)~(1 << AUX_FUEL_PUMP);

    /* realtime */
    rt.engine = (uint8_t)(calc.state | (pump ? ENG_PUMP : 0) | (tune_busy() ? ENG_BURNING : 0));
    rt.faults = (uint8_t)(s->faults | (tune_from_defaults() ? FLT_DEFAULTS : 0));
    rt.tps = s->tps_x10;
    rt.tpsDot = calc.tps_dot;
    rt.clt = s->clt_x10;
    rt.iat = s->iat_x10;
    rt.vbat = s->vbat_c;
    rt.mafVolts = s->maf_mv;
    rt.mafGs = s->maf_c;
    rt.afr = s->afr_c;
    rt.afrTarget = calc.afr_target_c;
    rt.nbo2a = s->nbo2a_mv;
    rt.nbo2b = s->nbo2b_mv;
    rt.air = calc.air_d;
    rt.ve = calc.ve_x10;
    rt.blend = calc.blend;
    rt.warmupCor = calc.warmup;
    rt.aeCor = calc.ae;
    rt.asCor = calc.as;
    rt.pw = calc.pw_us[0];
    rt.deadTime = calc.dead_us;
    rt.duty = calc.duty;
    rt.advance = calc.adv_x10;
    rt.dwellUs = calc.dwell_us;
    rt.engine2 = ctl.flags;
    rt.lambdaTrimA = ctl.trim_a;
    rt.lambdaTrimB = ctl.trim_b;
    rt.iacDuty = ctl.iac_duty;
    rt.idleRpmTarget = ctl.idle_target;
    rt.idleIgn = ctl.idle_ign_x10;
    rt.eoi = calc.eoi_x10;
}
