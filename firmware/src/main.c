/*
 * E36 M52 ECU — main.
 *
 * Structure: everything time-critical happens in hardware (input capture,
 * output compare) and in short ISRs. main() runs a cooperative loop: comms
 * on every pass, everything else on the 1 kHz tick at fixed rates.
 */
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "board.h"
#include "comms/comms.h"
#include "decoder/decoder.h"
#include "engine/engine.h"
#include "tune/tune.h"
#include "sched/sched.h"
#include "hal/adc.h"
#include "hal/io.h"
#include "hal/reset.h"
#include "hal/timers.h"
#include "hal/uart.h"
#include "status.h"

realtime_t rt;

static uint16_t loop_count;

static void task_1khz(void)
{
    /* Phase 3+: fast sensors (TPS rate, MAF), acceleration enrichment. */
}

static void task_100hz(void)
{
    decoder_status_t ds;
    decoder_poll(&ds);          /* stall detection needs >= 1 call per 60 ms */
    rt.rpm = ds.rpm;
    rt.rpm_inst = ds.rpm_inst;
    rt.sync = ds.sync;
    rt.cam_pos = ds.cam_pos;
    rt.loss_reason = ds.loss_reason;
    rt.sync_losses = ds.sync_losses;
    rt.noise_teeth = ds.noise_teeth;
    rt.cam_errors = ds.cam_errors;

    /* Engine calculation always runs (live data); bench test settings, when
     * enabled, replace its outputs. */
    sched_input_t in;
    engine_update(&ds, &in);
    if (sched_test_active()) {
        sched_test_input(&in);
        rt.engine |= 0x20;      /* ENG_TESTMODE */
    }
    sched_update(ds.sync, ds.period4, ds.rev_ticks, &in);
    sched_stats_t st;
    sched_get_stats(&st);
    rt.out_enable = in.enable;
    (void)st.enable;
    rt.sched_late = st.late;
    rt.sched_overlap = st.overlap;

    /* Phase 9: lambda control, idle, VANOS. */
}

static void task_10hz(void)
{
    for (uint8_t ch = 0; ch < ADC_NUM_CH; ch++) {
        rt.adc[ch] = adc_get(ch);
    }
    rt.uart_overflows = uart_rx_overflows();
}

static void task_1hz(void)
{
    rt.loops_per_sec = loop_count;
    loop_count = 0;
    if (timers_check_sync()) {
        rt.flags |= RT_FLAG_TIMERS_SYNCED;
    } else {
        rt.flags &= (uint8_t)~RT_FLAG_TIMERS_SYNCED;
    }
}

int main(void)
{
    io_init_safe();         /* outputs off before anything else runs */
    timers_init();
    sched_init_hw();        /* compare units own the output pins, all OFF */
    adc_init();
    tune_init();            /* tune from EEPROM (defaults if none) */
    decoder_init(tu_camEnable());
    engine_init();
    uart_init();
    comms_init();

    rt.reset_cause = reset_cause;

    wdt_enable(WDTO_60MS);
    sei();

    uint32_t tick_ms = timers_millis();
    uint8_t div100 = 10, div10 = 10, div1 = 10;

    for (;;) {
        wdt_reset();

        uint32_t now = timers_millis();
        comms_poll(now);
        tune_poll();            /* background EEPROM burn */

        /* Run each missed 1 ms tick once, so rates stay exact even if one
         * pass of the loop was slow. */
        if (tick_ms != now) {
            tick_ms++;
            rt.uptime_ms = tick_ms;
            task_1khz();
            if (--div100 == 0) {
                div100 = 10;
                task_100hz();
                if (--div10 == 0) {
                    div10 = 10;
                    task_10hz();
                    if (--div1 == 0) {
                        div1 = 10;
                        task_1hz();
                    }
                }
            }
        }

        loop_count++;
    }
}
