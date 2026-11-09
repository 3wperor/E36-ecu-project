#include "decoder/decoder.h"

/* Rare paths as real functions: keeps the crank ISR's register save small. */
#define DEC_COLD   static __attribute__((noinline))
#define SCHED_COLD static __attribute__((noinline))

#include "decoder/decoder_core.h"
#include "sched/sched_hw_avr.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

/* Only the ISRs modify this outside atomic blocks. */
static decoder_t dec;

ISR(TIMER5_CAPT_vect)
{
    uint16_t icr = ICR5;
    uint8_t ev = dec_crank_edge(&dec, icr);
    sched_tooth(&sched, &dec, ev, icr);
    if (ev & EV_SYNC_LOST) {
        sched_cancel_pending(&sched);
    }
}

ISR(TIMER4_CAPT_vect)
{
    dec_cam_edge(&dec);
}

void decoder_init(uint8_t cam_enabled)
{
    dec_init(&dec, cam_enabled);

    /* Rising edges, hardware noise canceller (4 samples), keep the clock
     * select bits that timers_init() set. */
    TCCR5B |= (1 << ICNC5) | (1 << ICES5);
    TCCR4B |= (1 << ICNC4) | (1 << ICES4);
    TIFR5 = (1 << ICF5);
    TIFR4 = (1 << ICF4);
    TIMSK5 |= (1 << ICIE5);
    TIMSK4 |= (1 << ICIE4);
}

void decoder_poll(decoder_status_t *out)
{
    uint32_t rev_ticks;
    uint16_t last_interval;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (dec_check_stall(&dec, TCNT5)) {
            sched_cancel_pending(&sched);
        }
        out->sync = dec.sync;
        out->rev = dec.rev;
        out->cam_pos = dec.cam_pos;
        out->loss_reason = dec.loss_reason;
        out->sync_losses = dec.sync_losses;
        out->noise_teeth = dec.noise_teeth;
        out->cam_errors = dec.cam_errors;
        rev_ticks = dec.rev_ticks;
        last_interval = dec.has_interval ? dec.tooth_period : 0;
        out->period4 = dec.has_interval ? dec.period4 : 0;
    }
    out->tooth_period = last_interval;
    out->rev_ticks = (out->sync >= SYNC_CRANK) ? rev_ticks : 0;

    /* Divisions outside the atomic block (~40 us each on AVR). */
    out->rpm = (out->sync >= SYNC_CRANK) ? dec_rpm(rev_ticks) : 0;
    out->rpm_inst = dec_rpm_tooth(last_interval);
}

uint8_t decoder_take_log(uint8_t *out)
{
    uint8_t count;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        dec.log_active = 0;         /* stop the ISR writing while we copy */
        count = dec.log_count;
    }
    uint8_t *p = out;
    for (uint8_t i = 0; i < count; i++) {
        const tooth_log_t *e = &dec.log[i];
        *p++ = (uint8_t)e->interval;
        *p++ = (uint8_t)(e->interval >> 8);
        *p++ = e->pos;
        *p++ = e->flags;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        dec_log_start(&dec);
    }
    return count;
}
