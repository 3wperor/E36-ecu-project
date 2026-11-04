#include "hal/timers.h"
#include <avr/interrupt.h>
#include <util/atomic.h>

static volatile uint32_t ms_count;

ISR(TIMER0_COMPA_vect)
{
    ms_count++;
}

void timers_init(void)
{
    /* Hold the shared prescaler (Timers 0,1,3,4,5) in reset while configuring. */
    GTCCR = (1 << TSM) | (1 << PSRSYNC);

    /* Timer0: CTC, /64, 250 counts -> 1 kHz. */
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A  = (uint8_t)(TICKS_PER_MS - 1);
    TCNT0  = 0;
    TIMSK0 = (1 << OCIE0A);

    /* 16-bit timers: normal mode, /64, outputs disconnected for now
     * (COMnx = 0, pins under PORT control and held low by io_init_safe). */
    TCCR1A = 0; TCCR1C = 0; TCCR1B = (1 << CS11) | (1 << CS10); TCNT1 = 0;
    TCCR3A = 0; TCCR3C = 0; TCCR3B = (1 << CS31) | (1 << CS30); TCNT3 = 0;
    TCCR4A = 0; TCCR4C = 0; TCCR4B = (1 << CS41) | (1 << CS40); TCNT4 = 0;
    TCCR5A = 0; TCCR5C = 0; TCCR5B = (1 << CS51) | (1 << CS50); TCNT5 = 0;

    /* Release: all counters start on the same prescaler edge. */
    GTCCR = 0;
}

uint32_t timers_millis(void)
{
    uint32_t v;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = ms_count; }
    return v;
}

uint8_t timers_check_sync(void)
{
    uint16_t t1, t3, t4, t5;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        t1 = TCNT1; t3 = TCNT3; t4 = TCNT4; t5 = TCNT5;
    }
    /* Reads are a few CPU cycles apart; one tick is 64 cycles. */
    return (uint16_t)(t3 - t1) <= 1 && (uint16_t)(t4 - t3) <= 1 && (uint16_t)(t5 - t4) <= 1;
}
