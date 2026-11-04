/*
 * Time bases.
 *
 *  - Timers 1, 3, 4, 5: free-running 16-bit counters at F_CPU/64 = 250 kHz
 *    (4 us per tick, wraps every 262.144 ms). They are started on the same
 *    prescaler edge, so TCNT1 == TCNT3 == TCNT4 == TCNT5 at all times. A
 *    timestamp captured on one timer (crank ICR5) can therefore schedule an
 *    output compare on any other.
 *  - Timer0: 1 kHz system tick for the cooperative main loop.
 *
 * AVR gotcha: all 16-bit timer registers share one TEMP byte. Only touch
 * them from ISRs, or with interrupts disabled.
 */
#ifndef HAL_TIMERS_H
#define HAL_TIMERS_H

#include <stdint.h>
#include <avr/io.h>

#define TICKS_PER_MS   250u
#define US_PER_TICK    4u

void timers_init(void);

/* Milliseconds since start-up (atomic read). */
uint32_t timers_millis(void);

/* Returns 1 if the four 16-bit counters agree to within one tick. */
uint8_t timers_check_sync(void);

/* Current 16-bit tick count. Call from an ISR or with interrupts disabled. */
static inline uint16_t timers_now_isr(void) { return TCNT5; }

#endif
