/*
 * Knock control — AVR side: TPIC8101 knock chip on SPI, INT/HOLD window pin
 * driven from the crank ISR. Logic in knock_core.h.
 */
#ifndef KNOCK_H
#define KNOCK_H

#include <stdint.h>

/* SPI and pins; call once at start-up. */
void knock_init(void);

/* Every main-loop pass: collect the chip's result for the last window. */
void knock_poll(void);

/* 100 Hz, from the engine calculation: configuration, window table,
 * enable conditions, recovery, realtime data. */
void knock_update(uint16_t rpm, uint16_t air_mg, int16_t clt_x10, uint8_t full_sync, uint8_t cranking);

/* Timing to remove from cylinder c (0..5), 0.1 deg. */
uint8_t knock_get_retard(uint8_t c);

/* ---- crank ISR hooks (decoder.c) ---------------------------------------- */
extern uint8_t knock_act[120];
extern volatile uint8_t knock_pending;
void knock_tooth_events(uint8_t act);
void knock_stop(void);

/* Every tooth while in full (720 deg) sync; pos = tooth + 60 x rev. */
static inline __attribute__((always_inline)) void knock_tooth(uint8_t pos)
{
    uint8_t a = knock_act[pos];
    if (a | knock_pending) {
        knock_tooth_events(a);
    }
}

#endif
