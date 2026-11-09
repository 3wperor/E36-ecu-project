/*
 * AVR register hooks for sched_core.h. Channel -> hardware (board.h):
 *
 *   ch 0..2  INJ 1..3  Timer1 OC1A/B/C      ch 6..8   IGN 1..3  Timer4 OC4A/B/C
 *   ch 3..5  INJ 4..6  Timer3 OC3A/B/C      ch 9..11  IGN 4..6  Timer5 OC5A/B/C
 *
 * COM bits per channel in TCCRnA: A = bits 7:6, B = 5:4, C = 3:2.
 *   10 = clear pin on compare (idle / OFF edge), 11 = set pin on compare (ON edge).
 * Timers run in Normal mode, synchronised, so TCNT5 is "now" for all of them.
 * Only ISRs (or code with interrupts off) may call these: TCCRnA is shared by
 * three channels and 16-bit registers share the TEMP byte.
 */
#ifndef SCHED_HW_AVR_H
#define SCHED_HW_AVR_H

#include <avr/io.h>
#include <stdint.h>

#define SCHED_COM_CLEAR_ALL 0xA8u   /* COMnA = COMnB = COMnC = 10, WGM bits 0 */

static inline __attribute__((always_inline)) uint16_t sched_hw_now(void)
{
    return TCNT5;
}

#define HW_ARM_(TCCRA, OCR, TIFR, TIMSK, BIT, SH)          \
    do {                                                   \
        TCCRA = (uint8_t)(TCCRA | (3u << (SH)));           \
        OCR = when;                                        \
        TIFR = (uint8_t)(1u << (BIT));                     \
        TIMSK = (uint8_t)(TIMSK | (1u << (BIT)));          \
    } while (0)

#define HW_END_(TCCRA, OCR, SH)                                                  \
    do {                                                                         \
        TCCRA = (uint8_t)((TCCRA & (uint8_t)~(3u << (SH))) | (2u << (SH)));      \
        OCR = when;                                                              \
    } while (0)

#define HW_IDLE_(TCCRA, TIMSK, BIT, SH)                                          \
    do {                                                                         \
        TCCRA = (uint8_t)((TCCRA & (uint8_t)~(3u << (SH))) | (2u << (SH)));      \
        TIMSK = (uint8_t)(TIMSK & (uint8_t)~(1u << (BIT)));                      \
    } while (0)

/* Expand OP for channel n: OP(timer number, channel letter suffix...) */
#define SCHED_HW_SWITCH(n, A1, B1, C1, A3, B3, C3, A4, B4, C4, A5, B5, C5) \
    switch (n) {                                                            \
    case 0:  A1; break;  case 1:  B1; break;  case 2:  C1; break;          \
    case 3:  A3; break;  case 4:  B3; break;  case 5:  C3; break;          \
    case 6:  A4; break;  case 7:  B4; break;  case 8:  C4; break;          \
    case 9:  A5; break;  case 10: B5; break;  default: C5; break;          \
    }

/* Register table for channels only known at run time (crank ISR loops):
 * compact code, few registers. Constant channels (OC ISRs) use the switch,
 * which the compiler folds to direct instructions. */
typedef struct {
    volatile uint8_t *tccra;
    volatile uint8_t *ocrl;         /* OCRnx low byte; high byte is ocrl[1] */
    volatile uint8_t *timsk;
    volatile uint8_t *tifr;
    uint8_t com_mask;               /* 11 << shift: both COM bits = "set" */
    uint8_t com_clear;              /* 10 << shift */
    uint8_t bit;                    /* OCIEnx / OCFnx bit mask */
} sched_hw_ch_t;

extern const sched_hw_ch_t sched_hw_tab[12];

#define SCHED_HW_TAB_INIT                                                                            \
    { &TCCR1A, &OCR1AL, &TIMSK1, &TIFR1, 0xC0, 0x80, 1 << OCIE1A },                                  \
    { &TCCR1A, &OCR1BL, &TIMSK1, &TIFR1, 0x30, 0x20, 1 << OCIE1B },                                  \
    { &TCCR1A, &OCR1CL, &TIMSK1, &TIFR1, 0x0C, 0x08, 1 << OCIE1C },                                  \
    { &TCCR3A, &OCR3AL, &TIMSK3, &TIFR3, 0xC0, 0x80, 1 << OCIE3A },                                  \
    { &TCCR3A, &OCR3BL, &TIMSK3, &TIFR3, 0x30, 0x20, 1 << OCIE3B },                                  \
    { &TCCR3A, &OCR3CL, &TIMSK3, &TIFR3, 0x0C, 0x08, 1 << OCIE3C },                                  \
    { &TCCR4A, &OCR4AL, &TIMSK4, &TIFR4, 0xC0, 0x80, 1 << OCIE4A },                                  \
    { &TCCR4A, &OCR4BL, &TIMSK4, &TIFR4, 0x30, 0x20, 1 << OCIE4B },                                  \
    { &TCCR4A, &OCR4CL, &TIMSK4, &TIFR4, 0x0C, 0x08, 1 << OCIE4C },                                  \
    { &TCCR5A, &OCR5AL, &TIMSK5, &TIFR5, 0xC0, 0x80, 1 << OCIE5A },                                  \
    { &TCCR5A, &OCR5BL, &TIMSK5, &TIFR5, 0x30, 0x20, 1 << OCIE5B },                                  \
    { &TCCR5A, &OCR5CL, &TIMSK5, &TIFR5, 0x0C, 0x08, 1 << OCIE5C }

static inline __attribute__((always_inline)) void sched_hw_write_ocr(const sched_hw_ch_t *h, uint16_t when)
{
    h->ocrl[1] = (uint8_t)(when >> 8);      /* high byte first (TEMP register) */
    h->ocrl[0] = (uint8_t)when;
}

static inline __attribute__((always_inline)) void sched_hw_arm(uint8_t n, uint16_t when)
{
    if (!__builtin_constant_p(n)) {
        const sched_hw_ch_t *h = &sched_hw_tab[n];
        *h->tccra = (uint8_t)(*h->tccra | h->com_mask);
        sched_hw_write_ocr(h, when);
        *h->tifr = h->bit;
        *h->timsk = (uint8_t)(*h->timsk | h->bit);
        return;
    }
    SCHED_HW_SWITCH(n,
        HW_ARM_(TCCR1A, OCR1A, TIFR1, TIMSK1, OCIE1A, 6), HW_ARM_(TCCR1A, OCR1B, TIFR1, TIMSK1, OCIE1B, 4),
        HW_ARM_(TCCR1A, OCR1C, TIFR1, TIMSK1, OCIE1C, 2), HW_ARM_(TCCR3A, OCR3A, TIFR3, TIMSK3, OCIE3A, 6),
        HW_ARM_(TCCR3A, OCR3B, TIFR3, TIMSK3, OCIE3B, 4), HW_ARM_(TCCR3A, OCR3C, TIFR3, TIMSK3, OCIE3C, 2),
        HW_ARM_(TCCR4A, OCR4A, TIFR4, TIMSK4, OCIE4A, 6), HW_ARM_(TCCR4A, OCR4B, TIFR4, TIMSK4, OCIE4B, 4),
        HW_ARM_(TCCR4A, OCR4C, TIFR4, TIMSK4, OCIE4C, 2), HW_ARM_(TCCR5A, OCR5A, TIFR5, TIMSK5, OCIE5A, 6),
        HW_ARM_(TCCR5A, OCR5B, TIFR5, TIMSK5, OCIE5B, 4), HW_ARM_(TCCR5A, OCR5C, TIFR5, TIMSK5, OCIE5C, 2))
}

static inline __attribute__((always_inline)) void sched_hw_end(uint8_t n, uint16_t when)
{
    if (!__builtin_constant_p(n)) {
        const sched_hw_ch_t *h = &sched_hw_tab[n];
        *h->tccra = (uint8_t)((*h->tccra & (uint8_t)~h->com_mask) | h->com_clear);
        sched_hw_write_ocr(h, when);
        return;
    }
    SCHED_HW_SWITCH(n,
        HW_END_(TCCR1A, OCR1A, 6), HW_END_(TCCR1A, OCR1B, 4), HW_END_(TCCR1A, OCR1C, 2),
        HW_END_(TCCR3A, OCR3A, 6), HW_END_(TCCR3A, OCR3B, 4), HW_END_(TCCR3A, OCR3C, 2),
        HW_END_(TCCR4A, OCR4A, 6), HW_END_(TCCR4A, OCR4B, 4), HW_END_(TCCR4A, OCR4C, 2),
        HW_END_(TCCR5A, OCR5A, 6), HW_END_(TCCR5A, OCR5B, 4), HW_END_(TCCR5A, OCR5C, 2))
}

static inline __attribute__((always_inline)) void sched_hw_idle(uint8_t n)
{
    if (!__builtin_constant_p(n)) {
        const sched_hw_ch_t *h = &sched_hw_tab[n];
        *h->tccra = (uint8_t)((*h->tccra & (uint8_t)~h->com_mask) | h->com_clear);
        *h->timsk = (uint8_t)(*h->timsk & (uint8_t)~h->bit);
        return;
    }
    SCHED_HW_SWITCH(n,
        HW_IDLE_(TCCR1A, TIMSK1, OCIE1A, 6), HW_IDLE_(TCCR1A, TIMSK1, OCIE1B, 4), HW_IDLE_(TCCR1A, TIMSK1, OCIE1C, 2),
        HW_IDLE_(TCCR3A, TIMSK3, OCIE3A, 6), HW_IDLE_(TCCR3A, TIMSK3, OCIE3B, 4), HW_IDLE_(TCCR3A, TIMSK3, OCIE3C, 2),
        HW_IDLE_(TCCR4A, TIMSK4, OCIE4A, 6), HW_IDLE_(TCCR4A, TIMSK4, OCIE4B, 4), HW_IDLE_(TCCR4A, TIMSK4, OCIE4C, 2),
        HW_IDLE_(TCCR5A, TIMSK5, OCIE5A, 6), HW_IDLE_(TCCR5A, TIMSK5, OCIE5B, 4), HW_IDLE_(TCCR5A, TIMSK5, OCIE5C, 2))
}

#include "sched/sched_core.h"

/* The one scheduler instance (defined in sched.c, used by the crank ISR). */
extern sched_t sched;

#endif
