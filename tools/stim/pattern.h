/*
 * M52 trigger pattern: 60-2 crank wheel + configurable cam signal.
 * Portable (used by the AVR firmware and by the host test).
 *
 * The 720° engine cycle is split into 240 half-tooth steps of 3°:
 *   even step = tooth high, odd step = tooth low.
 * Tooth index 0..59 per revolution; index 0 is the first tooth after the
 * gap, indices 58 and 59 are the missing teeth (output stays low).
 *
 * The real M52 cam pattern is confirmed on the car in Phase 0, so both
 * likely shapes are provided:
 *   CAM_PULSE     one tooth-width pulse per cycle
 *   CAM_HALFMOON  high for 360° of crank, low for the other 360°
 * starting at half-step `cam_start`.
 */
#ifndef STIM_PATTERN_H
#define STIM_PATTERN_H

#include <stdint.h>

#define STIM_STEPS        240u
#define STIM_TEETH        60u
#define STIM_MISSING      2u

#define STIM_OUT_CRANK    0x01
#define STIM_OUT_CAM      0x02

enum { CAM_NONE = 0, CAM_PULSE = 1, CAM_HALFMOON = 2 };

static inline uint8_t stim_pattern(uint8_t step, uint8_t cam_mode, uint8_t cam_start)
{
    uint8_t out = 0;
    uint8_t tooth = (uint8_t)(step >> 1);
    if (tooth >= STIM_TEETH) {
        tooth = (uint8_t)(tooth - STIM_TEETH);
    }
    if (!(step & 1u) && tooth < STIM_TEETH - STIM_MISSING) {
        out |= STIM_OUT_CRANK;
    }

    int16_t rel = (int16_t)step - (int16_t)cam_start;   /* no modulo: ISR-cheap */
    if (rel < 0) {
        rel += STIM_STEPS;
    }
    if ((cam_mode == CAM_PULSE && rel < 2) || (cam_mode == CAM_HALFMOON && rel < (int16_t)(STIM_STEPS / 2))) {
        out |= STIM_OUT_CAM;
    }
    return out;
}

/* Timer ticks (0.5 us) per half-tooth step: 3° at rpm = 0.5/rpm s. */
#define STIM_TICKS_PER_STEP(rpm) (1000000UL / (rpm))

#endif
