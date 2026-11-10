#include "engine/aux.h"

#include <util/atomic.h>

uint8_t aux_tach_map[AUX_TACH_TEETH];
static uint8_t tach_mode;
static uint8_t iac_on, iac_duty, iac_freq;

void aux_init(void)
{
    /* Timer2: phase-correct PWM, TOP 0xFF, outputs connected by aux_iac() */
    TCCR2A = (1 << WGM20);
    TCCR2B = 0;
    OCR2A = 0;
    OCR2B = 0;
}

void aux_iac(uint8_t duty, uint8_t freq_sel, uint8_t enable)
{
    if (!enable) {
        if (iac_on) {
            TCCR2A = (1 << WGM20);                       /* disconnect: pins back to PORT (low) */
            TCCR2B = 0;
            iac_on = 0;
        }
        return;
    }
    if (duty > 100) duty = 100;
    if (iac_on && duty == iac_duty && freq_sel == iac_freq) return;   /* nothing changed */
    iac_duty = duty;
    iac_freq = freq_sel;
    uint8_t ocr = (uint8_t)(((uint16_t)duty * 255u + 50u) / 100u);
    OCR2A = ocr;                                         /* opening coil: on for `duty` */
    OCR2B = ocr;                                         /* closing coil: inverted output */
    uint8_t cs = freq_sel ? ((1 << CS22) | (1 << CS20)) /* /128: 245 Hz */
                          : ((1 << CS22) | (1 << CS21));/* /256: 122 Hz */
    TCCR2A = (1 << WGM20) | (1 << COM2A1) | (1 << COM2B1) | (1 << COM2B0);
    TCCR2B = cs;
    iac_on = 1;
}

void aux_vanos(uint8_t on)
{
    if (on) AUX_PORT |= (1 << AUX_VANOS);
    else AUX_PORT &= (uint8_t)~(1 << AUX_VANOS);
}

void aux_heaters(uint8_t on)
{
    if (on) {
        AUX_PORT |= (1 << AUX_NB_HEAT1);
        AUX_PORT |= (1 << AUX_NB_HEAT2);
    } else {
        AUX_PORT &= (uint8_t)~(1 << AUX_NB_HEAT1);
        AUX_PORT &= (uint8_t)~(1 << AUX_NB_HEAT2);
    }
}

void aux_tach_mode(uint8_t mode, uint8_t synced)
{
    if (!synced || !mode || mode > 3) {
        if (tach_mode != 0) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                for (uint8_t i = 0; i < AUX_TACH_TEETH; i++) aux_tach_map[i] = 0;
            }
            tach_mode = 0;
        }
        AUX_PORT &= (uint8_t)~(1 << AUX_TACH);
        return;
    }
    if (mode == tach_mode) return;
    /* toggle every 60 deg (3 pulses/rev), 90 deg (2/rev) or 180 deg (1/rev);
     * teeth 0..57 are 6 deg apart, so every 10th / 15th / 30th tooth */
    static const uint8_t step[4] = { 0, 10, 15, 30 };
    /* entries change one byte at a time (atomic); a revolution with a mix of
     * old and new entries only gives one odd tach period */
    uint8_t next = 0;
    for (uint8_t t = 0; t < AUX_TACH_TEETH; t++) {
        uint8_t v = 0;
        if (t == next) { v = (uint8_t)(1u << AUX_TACH); next = (uint8_t)(next + step[mode]); }
        aux_tach_map[t] = v;
    }
    tach_mode = mode;
}
