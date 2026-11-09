#include "sched/sched.h"
#include "sched/sched_hw_avr.h"

#include <avr/interrupt.h>
#include <string.h>
#include <util/atomic.h>

sched_t sched;
const sched_hw_ch_t sched_hw_tab[12] = { SCHED_HW_TAB_INIT };

/* Power-up default: everything OFF. The numbers are bench-test defaults. */
static sched_cfg_t cfg = {
    .enable = 0,
    .tdc_offset = 0,
    .advance = 100,         /* 10° BTDC */
    .dwell_us = 3000,
    .inj_pw_us = 2500,
    .inj_end = 3800,        /* 380° before compression TDC: closed intake valve */
};

/* ---- output-compare ISRs: one per channel, constant channel number so the
 * register accesses compile to direct instructions. ---------------------- */

#define OC_ISR(vect, n, OCR) ISR(vect) { sched_oc(&sched, (n), OCR); }

OC_ISR(TIMER1_COMPA_vect, 0, OCR1A)
OC_ISR(TIMER1_COMPB_vect, 1, OCR1B)
OC_ISR(TIMER1_COMPC_vect, 2, OCR1C)
OC_ISR(TIMER3_COMPA_vect, 3, OCR3A)
OC_ISR(TIMER3_COMPB_vect, 4, OCR3B)
OC_ISR(TIMER3_COMPC_vect, 5, OCR3C)
OC_ISR(TIMER4_COMPA_vect, 6, OCR4A)
OC_ISR(TIMER4_COMPB_vect, 7, OCR4B)
OC_ISR(TIMER4_COMPC_vect, 8, OCR4C)
OC_ISR(TIMER5_COMPA_vect, 9, OCR5A)
OC_ISR(TIMER5_COMPB_vect, 10, OCR5B)
OC_ISR(TIMER5_COMPC_vect, 11, OCR5C)

void sched_init_hw(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sched_init(&sched);
        /* Hand the pins to the compare units in "clear" mode. The internal
         * output latches are 0 after reset, so the pins stay low. */
        TCCR1A = SCHED_COM_CLEAR_ALL;
        TCCR3A = SCHED_COM_CLEAR_ALL;
        TCCR4A = SCHED_COM_CLEAR_ALL;
        TCCR5A = SCHED_COM_CLEAR_ALL;
    }
}

uint8_t sched_set_cfg(const uint8_t *raw12)
{
    sched_cfg_t c;
    memcpy(&c, raw12, sizeof c);
    if (!sched_cfg_valid(&c)) {
        return 0;
    }
    cfg = c;
    return 1;
}

void sched_get_cfg(uint8_t *raw12)
{
    memcpy(raw12, &cfg, sizeof cfg);
}

void sched_update(uint8_t sync, uint16_t period4, uint32_t rev_ticks)
{
    sched_slot_t slot[SCHED_CHANNELS];
    sched_compute(&cfg, sync, period4, rev_ticks, slot);   /* divisions: outside atomic */

    uint8_t synced = sync >= SYNC_CRANK;
    for (uint8_t n = 0; n < SCHED_CHANNELS; n++) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {            /* one short block per channel */
            sched_post(&sched, n, &slot[n], synced);
        }
    }
}

void sched_get_stats(sched_stats_t *st)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        st->late = sched.late;
        st->overlap = sched.overlap;
    }
    st->enable = cfg.enable;
}
