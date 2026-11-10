/*
 * Auxiliary outputs: idle valve PWM (Timer2, complementary on OC2A / OC2B),
 * VANOS, narrowband heaters, tach signal for the instrument cluster.
 */
#ifndef AUX_H
#define AUX_H

#include <stdint.h>
#include <avr/io.h>
#include "board.h"

void aux_init(void);

/* Idle valve: duty 0..100 % on the opening coil, the closing coil gets the
 * complement. freq_sel 0 = 122 Hz, 1 = 245 Hz. enable 0 = both coils off. */
void aux_iac(uint8_t duty, uint8_t freq_sel, uint8_t enable);

void aux_vanos(uint8_t on);
void aux_heaters(uint8_t on);

/* Tach: mode 0 off, 1 = 3 pulses/rev, 2 = 2/rev, 3 = 1/rev. Not synced:
 * output held low. */
void aux_tach_mode(uint8_t mode, uint8_t synced);

/* crank ISR hook: one byte per tooth (0..57 within the revolution), either
 * 0 or the tach pin's mask. Writing 1 to a PINx bit toggles that pin in one
 * instruction (so it never races the main loop's port writes) and writing 0
 * does nothing, so the hook is a load and a store with no branch. */
#define AUX_TACH_TEETH 58
extern uint8_t aux_tach_map[AUX_TACH_TEETH];
static inline __attribute__((always_inline)) void aux_tach_tooth(uint8_t tooth)
{
    PINA = aux_tach_map[tooth];
}

#endif
