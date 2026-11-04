#ifndef HAL_RESET_H
#define HAL_RESET_H

#include <stdint.h>

/* MCUSR value captured at start-up (bit 0 power-on, 1 external, 2 brown-out,
 * 3 watchdog, 4 JTAG). Watchdog resets in the field show up here. */
extern uint8_t reset_cause;

#endif
