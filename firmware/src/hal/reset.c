#include "hal/reset.h"
#include <avr/io.h>
#include <avr/wdt.h>

uint8_t reset_cause __attribute__((section(".noinit")));

/* Runs before main() and before .data/.bss are initialised. After a watchdog
 * reset the watchdog stays enabled with its shortest timeout, so it must be
 * turned off immediately or the chip resets forever. */
void reset_capture(void) __attribute__((naked, used, section(".init3")));
void reset_capture(void)
{
    reset_cause = MCUSR;
    MCUSR = 0;
    wdt_disable();
}
