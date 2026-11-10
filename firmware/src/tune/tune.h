/*
 * Tune pages: live copies in RAM (edited by the tuner), stored in EEPROM.
 * Portable declarations; AVR implementation in tune/storage.c.
 */
#ifndef TUNE_H
#define TUNE_H

#include <stdint.h>
#include "generated/tune_def.h"

/* Load all pages from EEPROM (defaults for any page without a valid copy). */
void     tune_init(void);

/* Background EEPROM work; call every main-loop pass. */
void     tune_poll(void);

/* RAM page for the protocol; NULL if the page does not exist. */
uint8_t *tune_page(uint8_t page, uint16_t *size);

/* The tuner changed page bytes (re-burns automatically if a burn of that
 * page is in progress, so the stored copy never misses the edit). */
void     tune_modified(uint8_t page);

/* Copy the page's defaults into RAM (not stored until burned). */
uint8_t  tune_restore_defaults(uint8_t page);

/* Request a burn of the page to EEPROM. Returns 0 for a bad page. */
uint8_t  tune_burn(uint8_t page);

uint8_t  tune_busy(void);               /* a burn is pending or running */
uint8_t  tune_from_defaults(void);      /* bit n: page n is running on defaults */

#endif
