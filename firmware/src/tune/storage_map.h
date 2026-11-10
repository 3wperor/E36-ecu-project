/*
 * EEPROM map (4 KB): two slots (A/B) per tune page, each
 * [12-byte header][page data]. Shared by the firmware and the host tests.
 */
#ifndef STORAGE_MAP_H
#define STORAGE_MAP_H

#include "generated/tune_def.h"

#define STOR_P0_A 0u
#define STOR_P0_B 384u
#define STOR_P1_A 768u
#define STOR_P1_B 2304u
#define STOR_EEPROM_SIZE 4096u

_Static_assert(12u + TUNE_PAGE0_SIZE <= STOR_P0_B - STOR_P0_A, "page 0 slot too small: move the EEPROM map");
_Static_assert(STOR_P0_B + 12u + TUNE_PAGE0_SIZE <= STOR_P1_A, "page 0 slot B overlaps page 1");
_Static_assert(12u + TUNE_PAGE1_SIZE <= STOR_P1_B - STOR_P1_A, "page 1 slot too small: move the EEPROM map");
_Static_assert(STOR_P1_B + 12u + TUNE_PAGE1_SIZE <= STOR_EEPROM_SIZE, "EEPROM full");
_Static_assert(TUNE_PAGES == 2, "update the EEPROM map");

#endif
