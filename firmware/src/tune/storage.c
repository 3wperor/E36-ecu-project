#include "tune/tune.h"

#include <avr/eeprom.h>

uint8_t tune_page0[TUNE_PAGE0_SIZE];
uint8_t tune_page1[TUNE_PAGE1_SIZE];

/* avr-libc: eeprom_read_byte waits for a write in progress, so the core only
 * reads after stor_ee_ready(); eeprom_write_byte starts a write and returns. */
static inline uint8_t stor_ee_read(uint16_t a) { return eeprom_read_byte((const uint8_t *)a); }
static inline void stor_ee_write(uint16_t a, uint8_t v) { eeprom_write_byte((uint8_t *)a, v); }
static inline uint8_t stor_ee_ready(void) { return eeprom_is_ready() ? 1 : 0; }

#include "tune/storage_core.h"

/* EEPROM map (4 KB): page 0 slots at 0 and 256, page 1 slots at 512 and 2048. */
#define P0_A 0u
#define P0_B 256u
#define P1_A 512u
#define P1_B 2048u
_Static_assert(STOR_HDR + TUNE_PAGE0_SIZE <= P0_B - P0_A, "page 0 slot too small");
_Static_assert(STOR_HDR + TUNE_PAGE1_SIZE <= P1_B - P1_A, "page 1 slot too small");
_Static_assert(P1_B + STOR_HDR + TUNE_PAGE1_SIZE <= 4096u, "EEPROM full");
_Static_assert(TUNE_PAGES == 2, "update the EEPROM map");

static storage_t st;

void tune_init(void)
{
    st.page[0] = (stor_page_t){ tune_page0, tune_default_page0, TUNE_PAGE0_SIZE, { P0_A, P0_B } };
    st.page[1] = (stor_page_t){ tune_page1, tune_default_page1, TUNE_PAGE1_SIZE, { P1_A, P1_B } };
    storage_load(&st);
}

void tune_poll(void)
{
    storage_poll(&st);
}

uint8_t *tune_page(uint8_t page, uint16_t *size)
{
    if (page >= TUNE_PAGES) return 0;
    *size = st.page[page].size;
    return st.page[page].ram;
}

void tune_modified(uint8_t page)
{
    if (page < TUNE_PAGES && st.state != ST_IDLE && st.burn_page == page) {
        storage_burn(&st, page);
    }
}

uint8_t tune_restore_defaults(uint8_t page)
{
    if (page >= TUNE_PAGES) return 0;
    storage_load_defaults(&st, page);
    tune_modified(page);
    return 1;
}

uint8_t tune_burn(uint8_t page)
{
    if (page >= TUNE_PAGES) return 0;
    storage_burn(&st, page);
    return 1;
}

uint8_t tune_busy(void)
{
    return storage_busy(&st);
}

uint8_t tune_from_defaults(void)
{
    return st.from_defaults;
}
