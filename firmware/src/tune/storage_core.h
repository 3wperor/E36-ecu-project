/*
 * Tune storage in EEPROM — core logic (portable, header-only).
 *
 * Each tune page has two slots (A/B). A slot is
 *     [magic u16][seq u16][layout hash u32][crc32 u32][data ...]
 * and is valid only if the magic, the layout hash (same firmware layout) and
 * the CRC all match. At boot the valid slot with the newest sequence number
 * wins; with no valid slot the page falls back to the defaults in flash.
 *
 * A burn writes the INACTIVE slot, in this order:
 *     1. invalidate its magic        (a half-written slot can never look valid)
 *     2. data bytes                  (only bytes that differ are written)
 *     3. layout hash, CRC, sequence
 *     4. magic                       (the slot becomes valid only now)
 * so a power cut at any point leaves the previous tune in force.
 *
 * EEPROM writes take ~3.4 ms per byte, so a burn runs in the background:
 * storage_poll() does at most one write (and a bounded number of reads) per
 * call. If the page is edited during a burn, it is burned again afterwards.
 *
 * The includer defines:
 *   uint8_t stor_ee_read(uint16_t addr);
 *   void    stor_ee_write(uint16_t addr, uint8_t v);   starts a write
 *   uint8_t stor_ee_ready(void);                        1 when a write may start
 */
#ifndef STORAGE_CORE_H
#define STORAGE_CORE_H

#include <stdint.h>
#include "comms/crc32.h"
#include "generated/tune_def.h"

#define STOR_MAGIC        0xE36Eu
#define STOR_HDR          12u
#define STOR_SCAN_PER_POLL 32u     /* data bytes compared per poll when nothing needs writing */

typedef struct {
    uint8_t  *ram;                  /* the live page */
    const uint8_t *defaults;        /* flash */
    uint16_t size;
    uint16_t slot_addr[2];          /* EEPROM address of slot A and B */
} stor_page_t;

enum { ST_IDLE = 0, ST_INVALIDATE, ST_DATA, ST_TRAILER, ST_MAGIC };

typedef struct {
    stor_page_t page[TUNE_PAGES];
    uint8_t  active[TUNE_PAGES];    /* slot holding the current tune, 0/1, 0xFF = none */
    uint16_t seq[TUNE_PAGES];
    uint8_t  from_defaults;         /* bit n: page n was loaded from defaults */
    /* burn state */
    uint8_t  state;
    uint8_t  burn_page;
    uint8_t  pending;               /* bit n: page n needs (another) burn */
    uint8_t  target;
    uint16_t pos;
    uint32_t crc;
    uint8_t  trailer[8];            /* hash + crc + seq, written in order */
    uint16_t burns;                 /* completed burns (wraps) */
} storage_t;

static inline uint32_t stor_read32(uint16_t a)
{
    return (uint32_t)stor_ee_read(a) | ((uint32_t)stor_ee_read(a + 1) << 8) |
           ((uint32_t)stor_ee_read(a + 2) << 16) | ((uint32_t)stor_ee_read(a + 3) << 24);
}

static inline uint16_t stor_read16(uint16_t a)
{
    return (uint16_t)(stor_ee_read(a) | (stor_ee_read(a + 1) << 8));
}

/* 1 if the slot is valid; *seq gets its sequence number. */
static inline uint8_t stor_slot_valid(const stor_page_t *p, uint8_t slot, uint16_t *seq)
{
    uint16_t a = p->slot_addr[slot];
    if (stor_read16(a) != STOR_MAGIC) return 0;
    if (stor_read32(a + 4) != TUNE_LAYOUT_HASH) return 0;
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint16_t i = 0; i < p->size; i++) {
        uint8_t b = stor_ee_read((uint16_t)(a + STOR_HDR + i));
        crc = crc32_update(crc, &b, 1);
    }
    if (~crc != stor_read32(a + 8)) return 0;
    *seq = stor_read16(a + 2);
    return 1;
}

/* Load every page: newest valid slot, else defaults. */
static inline void storage_load(storage_t *s)
{
    s->from_defaults = 0;
    for (uint8_t n = 0; n < TUNE_PAGES; n++) {
        stor_page_t *p = &s->page[n];
        uint16_t sa = 0, sb = 0;
        uint8_t va = stor_slot_valid(p, 0, &sa);
        uint8_t vb = stor_slot_valid(p, 1, &sb);
        uint8_t use;
        if (va && vb) use = ((int16_t)(sb - sa) > 0) ? 1 : 0;
        else if (va) use = 0;
        else if (vb) use = 1;
        else use = 0xFF;

        if (use == 0xFF) {
            for (uint16_t i = 0; i < p->size; i++) p->ram[i] = tune_flash_byte(&p->defaults[i]);
            s->active[n] = 0xFF;
            s->seq[n] = 0;
            s->from_defaults |= (uint8_t)(1u << n);
        } else {
            uint16_t a = (uint16_t)(p->slot_addr[use] + STOR_HDR);
            for (uint16_t i = 0; i < p->size; i++) p->ram[i] = stor_ee_read((uint16_t)(a + i));
            s->active[n] = use;
            s->seq[n] = use ? sb : sa;
        }
    }
}

/* Restore the defaults into RAM (not burned until storage_burn()). */
static inline void storage_load_defaults(storage_t *s, uint8_t n)
{
    stor_page_t *p = &s->page[n];
    for (uint16_t i = 0; i < p->size; i++) p->ram[i] = tune_flash_byte(&p->defaults[i]);
}

/* Request a burn of page n (also call after edits made during a burn). */
static inline void storage_burn(storage_t *s, uint8_t n)
{
    if (n < TUNE_PAGES) s->pending |= (uint8_t)(1u << n);
}

static inline uint8_t storage_busy(const storage_t *s)
{
    return s->state != ST_IDLE || s->pending;
}

/* Background work; call every main-loop pass. */
static inline void storage_poll(storage_t *s)
{
    if (s->state == ST_IDLE) {
        if (!s->pending) return;
        uint8_t n = 0;
        while (!(s->pending & (1u << n))) n++;
        s->pending &= (uint8_t)~(1u << n);
        s->burn_page = n;
        s->target = (s->active[n] == 0) ? 1 : 0;
        s->state = ST_INVALIDATE;
    }
    if (!stor_ee_ready()) return;

    stor_page_t *p = &s->page[s->burn_page];
    uint16_t base = p->slot_addr[s->target];

    switch (s->state) {
    case ST_INVALIDATE:
        if (stor_ee_read(base) != 0xFF) {
            stor_ee_write(base, 0xFF);
        }
        s->pos = 0;
        s->crc = 0xFFFFFFFFUL;
        s->state = ST_DATA;
        break;

    case ST_DATA: {
        uint8_t scanned = 0;
        while (s->pos < p->size && scanned < STOR_SCAN_PER_POLL) {
            uint16_t a = (uint16_t)(base + STOR_HDR + s->pos);
            uint8_t b = p->ram[s->pos];
            s->crc = crc32_update(s->crc, &b, 1);
            s->pos++;
            scanned++;
            if (stor_ee_read(a) != b) {
                stor_ee_write(a, b);
                return;                 /* one write per poll */
            }
        }
        if (s->pos >= p->size) {
            uint32_t h = TUNE_LAYOUT_HASH, c = ~s->crc;
            uint16_t seq = (uint16_t)(s->seq[s->burn_page] + 1u);
            for (uint8_t k = 0; k < 4; k++) {
                s->trailer[k] = (uint8_t)(h >> (8 * k));
                s->trailer[4 + k] = (uint8_t)(c >> (8 * k));
            }
            s->pos = 0;
            s->seq[s->burn_page] = seq;  /* provisional; slot not valid until magic */
            s->state = ST_TRAILER;
        }
        break;
    }

    case ST_TRAILER:
        /* bytes 2..11 of the header: seq (2), hash (4), crc (4) */
        while (s->pos < 10) {
            uint16_t seq = s->seq[s->burn_page];
            uint8_t b = s->pos < 2 ? (uint8_t)(seq >> (8 * s->pos)) : s->trailer[s->pos - 2];
            uint16_t a = (uint16_t)(base + 2 + s->pos);
            s->pos++;
            if (stor_ee_read(a) != b) {
                stor_ee_write(a, b);
                return;
            }
        }
        s->pos = 0;
        s->state = ST_MAGIC;
        break;

    case ST_MAGIC:
        if (s->pos == 0) {              /* high byte first, low byte (0x6E) last */
            s->pos = 1;
            if (stor_ee_read(base + 1) != (uint8_t)(STOR_MAGIC >> 8)) {
                stor_ee_write(base + 1, (uint8_t)(STOR_MAGIC >> 8));
                return;
            }
        }
        if (stor_ee_read(base) != (uint8_t)STOR_MAGIC) {
            stor_ee_write(base, (uint8_t)STOR_MAGIC);
        }
        s->active[s->burn_page] = s->target;
        s->from_defaults &= (uint8_t)~(1u << s->burn_page);
        s->burns++;
        s->state = ST_IDLE;
        break;

    default:
        s->state = ST_IDLE;
        break;
    }
}

#endif
