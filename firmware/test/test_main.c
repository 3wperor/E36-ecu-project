/*
 * Host-side unit tests for portable firmware modules.
 * Build and run: make test
 */
#include <stdio.h>
#include <string.h>

#include "comms/commands.h"
#include "comms/crc32.h"
#include "comms/frame.h"
#include "config.h"
#include "decoder/decoder.h"
#include "sched/sched.h"
#include "tune/tune.h"
#include "status.h"

realtime_t rt;

/* Host stubs for the AVR-only output config: accept advance <= 60.0°. */
static uint8_t stub_cfg[SCHED_CFG_SIZE];
uint8_t sched_set_cfg(const uint8_t *raw12)
{
    int16_t adv = (int16_t)(raw12[4] | (raw12[5] << 8));
    if (adv > 600) return 0;
    memcpy(stub_cfg, raw12, SCHED_CFG_SIZE);
    return 1;
}
void sched_get_cfg(uint8_t *raw12) { memcpy(raw12, stub_cfg, SCHED_CFG_SIZE); }

/* Host stubs for the AVR-only tune storage: two small pages. */
static uint8_t pg0[10], pg1[300];
static int burned[2], modified[2], restored[2];
uint8_t *tune_page(uint8_t page, uint16_t *size)
{
    if (page == 0) { *size = sizeof pg0; return pg0; }
    if (page == 1) { *size = sizeof pg1; return pg1; }
    return 0;
}
void tune_modified(uint8_t page) { modified[page]++; }
uint8_t tune_burn(uint8_t page) { if (page > 1) return 0; burned[page]++; return 1; }
uint8_t tune_restore_defaults(uint8_t page) { if (page > 1) return 0; restored[page]++; return 1; }

/* Host stub for the AVR-only tooth log: a full capture. */
uint8_t decoder_take_log(uint8_t *out)
{
    for (int i = 0; i < 256; i++) out[i] = (uint8_t)i;
    return 64;
}

static int failures, checks;

#define CHECK(cond) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

/* Build a request frame for payload p into out; returns its length. */
static uint16_t make_frame(uint8_t *out, const void *p, uint16_t len)
{
    memcpy(&out[2], p, len);
    return frame_wrap(out, len);
}

/* Feed bytes; return the last non-NONE result (or NONE). */
static frame_result_t feed(frame_rx_t *rx, const uint8_t *b, uint16_t n, uint32_t t)
{
    frame_result_t last = FRAME_NONE;
    for (uint16_t i = 0; i < n; i++) {
        frame_result_t r = frame_rx_byte(rx, b[i], t);
        if (r != FRAME_NONE) last = r;
    }
    return last;
}

static void test_crc32(void)
{
    CHECK(crc32((const uint8_t *)"123456789", 9) == 0xCBF43926UL);
    CHECK(crc32((const uint8_t *)"", 0) == 0x00000000UL);
    /* Incremental == one-shot */
    uint32_t c = crc32_update(0xFFFFFFFFUL, (const uint8_t *)"1234", 4);
    c = ~crc32_update(c, (const uint8_t *)"56789", 5);
    CHECK(c == 0xCBF43926UL);
}

static void test_frame_roundtrip(void)
{
    uint8_t buf[FRAME_MAX_PAYLOAD + FRAME_OVERHEAD];
    frame_rx_t rx;
    frame_rx_reset(&rx);

    uint16_t n = make_frame(buf, "Q", 1);
    CHECK(n == 7);
    CHECK(buf[0] == 0 && buf[1] == 1 && buf[2] == 'Q');
    CHECK(feed(&rx, buf, n, 0) == FRAME_OK);
    CHECK(rx.len == 1 && rx.buf[0] == 'Q');

    /* Two frames back to back */
    uint8_t two[32];
    uint16_t a = make_frame(two, "S", 1);
    uint16_t b = make_frame(two + a, "AB", 2);
    int ok = 0;
    for (uint16_t i = 0; i < a + b; i++) {
        if (frame_rx_byte(&rx, two[i], 1) == FRAME_OK) ok++;
    }
    CHECK(ok == 2);
    CHECK(rx.len == 2 && rx.buf[1] == 'B');
}

static void test_frame_errors(void)
{
    uint8_t buf[64];
    frame_rx_t rx;
    frame_rx_reset(&rx);

    /* Corrupted CRC */
    uint16_t n = make_frame(buf, "Q", 1);
    buf[n - 1] ^= 0x01;
    CHECK(feed(&rx, buf, n, 0) == FRAME_BAD_CRC);

    /* Parser recovers for the next frame */
    n = make_frame(buf, "Q", 1);
    CHECK(feed(&rx, buf, n, 0) == FRAME_OK);

    /* Zero and oversize lengths are rejected */
    uint8_t zero[2] = {0, 0};
    CHECK(feed(&rx, zero, 2, 0) == FRAME_BAD_LENGTH);
    uint8_t big[2] = {0xFF, 0xFF};
    CHECK(feed(&rx, big, 2, 0) == FRAME_BAD_LENGTH);

    /* A stale partial frame is dropped after the timeout */
    n = make_frame(buf, "Q", 1);
    feed(&rx, buf, 3, 100);                              /* header + 'Q' only */
    CHECK(feed(&rx, buf, n, 100 + FRAME_TIMEOUT_MS + 1) == FRAME_OK);
}

static void test_commands(void)
{
    uint8_t resp[FRAME_MAX_PAYLOAD];
    uint16_t n;

    n = commands_handle((const uint8_t *)"Q", 1, resp);
    CHECK(resp[0] == FRAME_RC_OK);
    CHECK(n == 1 + strlen(ECU_SIGNATURE));
    CHECK(memcmp(&resp[1], ECU_SIGNATURE, n - 1) == 0);

    n = commands_handle((const uint8_t *)"S", 1, resp);
    CHECK(resp[0] == FRAME_RC_OK && n == 1 + strlen(ECU_VERSION));

    rt.uptime_ms = 0x12345678UL;
    rt.adc[ADC_NUM_CH - 1] = 0x03FF;
    n = commands_handle((const uint8_t *)"A", 1, resp);
    CHECK(resp[0] == FRAME_RC_OK && n == 1 + sizeof(realtime_t));
    CHECK(sizeof(realtime_t) == REALTIME_SIZE);
    CHECK(resp[1] == 0x78 && resp[4] == 0x12);           /* little-endian */

    n = commands_handle((const uint8_t *)"T", 1, resp);
    CHECK(n == 258 && resp[0] == FRAME_RC_OK && resp[1] == 64 && resp[257] == 255);
    CHECK(n <= FRAME_MAX_PAYLOAD);

    uint8_t setp[13] = {'P', 3, 0, 0x10, 0x00, 100, 0, 0xB8, 0x0B, 0xC4, 0x09, 0xD8, 0x0E};
    n = commands_handle(setp, 13, resp);
    CHECK(n == 1 && resp[0] == FRAME_RC_OK);
    n = commands_handle((const uint8_t *)"p", 1, resp);
    CHECK(n == 13 && resp[0] == FRAME_RC_OK && memcmp(&resp[1], &setp[1], 12) == 0);
    setp[6] = 0x10;                                      /* advance 419.6°: invalid */
    n = commands_handle(setp, 13, resp);
    CHECK(n == 1 && resp[0] == FRAME_RC_RANGE_ERR);
    n = commands_handle(setp, 5, resp);                  /* wrong length */
    CHECK(n == 1 && resp[0] == FRAME_RC_RANGE_ERR);

    /* ---- tune pages ---- */
    for (int i = 0; i < 300; i++) pg1[i] = (uint8_t)i;
    uint8_t rd[7] = {'R', 1, 0, 0x10, 0, 5, 0};          /* page 1, offset 16, 5 bytes */
    n = commands_handle(rd, 7, resp);
    CHECK(n == 6 && resp[0] == FRAME_RC_OK && resp[1] == 16 && resp[5] == 20);
    uint8_t rd_big[7] = {'R', 1, 0, 0x28, 0x01, 5, 0};   /* offset 296 + 5 > 300 */
    CHECK(commands_handle(rd_big, 7, resp) == 1 && resp[0] == FRAME_RC_RANGE_ERR);
    uint8_t rd_page[7] = {'R', 2, 0, 0, 0, 1, 0};        /* no page 2 */
    CHECK(commands_handle(rd_page, 7, resp) == 1 && resp[0] == FRAME_RC_RANGE_ERR);
    uint8_t rd_page_hi[7] = {'R', 0, 1, 0, 0, 1, 0};     /* page 256 must not alias page 0 */
    CHECK(commands_handle(rd_page_hi, 7, resp) == 1 && resp[0] == FRAME_RC_RANGE_ERR);
    uint8_t rd_long[7] = {'R', 1, 0, 0, 0, 0x2C, 0x01};  /* 300 bytes > one frame */
    CHECK(commands_handle(rd_long, 7, resp) == 1 && resp[0] == FRAME_RC_RANGE_ERR);

    uint8_t wr[10] = {'M', 0, 0, 2, 0, 3, 0, 0xAA, 0xBB, 0xCC};   /* page 0, offset 2, 3 bytes */
    n = commands_handle(wr, 10, resp);
    CHECK(n == 1 && resp[0] == FRAME_RC_OK);
    CHECK(pg0[1] == 0 && pg0[2] == 0xAA && pg0[4] == 0xCC && pg0[5] == 0 && modified[0] == 1);
    uint8_t wr_bad[10] = {'M', 0, 0, 8, 0, 3, 0, 1, 2, 3};        /* 8 + 3 > 10 */
    CHECK(commands_handle(wr_bad, 10, resp) == 1 && resp[0] == FRAME_RC_RANGE_ERR && pg0[8] == 0);
    CHECK(commands_handle(wr, 9, resp) == 1 && resp[0] == FRAME_RC_RANGE_ERR);   /* data short */

    uint8_t bn[3] = {'B', 1, 0};
    CHECK(commands_handle(bn, 3, resp) == 1 && resp[0] == FRAME_RC_OK && burned[1] == 1);
    bn[1] = 5;
    CHECK(commands_handle(bn, 3, resp) == 1 && resp[0] == FRAME_RC_RANGE_ERR);
    uint8_t df[3] = {'D', 0, 0};
    CHECK(commands_handle(df, 3, resp) == 1 && resp[0] == FRAME_RC_OK && restored[0] == 1);

    rt.rpm = 0x1234;
    uint8_t och[5] = {'O', 32, 0, 2, 0};                  /* rpm is at offset 32 */
    n = commands_handle(och, 5, resp);
    CHECK(n == 3 && resp[0] == FRAME_RC_OK && resp[1] == 0x34 && resp[2] == 0x12);
    uint8_t och_bad[5] = {'O', 0, 0, (uint8_t)(REALTIME_SIZE + 1), 0};
    CHECK(commands_handle(och_bad, 5, resp) == 1 && resp[0] == FRAME_RC_RANGE_ERR);

    n = commands_handle((const uint8_t *)"z", 1, resp);
    CHECK(n == 1 && resp[0] == FRAME_RC_UNKNOWN);
}

int main(void)
{
    test_crc32();
    test_frame_roundtrip();
    test_frame_errors();
    test_commands();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
