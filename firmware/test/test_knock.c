/* Host tests for knock control logic (knock_core.h). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "knock/knock_core.h"

uint8_t tune_page0[TUNE_PAGE0_SIZE];
uint8_t tune_page1[TUNE_PAGE1_SIZE];

static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("  FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static void defaults(void)
{
    memcpy(tune_page0, tune_default_page0, TUNE_PAGE0_SIZE);
    memcpy(tune_page1, tune_default_page1, TUNE_PAGE1_SIZE);
}

static void test_positions(void)
{
    printf("angle -> tooth\n");
    CHECK(knock_angle_to_pos(0) == 0);
    CHECK(knock_angle_to_pos(59) == 1 && knock_angle_to_pos(60) == 1);   /* first tooth at or after */
    CHECK(knock_angle_to_pos(3450) == 60);      /* 345 deg = tooth 58 (missing) -> next rev tooth 0 */
    CHECK(knock_angle_to_pos(3570) == 60);      /* 357 deg: tooth 59.5 -> 60 */
    CHECK(knock_angle_to_pos(7140) == 0);       /* 714 deg = rev 1 tooth 59 (missing) -> tooth 0 */
    CHECK(knock_angle_to_pos(7199) == 0);
}

static void test_windows(void)
{
    printf("window table across every TDC offset, start and length\n");
    uint8_t act[120];
    int bad = 0, cases = 0;
    for (unsigned tdc = 0; tdc < 7200; tdc += 70)
        for (unsigned st = 0; st <= 60; st += 10)
            for (unsigned len = 6; len <= 96; len += 15) {
                cases++;
                knock_build_windows(act, (uint16_t)tdc, (uint8_t)st, (uint8_t)len);
                int opens = 0, closes = 0, open_pos[6], close_pos[6];
                for (int i = 0; i < 120; i++) {
                    if (act[i] & KNOCK_OPEN) { opens++; open_pos[act[i] & 7] = i; }
                    if (act[i] & KNOCK_CLOSE) { closes++; close_pos[act[i] & 7] = i; }
                    if ((act[i] & KNOCK_OPEN) && (act[i] & KNOCK_CLOSE)) bad++;
                }
                if (opens != 6 || closes != 6) { bad++; continue; }
                for (int c = 0; c < 6; c++) {
                    /* window length in teeth (positions; missing teeth count as positions) */
                    int w = (close_pos[c] - open_pos[c] + 120) % 120;
                    if (w < (int)len / 6 - 2 || w > (int)len / 6 + 3) bad++;
                    /* gap to the next firing cylinder's open: >= 2 teeth (read + switch sensor) */
                    int next = knock_next_cyl[c];
                    int g = (open_pos[next] - close_pos[c] + 120) % 120;
                    if (g < 2) bad++;
                }
            }
    printf("  %d layouts checked, %d problems\n", cases, bad);
    CHECK(bad == 0);
    /* sensors alternate with the firing order */
    for (int c = 0; c < 6; c++) CHECK(knock_channel(c) != knock_channel(knock_next_cyl[c]));
}

static void feed(knock_t *k, int cyl, int n, int base, int spread, uint8_t active, uint16_t rpm)
{
    for (int i = 0; i < n; i++) knock_sample(k, (uint8_t)cyl, (uint16_t)(base + rand() % (2 * spread + 1) - spread), active, rpm);
}

static void test_detection(void)
{
    printf("detection, retard, recovery\n");
    defaults();
    knock_t k;
    knock_reset(&k);
    srand(1);
    for (int c = 0; c < 6; c++) feed(&k, c, 40, 100, 10, 1, 3000);
    CHECK(k.noise16[2] / 16 > 90 && k.noise16[2] / 16 < 110);
    for (int c = 0; c < 6; c++) CHECK(k.retard[c] == 0);

    /* threshold at 3000 rpm is 2.2 x noise (~220). 300: knock, one step (2.0 deg) */
    knock_sample(&k, 2, 300, 1, 3000);
    CHECK(k.retard[2] == 20 && k.count == 1);
    for (int c = 0; c < 6; c++) if (c != 2) CHECK(k.retard[c] == 0);
    /* heavy (> 1.5 x threshold): two steps */
    knock_sample(&k, 2, 400, 1, 3000);
    CHECK(k.retard[2] == 60 && k.count == 2);
    /* capped at the maximum (10 deg) */
    for (int i = 0; i < 10; i++) knock_sample(&k, 2, 500, 1, 3000);
    CHECK(k.retard[2] == 100);
    /* knocking windows don't raise the noise */
    CHECK(k.noise16[2] / 16 < 110);
    /* below threshold: no knock */
    knock_sample(&k, 4, 200, 1, 3000);
    CHECK(k.retard[4] == 0);
    /* not active (e.g. light load): no retard */
    knock_sample(&k, 5, 500, 0, 3000);
    CHECK(k.retard[5] == 0);

    /* recovery 1.0 deg/s: 10 deg -> 9 deg after 1 s */
    knock_tick(&k, 1, 1);
    CHECK(k.state & KNOCK_ST_KNOCK);
    for (int i = 0; i < 99; i++) knock_tick(&k, 1, 1);
    printf("  after 1 s: cylinder 3 retard %.1f deg\n", k.retard[2] / 10.0);
    CHECK(k.retard[2] == 90);
    for (int i = 0; i < 900; i++) knock_tick(&k, 1, 1);
    CHECK(k.retard[2] == 0 && !(k.state & KNOCK_ST_KNOCK));

    /* disabled: everything cleared */
    knock_sample(&k, 1, 600, 1, 3000);
    CHECK(k.retard[1] > 0);
    knock_tick(&k, 0, 0);
    CHECK(k.retard[1] == 0 && !(k.state & KNOCK_ST_ENABLED));
}

static void test_fault_and_curve(void)
{
    printf("chip fault, threshold curve, result parsing\n");
    defaults();
    knock_t k;
    knock_reset(&k);
    for (int i = 0; i < 7; i++) knock_sample(&k, 0, 0, 1, 3000);
    CHECK(!k.fault);
    knock_sample(&k, 0, 0, 1, 3000);
    knock_tick(&k, 1, 1);
    CHECK(k.fault && (k.state & KNOCK_ST_FAULT));
    for (int c = 0; c < 6; c++) CHECK(knock_retard(&k, c) == 30);    /* 3.0 deg everywhere */
    for (int i = 0; i < 8; i++) knock_sample(&k, 0, 1023, 1, 3000);   /* stuck at full scale: still fault */
    CHECK(k.fault);
    knock_sample(&k, 0, 120, 1, 3000);
    CHECK(!k.fault && knock_retard(&k, 3) == 0);

    CHECK(knock_threshold_x10(500) == 25);
    CHECK(knock_threshold_x10(1500) == 25);
    CHECK(knock_threshold_x10(2500) == 23 || knock_threshold_x10(2500) == 24);   /* half way 25 -> 22 */
    CHECK(knock_threshold_x10(8000) == 20);

    CHECK(knock_parse(0x34, 0x80, 0) == 0x234);     /* datasheet: D7..D0, then D9..D8 in bits 7..6 */
    CHECK(knock_parse(0xFF, 0xC0, 0) == 1023);
    CHECK(knock_parse(0x34, 0x02, 1) == 0x234);     /* D7..D0, then D9..D8 in bits 1..0 */
    CHECK(knock_parse(0xAB, 0xC0, 2) == 687);       /* D9..D2, then D1..D0 in bits 7..6 */
}

int main(void)
{
    test_positions();
    test_windows();
    test_detection();
    test_fault_and_curve();
    printf("knock: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
