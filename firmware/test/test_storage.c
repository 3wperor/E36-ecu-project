/*
 * Host tests for tune storage (A/B slots in EEPROM) with a model EEPROM,
 * including a power cut after every single write of a burn.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stdint.h>

/* ---- EEPROM model ---------------------------------------------------- */
static uint8_t ee[4096];
static int ee_busy;          /* polls until the current write completes */
static long ee_writes;
static long ee_write_limit = -1;   /* power cut: writes after this are lost */

static uint8_t stor_ee_read(uint16_t a) { return ee[a]; }
static void stor_ee_write(uint16_t a, uint8_t v)
{
    if (ee_write_limit >= 0 && ee_writes >= ee_write_limit) { ee_writes++; return; }
    ee[a] = v;
    ee_writes++;
    ee_busy = 2;
}
static uint8_t stor_ee_ready(void)
{
    if (ee_busy) { ee_busy--; return 0; }
    return 1;
}

#include "tune/storage_core.h"

uint8_t tune_page0[TUNE_PAGE0_SIZE];
uint8_t tune_page1[TUNE_PAGE1_SIZE];

static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("  FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static storage_t S;

static void setup(void)
{
    memset(&S, 0, sizeof S);
    S.page[0] = (stor_page_t){ tune_page0, tune_default_page0, TUNE_PAGE0_SIZE, { 0, 256 } };
    S.page[1] = (stor_page_t){ tune_page1, tune_default_page1, TUNE_PAGE1_SIZE, { 512, 2048 } };
}

/* boot: wipe RAM, load from EEPROM */
static void boot(void)
{
    setup();
    memset(tune_page0, 0x5A, sizeof tune_page0);
    memset(tune_page1, 0x5A, sizeof tune_page1);
    storage_load(&S);
}

static long run_burn(void)
{
    long polls = 0;
    while (storage_busy(&S) && polls < 1000000) { storage_poll(&S); polls++; }
    return polls;
}

static void test_blank(void)
{
    printf("blank EEPROM -> defaults\n");
    memset(ee, 0xFF, sizeof ee);
    boot();
    CHECK(S.from_defaults == 3);
    CHECK(memcmp(tune_page0, tune_default_page0, TUNE_PAGE0_SIZE) == 0);
    CHECK(memcmp(tune_page1, tune_default_page1, TUNE_PAGE1_SIZE) == 0);
}

static void test_burn_reload(void)
{
    printf("burn and reload\n");
    memset(ee, 0xFF, sizeof ee);
    boot();
    tune_page1[100] = 77;
    tune_page1[TUNE_PAGE1_SIZE - 1] = 99;
    storage_burn(&S, 1);
    ee_writes = 0;
    long polls = run_burn();
    printf("  page 1 (%u bytes): %ld writes, %ld polls\n", TUNE_PAGE1_SIZE, ee_writes, polls);
    CHECK(!storage_busy(&S) && S.burns == 1);
    boot();
    CHECK(tune_page1[100] == 77 && tune_page1[TUNE_PAGE1_SIZE - 1] == 99);
    CHECK(S.from_defaults == 1);            /* page 0 never burned */
    CHECK(S.active[1] == 0);                /* first burn goes to slot A */

    /* second burn alternates to slot B; third back to A and writes only what changed */
    tune_page1[5] = 1;
    storage_burn(&S, 1);
    run_burn();
    boot();
    CHECK(S.active[1] == 1 && tune_page1[5] == 1 && tune_page1[100] == 77);
    storage_burn(&S, 1);
    ee_writes = 0;
    run_burn();
    printf("  re-burn into the older slot: %ld writes (only changed bytes + header)\n", ee_writes);
    CHECK(ee_writes < 20);
    boot();
    CHECK(S.active[1] == 0 && tune_page1[5] == 1);
}

static void test_power_cut(void)
{
    printf("power cut after every write of a burn\n");
    memset(ee, 0xFF, sizeof ee);
    boot();
    memset(tune_page0, 0x11, TUNE_PAGE0_SIZE);
    storage_burn(&S, 0);
    run_burn();
    boot();
    CHECK(tune_page0[0] == 0x11);
    static uint8_t snapshot[4096];
    memcpy(snapshot, ee, sizeof ee);

    /* new tune, count the writes of a full burn */
    memset(tune_page0, 0x22, TUNE_PAGE0_SIZE);
    storage_burn(&S, 0);
    ee_writes = 0;
    run_burn();
    long total = ee_writes;

    int old_ok = 0, new_ok = 0, bad = 0;
    for (long cut = 0; cut <= total; cut++) {
        memcpy(ee, snapshot, sizeof ee);
        boot();
        memset(tune_page0, 0x22, TUNE_PAGE0_SIZE);
        storage_burn(&S, 0);
        ee_writes = 0;
        ee_write_limit = cut;
        run_burn();
        ee_write_limit = -1;
        boot();
        int all11 = 1, all22 = 1;
        for (unsigned i = 0; i < TUNE_PAGE0_SIZE; i++) {
            if (tune_page0[i] != 0x11) all11 = 0;
            if (tune_page0[i] != 0x22) all22 = 0;
        }
        if (all11 && !(S.from_defaults & 1)) old_ok++;
        else if (all22 && !(S.from_defaults & 1)) new_ok++;
        else bad++;
    }
    printf("  %ld cut points: old tune %d, new tune %d, corrupt/defaults %d\n", total + 1, old_ok, new_ok, bad);
    CHECK(bad == 0);
    CHECK(new_ok == 1 && old_ok == total);     /* new tune valid only once the last write (magic) lands */
}

static void test_corruption(void)
{
    printf("corrupted slot -> previous copy; other layout -> defaults\n");
    memset(ee, 0xFF, sizeof ee);
    boot();
    tune_page1[10] = 1;
    storage_burn(&S, 1);
    run_burn();
    tune_page1[10] = 2;
    storage_burn(&S, 1);
    run_burn();
    boot();
    CHECK(tune_page1[10] == 2);
    uint16_t active = S.page[1].slot_addr[S.active[1]];
    ee[active + STOR_HDR + 500] ^= 0x01;     /* bit flip in the newest copy */
    boot();
    CHECK(tune_page1[10] == 1 && !(S.from_defaults & 2));

    /* a firmware with a different layout must not load this tune */
    for (int k = 0; k < 2; k++) ee[S.page[1].slot_addr[k] + 4] ^= 0xFF;
    boot();
    CHECK(S.from_defaults & 2);
    CHECK(memcmp(tune_page1, tune_default_page1, TUNE_PAGE1_SIZE) == 0);
}

static void test_edit_during_burn(void)
{
    printf("edit during a burn -> burned again\n");
    memset(ee, 0xFF, sizeof ee);
    boot();
    tune_page1[1000] = 3;
    storage_burn(&S, 1);
    for (int i = 0; i < 2000; i++) storage_poll(&S);      /* part way */
    CHECK(storage_busy(&S));
    tune_page1[2] = 4;                                     /* already passed by the burn */
    storage_burn(&S, 1);                                   /* what tune_modified() does */
    run_burn();
    boot();
    CHECK(tune_page1[1000] == 3 && tune_page1[2] == 4);
    CHECK(S.burns == 0);    /* fresh storage_t after boot */
}

int main(void)
{
    test_blank();
    test_burn_reload();
    test_power_cut();
    test_corruption();
    test_edit_during_burn();
    printf("storage: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
