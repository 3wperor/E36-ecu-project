/*
 * End-to-end test: run the real firmware ELF in simavr, talk to it over the
 * emulated UART0 and check the replies. Drives simulated ADC voltages and a
 * simulated engine (60-2 crank + half-moon cam on the ICP5/ICP4 pins, made
 * with the trigger simulator's own pattern code), times every ISR, and
 * checks the injector/coil schedule from the output-compare register writes
 * (simavr mis-models OC pins in Normal mode, see firmware/README.md).
 *
 *   make sim-test
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sim_avr.h"
#include "sim_elf.h"
#include "sim_irq.h"
#include "sim_io.h"
#include "avr_adc.h"
#include "avr_ioport.h"
#include "avr_uart.h"

#include "comms/crc32.h"
#include "pattern.h"

#define F_CPU 16000000UL
#define CAM_START 41

static avr_t *avr;
static avr_irq_t *uart_in, *crank_pin, *cam_pin;
static uint8_t rx[1024];
static size_t rx_len;
static int failures, checks;

#define CHECK(cond) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL line %d: %s\n", __LINE__, #cond); } \
} while (0)

/* ---- simulated engine --------------------------------------------------- */

static struct {
    double  rpm;
    double  next;           /* CPU cycle of the next half-step */
    uint8_t step, out;
    double  ramp_from, ramp_to, ramp_t0, ramp_cycles;   /* linear speed ramp */
} eng;

static void engine_set_rpm(double rpm)
{
    if (rpm > 0 && eng.rpm <= 0) {
        eng.next = (double)avr->cycle + 1000;
    }
    eng.rpm = rpm;
    if (rpm <= 0) {
        avr_raise_irq(crank_pin, 0);
        avr_raise_irq(cam_pin, 0);
        eng.out = 0;
    }
}

/* Change speed smoothly, like a real engine (an instant jump from 1000 to
 * 7200 rpm is physically impossible and is rejected by the decoder). */
static void engine_ramp(double to, double seconds)
{
    eng.ramp_from = eng.rpm;
    eng.ramp_to = to;
    eng.ramp_t0 = (double)avr->cycle;
    eng.ramp_cycles = seconds * F_CPU;
}

static void engine_poll(void)
{
    if (eng.rpm <= 0) return;
    if (eng.ramp_cycles > 0) {
        double x = ((double)avr->cycle - eng.ramp_t0) / eng.ramp_cycles;
        if (x >= 1) { x = 1; eng.ramp_cycles = 0; }
        eng.rpm = eng.ramp_from + (eng.ramp_to - eng.ramp_from) * x;
    }
    while ((double)avr->cycle >= eng.next) {
        eng.step = (uint8_t)((eng.step + 1) % STIM_STEPS);
        uint8_t n = stim_pattern(eng.step, CAM_HALFMOON, CAM_START);
        if ((n ^ eng.out) & STIM_OUT_CRANK) avr_raise_irq(crank_pin, n & STIM_OUT_CRANK);
        if ((n ^ eng.out) & STIM_OUT_CAM)   avr_raise_irq(cam_pin, !!(n & STIM_OUT_CAM));
        eng.out = n;
        eng.next += 8e6 / eng.rpm;          /* 3° = 0.5/rpm s = 8e6/rpm cycles */
    }
}

/* ---- ISR timing: every __vector_N, by program-counter range ------------- */

#define NVEC 57
static uint8_t isr_of_pc[0x10000];            /* word address -> vector + 1 */
static int isr_loaded;
static int cur_isr = -1;
static avr_cycle_count_t isr_start;
static uint32_t isr_max[NVEC], isr_count[NVEC];
static uint64_t isr_total[NVEC];

static void isr_reset_stats(void)
{
    memset(isr_max, 0, sizeof isr_max);
    memset(isr_count, 0, sizeof isr_count);
    memset(isr_total, 0, sizeof isr_total);
}

/* An ISR starts when the PC reaches a vector function's first instruction
 * and ends when its return address is popped (stack pointer back above the
 * entry value). Time spent in helpers it calls (e.g. libgcc multiply) is
 * included. */
static uint32_t isr_entry_addr[NVEC];
static uint16_t isr_sp0;

static uint16_t sp(void) { return (uint16_t)(avr->data[0x5D] | (avr->data[0x5E] << 8)); }

static void isr_poll(void)
{
    if (!isr_loaded) return;
    if (cur_isr < 0) {
        int v = (int)isr_of_pc[(avr->pc >> 1) & 0xFFFF] - 1;
        if (v >= 0 && avr->pc == isr_entry_addr[v]) {
            cur_isr = v;
            isr_start = avr->cycle;
            isr_sp0 = sp();
        }
    } else if (sp() > isr_sp0) {
        /* + 5 cycles interrupt response + 3 for the vector-table jump */
        uint32_t c = (uint32_t)(avr->cycle - isr_start) + 8;
        if (c > isr_max[cur_isr]) isr_max[cur_isr] = c;
        isr_total[cur_isr] += c;
        isr_count[cur_isr]++;
        cur_isr = -1;
    }
}

/* ---- output-compare register capture -------------------------------------
 * The firmware arms every edge as: TCCRnA (COM bits = level after the match),
 * then OCRnx (high byte, then low byte). Each low-byte write is therefore one
 * scheduled edge. A refined spark rewrites the OFF edge; the last OFF write
 * before the next ON is the real one. */

static const uint16_t tccra_addr[4] = {0x80, 0x90, 0xA0, 0x120};
static const uint16_t ocr_addr[12] = {0x88, 0x8A, 0x8C, 0x98, 0x9A, 0x9C,
                                      0xA8, 0xAA, 0xAC, 0x128, 0x12A, 0x12C};
static uint8_t tccra_val[4], ocr_hi[12];

#define MAXP 4000
typedef struct { uint16_t on, off; avr_cycle_count_t when; } opulse_t;
static opulse_t op[12][MAXP];
static int nop[12];
static int have_on[12];
static uint16_t cur_on[12], cur_off[12];
static avr_cycle_count_t cur_when[12];
static int capture_on;

static void on_tccra(struct avr_irq_t *irq, uint32_t v, void *p)
{
    (void)irq;
    tccra_val[(intptr_t)p] = (uint8_t)v;
}

static void on_ocr_hi(struct avr_irq_t *irq, uint32_t v, void *p)
{
    (void)irq;
    ocr_hi[(intptr_t)p] = (uint8_t)v;
}

static void on_ocr_lo(struct avr_irq_t *irq, uint32_t v, void *p)
{
    (void)irq;
    int ch = (int)(intptr_t)p;
    uint16_t val = (uint16_t)((ocr_hi[ch] << 8) | (v & 0xFF));
    int shift = 6 - 2 * (ch % 3);
    int level_on = ((tccra_val[ch / 3] >> shift) & 3) == 3;
    /* simavr repeats the notification (same value) when the compare
     * matches; only real firmware writes change the value or level. */
    static uint16_t last_val[12];
    static int last_lvl[12] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    if (val == last_val[ch] && level_on == last_lvl[ch]) return;
    last_val[ch] = val;
    last_lvl[ch] = level_on;
    if (level_on) {
        if (have_on[ch] && capture_on && nop[ch] < MAXP) {
            op[ch][nop[ch]].on = cur_on[ch];
            op[ch][nop[ch]].off = cur_off[ch];
            op[ch][nop[ch]].when = cur_when[ch];
            nop[ch]++;
        }
        have_on[ch] = 1;
        cur_on[ch] = val;
        cur_off[ch] = val;
        cur_when[ch] = avr->cycle;
    } else {
        cur_off[ch] = val;
    }
}

static void capture_init(void)
{
    static char names[40][16];
    int k = 0;
    for (int t = 0; t < 4; t++) {
        snprintf(names[k], 16, "tccra%d", t);
        avr_irq_register_notify(avr_iomem_getirq(avr, tccra_addr[t], names[k++], 8), on_tccra, (void *)(intptr_t)t);
    }
    for (int c = 0; c < 12; c++) {
        snprintf(names[k], 16, "ocrh%d", c);
        avr_irq_register_notify(avr_iomem_getirq(avr, ocr_addr[c] + 1, names[k++], 8), on_ocr_hi, (void *)(intptr_t)c);
        snprintf(names[k], 16, "ocrl%d", c);
        avr_irq_register_notify(avr_iomem_getirq(avr, ocr_addr[c], names[k++], 8), on_ocr_lo, (void *)(intptr_t)c);
    }
}

/* ---- simulation helpers -------------------------------------------------- */

static void on_uart_byte(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq; (void)param;
    if (rx_len < sizeof rx) rx[rx_len++] = (uint8_t)value;
}

static void run_us(uint64_t us)
{
    avr_cycle_count_t end = avr->cycle + us * (F_CPU / 1000000UL);
    while (avr->cycle < end) {
        int state = avr_run(avr);
        if (state == cpu_Done || state == cpu_Crashed) {
            printf("simulated CPU stopped (state %d)\n", state);
            exit(2);
        }
        engine_poll();
        isr_poll();
    }
}

/* Send one framed request, one byte per ~UART byte time. */
static void send_frame(const uint8_t *payload, uint16_t len, int corrupt)
{
    uint8_t f[300];
    f[0] = (uint8_t)(len >> 8); f[1] = (uint8_t)len;
    memcpy(&f[2], payload, len);
    uint32_t c = crc32(payload, len);
    if (corrupt) c ^= 1;
    f[2 + len] = (uint8_t)(c >> 24); f[3 + len] = (uint8_t)(c >> 16);
    f[4 + len] = (uint8_t)(c >> 8);  f[5 + len] = (uint8_t)c;
    for (uint16_t i = 0; i < len + 6u; i++) {
        avr_raise_irq(uart_in, f[i]);
        run_us(100);
    }
}

/* Wait for one complete response frame; returns payload length or -1. */
static int recv_frame(uint8_t *payload, uint32_t timeout_us)
{
    for (uint32_t t = 0; t < timeout_us; t += 100) {
        if (rx_len >= 2) {
            size_t len = ((size_t)rx[0] << 8) | rx[1];
            if (rx_len >= len + 6) {
                uint32_t c = ((uint32_t)rx[2 + len] << 24) | ((uint32_t)rx[3 + len] << 16) |
                             ((uint32_t)rx[4 + len] << 8) | rx[5 + len];
                int ok = crc32(&rx[2], (uint16_t)len) == c;
                memcpy(payload, &rx[2], len);
                memmove(rx, rx + len + 6, rx_len - (len + 6));
                rx_len -= len + 6;
                return ok ? (int)len : -1;
            }
        }
        run_us(100);
    }
    return -1;
}

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t le32(const uint8_t *p) { return (uint32_t)le16(p) | ((uint32_t)le16(p + 2) << 16); }

typedef struct {
    uint32_t uptime;
    uint16_t loops, rpm, rpm_inst, sync_losses, noise, cam_errors, maf, clt;
    uint8_t  reset_cause, flags, sync, cam_pos, loss_reason, out_enable;
    uint16_t late, overlap;
} rt_t;

static int get_rt(rt_t *r)
{
    uint8_t p[300];
    send_frame((const uint8_t *)"A", 1, 0);
    int n = recv_frame(p, 50000);
    if (n != 51 || p[0] != 0) return 0;
    const uint8_t *d = &p[1];
    r->uptime = le32(d);
    r->loops = le16(d + 4);
    r->reset_cause = d[6];
    r->flags = d[7];
    r->maf = le16(d + 8);
    r->clt = le16(d + 8 + 2 * 3);
    r->rpm = le16(d + 32);
    r->rpm_inst = le16(d + 34);
    r->sync = d[36];
    r->cam_pos = d[37];
    r->loss_reason = d[38];
    r->sync_losses = le16(d + 40);
    r->noise = le16(d + 42);
    r->cam_errors = le16(d + 44);
    r->out_enable = d[39];
    r->late = le16(d + 46);
    r->overlap = le16(d + 48);
    return 1;
}

static uint32_t sched_addr;      /* data address of the firmware's `sched` */

static void load_symbols(const char *symfile)
{
    FILE *f = fopen(symfile, "r");
    if (!f) return;
    char line[256];
    while (fgets(line, sizeof line, f)) {
        unsigned a, s, v;
        char type, name[128];
        if (sscanf(line, "%x %x %c %127s", &a, &s, &type, name) != 4) continue;
        if (sscanf(name, "__vector_%u", &v) == 1 && v < NVEC) {
            for (unsigned pc = a; pc < a + s; pc += 2) isr_of_pc[(pc >> 1) & 0xFFFF] = (uint8_t)(v + 1);
            isr_entry_addr[v] = a;
            isr_loaded = 1;
        }
        if (!strcmp(name, "sched")) sched_addr = a & 0xFFFF;
    }
    fclose(f);
}

/* sched_t on AVR (no padding): sched_chan_t is 22 bytes, state first. */
static int all_channels_idle(void)
{
    for (int n = 0; n < 12; n++) {
        if (avr->data[sched_addr + 22 * n] != 0) return 0;
    }
    return 1;
}

static void send_cfg(uint8_t en, uint16_t tdc, int16_t adv, uint16_t dwell_us, uint16_t pw_us, uint16_t eoi)
{
    uint8_t m[13] = {'P', en, 0, (uint8_t)tdc, (uint8_t)(tdc >> 8), (uint8_t)adv, (uint8_t)((uint16_t)adv >> 8),
                     (uint8_t)dwell_us, (uint8_t)(dwell_us >> 8), (uint8_t)pw_us, (uint8_t)(pw_us >> 8),
                     (uint8_t)eoi, (uint8_t)(eoi >> 8)};
    uint8_t p[16];
    send_frame(m, 13, 0);
    int n = recv_frame(p, 50000);
    CHECK(n == 1 && p[0] == 0);
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: %s ecu.elf mcu [ecu.sym]\n", argv[0]); return 2; }
    if (argc > 3) load_symbols(argv[3]);

    elf_firmware_t fw;
    memset(&fw, 0, sizeof fw);
    if (elf_read_firmware(argv[1], &fw)) { fprintf(stderr, "cannot read %s\n", argv[1]); return 2; }
    avr = avr_make_mcu_by_name(argv[2]);
    if (!avr) { fprintf(stderr, "simavr does not know %s\n", argv[2]); return 2; }
    avr_init(avr);
    fw.frequency = F_CPU;
    avr_load_firmware(avr, &fw);
    avr->frequency = F_CPU;
    avr->log = 0;
    avr->avcc = 5000;                   /* simavr defaults to 3.3 V */
    avr->aref = 5000;

    uint32_t flags = 0;
    avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &flags);
    flags &= ~AVR_UART_FLAG_STDIO;
    avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &flags);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT),
                            on_uart_byte, NULL);
    uart_in = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT);
    crank_pin = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('L'), 1);   /* PL1 = ICP5 */
    cam_pin   = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('L'), 0);   /* PL0 = ICP4 */
    capture_init();

    /* Sensor voltages (mV): MAF 2.5 V, CLT 1.25 V. */
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC0), 2500);
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC3), 1250);

    /* ---- 1. boot with the engine stopped ---- */
    run_us(1200000);
    CHECK(rx_len == 0);                 /* nothing unsolicited */

    uint8_t p[300];
    int n;

    send_frame((const uint8_t *)"Q", 1, 0);
    n = recv_frame(p, 50000);
    CHECK(n > 1 && p[0] == 0x00);
    if (n > 1) printf("signature: %.*s\n", n - 1, (const char *)&p[1]);

    send_frame((const uint8_t *)"S", 1, 0);
    n = recv_frame(p, 50000);
    CHECK(n > 1 && p[0] == 0x00);
    if (n > 1) printf("version:   %.*s\n", n - 1, (const char *)&p[1]);

    rt_t r;
    CHECK(get_rt(&r));
    printf("stopped:   uptime %u ms, main loop %u passes/s, reset cause 0x%02X, flags 0x%02X, "
           "MAF adc %u, CLT adc %u, rpm %u, sync %u\n",
           r.uptime, r.loops, r.reset_cause, r.flags, r.maf, r.clt, r.rpm, r.sync);
    CHECK(r.uptime >= 1200 && r.uptime < 1400);  /* no watchdog reset */
    CHECK(r.flags & 0x01);                       /* timers in sync */
    CHECK(r.maf >= 510 && r.maf <= 513);
    CHECK(r.clt >= 254 && r.clt <= 257);
    CHECK(r.rpm == 0 && r.sync == 0);
    uint16_t idle_loops = r.loops;

    send_frame((const uint8_t *)"A", 1, 1);      /* corrupted CRC */
    n = recv_frame(p, 50000);
    CHECK(n == 1 && p[0] == 0x82);

    send_frame((const uint8_t *)"?", 1, 0);      /* unknown command */
    n = recv_frame(p, 50000);
    CHECK(n == 1 && p[0] == 0x83);

    /* ---- 2. engine at 1000 rpm ---- */
    engine_set_rpm(1000);
    run_us(400000);                              /* ~6.7 revolutions */
    CHECK(get_rt(&r));
    printf("1000 rpm:  rpm %u (tooth %u), sync %u, cam at tooth %u, losses %u, noise %u, cam errors %u\n",
           r.rpm, r.rpm_inst, r.sync, r.cam_pos, r.sync_losses, r.noise, r.cam_errors);
    CHECK(r.sync == 3);
    CHECK(r.rpm >= 999 && r.rpm <= 1001);
    CHECK(r.sync_losses == 0 && r.noise == 0 && r.cam_errors == 0);
    CHECK(r.cam_pos == 20 || r.cam_pos == 80);   /* cam edge at half-step 41 = tooth 20 */

    /* ---- 3. outputs on: 6 injectors + 6 coils, sequential ---- */
    /* 10° advance, 3 ms dwell, 2.5 ms pulse, injection ends 380° BTDC */
    send_cfg(3, 600, 100, 3000, 2500, 3800);
    CHECK(get_rt(&r));
    CHECK(r.out_enable == 3);

    /* ---- 4. full-throttle rev to 7200 rpm, timing every ISR ---- */
    engine_ramp(7200, 0.6);
    run_us(700000);
    CHECK(get_rt(&r));
    uint16_t late0 = r.late, overlap0 = r.overlap;
    isr_reset_stats();
    memset(nop, 0, sizeof nop);
    capture_on = 1;
    run_us(1100000);                             /* > 1 s so loops/s reflects load */
    capture_on = 0;
    avr_cycle_count_t window = 1100000ULL * 16;
    CHECK(get_rt(&r));
    printf("7200 rpm:  rpm %u (tooth %u), sync %u, losses %u, late edges %u (during the rev-up %u), overlaps %u, "
           "main loop %u passes/s (idle %u)\n", r.rpm, r.rpm_inst, r.sync, r.sync_losses,
           (uint16_t)(r.late - late0), late0, (uint16_t)(r.overlap - overlap0), r.loops, idle_loops);
    CHECK(r.sync == 3);
    CHECK(r.rpm >= 7195 && r.rpm <= 7205);
    CHECK(r.sync_losses == 0 && r.noise == 0 && r.cam_errors == 0);
    CHECK(r.late == late0 && r.overlap == overlap0);   /* none at steady speed */
    CHECK(overlap0 == 0);

    if (isr_loaded) {
        static const int oc_vec[12] = {17, 18, 19, 32, 33, 34, 42, 43, 44, 47, 48, 49};
        uint64_t all = 0, oc_total = 0;
        uint32_t oc_count = 0, oc_max = 0;
        for (int v = 0; v < NVEC; v++) all += isr_total[v];
        for (int k = 0; k < 12; k++) {
            oc_total += isr_total[oc_vec[k]];
            oc_count += isr_count[oc_vec[k]];
            if (isr_max[oc_vec[k]] > oc_max) oc_max = isr_max[oc_vec[k]];
        }
        printf("crank ISR: %u calls, average %.2f us (limit 25), worst %.2f us (limit 60)\n", isr_count[46],
               (double)isr_total[46] / isr_count[46] / 16.0, isr_max[46] / 16.0);
        printf("OC ISRs:   %u calls, average %.2f us, worst %.2f us\n", oc_count,
               (double)oc_total / oc_count / 16.0, oc_max / 16.0);
        printf("interrupt load at 7200 rpm, all 12 outputs: %.1f %% of the CPU\n", 100.0 * (double)all / (double)window);
        CHECK(isr_count[46] > 7000);
        CHECK(isr_total[46] <= 400ULL * isr_count[46]);   /* average 25 us: regression guard, PLAN.md 3.6 */
        CHECK(isr_max[46] <= 960);                         /* worst 60 us */
        CHECK(oc_count > 2 * 12 * 55);       /* 2 edges x 12 channels x 60 cycles/s */
        CHECK(oc_max <= 160);                /* 10 us */
        CHECK(100.0 * (double)all / (double)window < 25.0);
    } else {
        printf("ISR timing: symbols not found\n");
        CHECK(0);
    }

    /* Schedule, from the register writes (ticks of 4 us). */
    {
        int sparks = 0, bad_dwell = 0, inj = 0, bad_pw = 0;
        for (int c = 0; c < 6; c++) {
            for (int i = 0; i < nop[6 + c]; i++) {
                int d = (uint16_t)(op[6 + c][i].off - op[6 + c][i].on);
                sparks++;
                if (abs(d - 750) > 12) bad_dwell++;               /* 3 ms +/- 1.6 % */
            }
            for (int i = 0; i < nop[c]; i++) {
                int d = (uint16_t)(op[c][i].off - op[c][i].on);
                inj++;
                if (abs(d - 625) > 1) bad_pw++;                   /* 2.5 ms */
            }
        }
        /* Merge sparks of all coils in time order; check firing order and
         * 120° (= 694.4 ticks at 7200 rpm) between consecutive sparks. */
        static struct { avr_cycle_count_t when; uint16_t off; int coil; } all_s[6 * MAXP];
        int ns = 0;
        for (int c = 0; c < 6; c++)
            for (int i = 0; i < nop[6 + c]; i++) {
                all_s[ns].when = op[6 + c][i].when;
                all_s[ns].off = op[6 + c][i].off;
                all_s[ns].coil = c;
                ns++;
            }
        for (int i = 1; i < ns; i++)                              /* insertion sort by arm time */
            for (int j = i; j > 0 && all_s[j].when < all_s[j - 1].when; j--) {
                __typeof__(all_s[0]) t = all_s[j]; all_s[j] = all_s[j - 1]; all_s[j - 1] = t;
            }
        static const int next_in_order[6] = {4, 3, 5, 0, 2, 1};   /* 1->5->3->6->2->4->1 (0-based) */
        int bad_order = 0, bad_gap = 0;
        double worst_gap = 0;
        for (int i = 1; i < ns; i++) {
            if (all_s[i].coil != next_in_order[all_s[i - 1].coil]) bad_order++;
            double g = (uint16_t)(all_s[i].off - all_s[i - 1].off);
            if (fabs(g - 694.4) > worst_gap) worst_gap = fabs(g - 694.4);
            if (fabs(g - 694.4) > 2) bad_gap++;
        }
        printf("schedule:  %d sparks (dwell off: %d), %d injections (pulse off: %d), firing order errors %d, "
               "spark spacing worst %.1f ticks off 120°\n", sparks, bad_dwell, inj, bad_pw, bad_order, worst_gap);
        CHECK(sparks > 6 * 60 && inj > 6 * 60);                  /* 66 cycles in 1.1 s */
        CHECK(bad_dwell == 0 && bad_pw == 0);
        CHECK(bad_order == 0 && bad_gap == 0);
    }

    /* Tooth log while running at 7200 rpm: the first 'T' starts a capture,
     * the second (after > 64 teeth = 9 ms) returns it. */
    send_frame((const uint8_t *)"T", 1, 0);
    n = recv_frame(p, 100000);
    CHECK(n >= 2 && p[0] == 0);
    run_us(20000);
    send_frame((const uint8_t *)"T", 1, 0);
    n = recv_frame(p, 100000);
    CHECK(n == 258 && p[0] == 0 && p[1] == 64);
    if (n == 258) {
        int gaps = 0, ok = 1;
        for (int i = 0; i < 64; i++) {
            const uint8_t *e = &p[2 + 4 * i];
            uint16_t iv = le16(e);
            if (e[3] & 0x01) {                           /* gap: 3 teeth = 104 ticks */
                gaps++;
                if (iv < 100 || iv > 108) ok = 0;
            } else if (iv < 33 || iv > 37) {             /* tooth: 34.7 ticks */
                ok = 0;
            }
        }
        printf("tooth log: %d entries, %d gaps, intervals %s\n", 64, gaps, ok ? "as expected" : "WRONG");
        CHECK(gaps >= 1 && ok);
    }

    /* ---- 4. engine stops ---- */
    engine_set_rpm(0);
    run_us(400000);
    CHECK(get_rt(&r));
    printf("stopped:   rpm %u, sync %u, last loss reason %u\n", r.rpm, r.sync, r.loss_reason);
    CHECK(r.rpm == 0 && r.sync == 0 && r.loss_reason == 3);
    CHECK(r.sync_losses == 0);
    if (sched_addr) {
        int idle = all_channels_idle();
        printf("stopped:   all 12 output channels idle: %s\n", idle ? "yes" : "NO");
        CHECK(idle);
    } else {
        CHECK(0);
    }

    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
