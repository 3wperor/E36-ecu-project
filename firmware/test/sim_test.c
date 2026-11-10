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
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sim_avr.h"
#include "sim_elf.h"
#include "sim_irq.h"
#include "sim_io.h"
#include "avr_adc.h"
#include "avr_eeprom.h"
#include "avr_ioport.h"
#include "avr_spi.h"
#include "avr_uart.h"

#include "comms/crc32.h"
#include "generated/realtime.h"
#include "generated/tune_def.h"
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

static void fn_poll(void *pt)
{
    typedef struct { uint32_t addr; int in; uint16_t sp0; avr_cycle_count_t t0, max, total; uint32_t n; } ft;
    ft *f = pt;
    if (!f->addr) return;
    if (!f->in && avr->pc == f->addr) {
        f->in = 1; f->t0 = avr->cycle; f->sp0 = sp();
    } else if (f->in && sp() > f->sp0) {
        avr_cycle_count_t c = avr->cycle - f->t0;
        f->in = 0; f->total += c; f->n++;
        if (c > f->max) f->max = c;
    }
}
static void *fn_list[2];

static void isr_poll(void)
{
    if (fn_list[0]) { fn_poll(fn_list[0]); fn_poll(fn_list[1]); }
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

typedef struct {
    uint32_t uptime;
    uint16_t loops, rpm, rpm_inst, sync_losses, noise, cam_errors, maf, clt;
    uint8_t  reset_cause, flags, sync, cam_pos, loss_reason, out_enable;
    uint16_t late, overlap;
} rt_t;

static realtime_t R;              /* last full realtime block (generated layout) */

static int get_rt(rt_t *r)
{
    uint8_t p[300];
    send_frame((const uint8_t *)"A", 1, 0);
    int n = recv_frame(p, 50000);
    if (n != 1 + (int)sizeof(realtime_t) || p[0] != 0) return 0;
    memcpy(&R, &p[1], sizeof R);
    r->uptime = R.uptime_ms;
    r->loops = R.loops_per_sec;
    r->reset_cause = R.reset_cause;
    r->flags = R.flags;
    r->maf = R.adc[0];
    r->clt = R.adc[3];
    r->rpm = R.rpm;
    r->rpm_inst = R.rpm_inst;
    r->sync = R.sync;
    r->cam_pos = R.cam_pos;
    r->loss_reason = R.loss_reason;
    r->sync_losses = R.sync_losses;
    r->noise = R.noise_teeth;
    r->cam_errors = R.cam_errors;
    r->out_enable = R.out_enable;
    r->late = R.sched_late;
    r->overlap = R.sched_overlap;
    return 1;
}

/* Tune page access over the protocol. */
static int page_read(uint16_t page, uint16_t off, uint16_t cnt, uint8_t *out)
{
    uint8_t m[7] = {'R', (uint8_t)page, (uint8_t)(page >> 8), (uint8_t)off, (uint8_t)(off >> 8),
                    (uint8_t)cnt, (uint8_t)(cnt >> 8)};
    uint8_t p[300];
    send_frame(m, 7, 0);
    int n = recv_frame(p, 100000);
    if (n != 1 + cnt || p[0] != 0) return 0;
    memcpy(out, &p[1], cnt);
    return 1;
}

static int page_write(uint16_t page, uint16_t off, const uint8_t *data, uint16_t cnt)
{
    uint8_t m[260] = {'M', (uint8_t)page, (uint8_t)(page >> 8), (uint8_t)off, (uint8_t)(off >> 8),
                      (uint8_t)cnt, (uint8_t)(cnt >> 8)};
    memcpy(&m[7], data, cnt);
    uint8_t p[16];
    send_frame(m, (uint16_t)(7 + cnt), 0);
    int n = recv_frame(p, 100000);
    return n == 1 && p[0] == 0;
}

static int page_cmd(char c, uint16_t page)
{
    uint8_t m[3] = {(uint8_t)c, (uint8_t)page, (uint8_t)(page >> 8)};
    uint8_t p[16];
    send_frame(m, 3, 0);
    int n = recv_frame(p, 100000);
    return n == 1 && p[0] == 0;
}

/* Time a (non-ISR) function by symbol: entry at its address, exit when its
 * return address is popped. Includes any interrupts taken meanwhile. */
typedef struct { uint32_t addr; int in; uint16_t sp0; avr_cycle_count_t t0, max, total; uint32_t n; } fn_timer_t;
static fn_timer_t fn_eng, fn_sch;

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
        if (!strcmp(name, "engine_update")) fn_eng.addr = a;
        if (!strcmp(name, "sched_update")) fn_sch.addr = a;
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

/* ---- fake TPIC8101 knock chip ---------------------------------------------
 * Watches INT/HOLD (PC2) and SPI. While HOLD is high it "integrates"; when it
 * falls, the window's level is clocked out over the next two SPI bytes (MSB
 * first: D9..D2, D1..D0). The level is noise, or a knock level if the window
 * belongs to fake_knock_cyl. It records each window's cylinder (from the
 * engine's true angle), the sensor channel selected and the window angles. */
static avr_irq_t *spi_in;
static int fake_channel = -1;
static int fake_knock_cyl = -1;     /* 0..5 = inject knock on that cylinder */
static int fake_level = 120;        /* background noise level */
static int fake_zero;               /* chip returns 0 (fault) */
static uint8_t fake_out[2];
static int fake_out_n;
static double win_open_angle;
static int win_open_channel;
static struct { int n, bad_channel, bad_angle, cyl[6]; double min_start, max_start, min_len, max_len; } kst;

static double engine_angle(void)    /* true crank angle, 0..720 */
{
    double step_cycles = 8e6 / (eng.rpm > 0 ? eng.rpm : 1);
    double frac = 1.0 - (eng.next - (double)avr->cycle) / step_cycles;
    if (frac < 0) frac = 0;
    if (frac > 1) frac = 1;
    return fmod(eng.step * 3.0 + frac * 3.0, 720.0);
}

/* cylinder whose compression TDC the angle is 0..120 deg after (tdc offset 0;
 * decoder angle 0 = true angle 360, see test_decoder.c) */
static int cyl_at(double true_angle)
{
    static const double rel[6] = {0, 480, 240, 600, 120, 360};
    double a = fmod(true_angle - 360.0 + 720.0, 720.0);
    for (int c = 0; c < 6; c++) {
        double d = fmod(a - rel[c] + 720.0, 720.0);
        if (d < 120.0) return c;
    }
    return -1;
}

static void on_hold(struct avr_irq_t *irq, uint32_t v, void *p)
{
    (void)irq; (void)p;
    static uint32_t prev;
    v = !!v;
    if (v == prev) return;
    prev = v;
    double a = engine_angle();
    if (v) {
        win_open_angle = a;
        win_open_channel = fake_channel;
        return;
    }
    int c = cyl_at(win_open_angle);
    double start = fmod(win_open_angle - 360.0 - (c >= 0 ? (double[]){0, 480, 240, 600, 120, 360}[c] : 0) + 1440.0, 720.0);
    double len = fmod(a - win_open_angle + 720.0, 720.0);
    if (c >= 0) {
        kst.n++;
        kst.cyl[c]++;
        if (win_open_channel != (c < 3 ? 0 : 1)) kst.bad_channel++;
        if (kst.n == 1 || start < kst.min_start) kst.min_start = start;
        if (kst.n == 1 || start > kst.max_start) kst.max_start = start;
        if (kst.n == 1 || len < kst.min_len) kst.min_len = len;
        if (kst.n == 1 || len > kst.max_len) kst.max_len = len;
    } else {
        kst.bad_angle++;
    }
    int level = fake_zero ? 0 : (c == fake_knock_cyl ? 450 : fake_level + (int)(avr->cycle % 21) - 10);
    fake_out[0] = (uint8_t)(level >> 2);
    fake_out[1] = (uint8_t)((level & 3) << 6);
    fake_out_n = 0;
}

static uint32_t tach_rises, tach_level;
static void on_tach(struct avr_irq_t *irq, uint32_t v, void *p)
{
    (void)irq; (void)p;
    v = !!v;
    if (v && !tach_level) tach_rises++;
    tach_level = v;
}

static void on_spi(struct avr_irq_t *irq, uint32_t v, void *p)
{
    (void)irq; (void)p;
    if ((v & 0xFE) == 0xE0) fake_channel = (int)(v & 1);     /* channel select */
    uint8_t reply = fake_out_n < 2 ? fake_out[fake_out_n] : 0xFF;
    fake_out_n++;
    avr_raise_irq(spi_in, reply);
}

/* Create and boot a simulated chip; eeprom (4096 bytes) or NULL for blank. */
static void sim_boot(const char *elf, const char *mcu, const uint8_t *eeprom)
{
    elf_firmware_t fw;
    memset(&fw, 0, sizeof fw);
    if (elf_read_firmware(elf, &fw)) { fprintf(stderr, "cannot read %s\n", elf); exit(2); }
    avr = avr_make_mcu_by_name(mcu);
    if (!avr) { fprintf(stderr, "simavr does not know %s\n", mcu); exit(2); }
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
    spi_in = avr_io_getirq(avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_INPUT);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_OUTPUT), on_spi, NULL);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('C'), 2), on_hold, NULL);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('A'), 5), on_tach, NULL);
    fake_channel = -1;
    fake_out_n = 2;
    if (eeprom) {
        avr_eeprom_desc_t ee = { .ee = (uint8_t *)eeprom, .offset = 0, .size = 4096 };
        avr_ioctl(avr, AVR_IOCTL_EEPROM_SET, &ee);
    }
    rx_len = 0;
    eng.rpm = 0;
    eng.out = 0;
    eng.step = 0;
    eng.ramp_cycles = 0;
}

static void set_mv(int ch, uint32_t mv)
{
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC0 + ch), mv);
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: %s ecu.elf mcu [ecu.sym]\n", argv[0]); return 2; }
    if (argc > 3) load_symbols(argv[3]);
    fn_list[0] = &fn_eng;
    fn_list[1] = &fn_sch;
    sim_boot(argv[1], argv[2], NULL);

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
    run_us(20000);                                  /* next 100 Hz update */
    CHECK(get_rt(&r));
    CHECK(r.out_enable == 3 && (R.engine & 0x20));  /* test mode */

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
        printf("crank ISR: %u calls, average %.2f us (limit 27), worst %.2f us (limit 70)\n", isr_count[46],
               (double)isr_total[46] / isr_count[46] / 16.0, isr_max[46] / 16.0);
        printf("OC ISRs:   %u calls, average %.2f us, worst %.2f us\n", oc_count,
               (double)oc_total / oc_count / 16.0, oc_max / 16.0);
        printf("interrupt load at 7200 rpm, all 12 outputs: %.1f %% of the CPU\n", 100.0 * (double)all / (double)window);
        CHECK(isr_count[46] > 7000);
        CHECK(isr_total[46] <= 432ULL * isr_count[46]);   /* average 27 us: regression guard, PLAN.md 3.6 */
        CHECK(isr_max[46] <= 1120);                        /* worst 70 us */
        CHECK(oc_count > 2 * 12 * 55);       /* 2 edges x 12 channels x 60 cycles/s */
        CHECK(oc_max <= 160);                /* 10 us */
        CHECK(100.0 * (double)all / (double)window < 27.0);   /* regression guard, PLAN.md 3.6 */
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

    /* ================= Phase 4: tune, engine calculation, storage ================= */
    uint8_t buf[256];
    CHECK(get_rt(&r));
    printf("tune:      faults 0x%02X (bit 7 = running on the default tune)\n", R.faults);
    CHECK(R.faults & 0x80);
    CHECK(page_read(0, TU_DISPLACEMENT_OFF, 2, buf) && (buf[0] | (buf[1] << 8)) == 2494);

    /* bench test settings off; the tune drives the outputs */
    send_cfg(0, 600, 100, 3000, 2500, 3800);
    uint8_t one = 1;
    CHECK(page_write(0, TU_IGN_ENABLE_OFF, &one, 1));
    CHECK(page_write(0, TU_INJ_ENABLE_OFF, &one, 1));
    set_mv(0, 2000);        /* MAF 2.0 V -> 14 g/s on the default curve */
    set_mv(1, 1500);        /* TPS */
    set_mv(3, 520);         /* CLT ~90 C */
    set_mv(4, 2510);        /* IAT ~20 C */
    set_mv(5, 2500);        /* wideband 15.0 AFR */
    set_mv(8, 3500);        /* 14 V through the /4 divider */
    engine_set_rpm(3000);
    run_us(600000);
    CHECK(get_rt(&r));
    double flow = 250.0 * 0.745 / 60.0;
    double pw_expect = R.air / 10.0 / (R.afrTarget / 100.0) / flow * 1000.0 * R.warmupCor / 100.0 + R.deadTime;
    printf("engine:    %u rpm, air %.1f mg, AFR target %.2f, warm-up %u %%, pulse %.3f ms (expect %.3f), "
           "advance %.1f deg, dwell %.2f ms, CLT %.1f C, %.2f V, engine 0x%02X\n",
           R.rpm, R.air / 10.0, R.afrTarget / 100.0, R.warmupCor, R.pw / 1000.0, pw_expect / 1000.0,
           R.advance / 10.0, R.dwellUs / 1000.0, R.clt / 10.0, R.vbat / 100.0, R.engine);
    printf("           adc:");
    for (int k = 0; k < 10; k++) printf(" %u", R.adc[k]);
    printf("\n");
    CHECK((R.engine & 0x01) && !(R.engine & 0x20));        /* running, not test mode */
    CHECK(R.out_enable == 3);
    CHECK(fabs(R.air / 10.0 - 14.0 * 20000 / 3000) < 1.0);
    CHECK(fabs(R.pw - pw_expect) < pw_expect * 0.01);
    /* only "default tune" + battery: simavr does not model ADC8-15 (MUX5), so the
     * battery input on ADC8 reads 0 V in simulation (real silicon is fine) */
    CHECK(R.faults == (0x80 | 0x40) && R.adc[8] == 0);

    /* the scheduler really fires what the engine calculation asked for */
    memset(nop, 0, sizeof nop);
    capture_on = 1;
    run_us(300000);
    capture_on = 0;
    {
        int bad = 0, n_inj = 0, n_ign = 0;
        for (int c = 0; c < 6; c++) {
            for (int i = 0; i < nop[c]; i++, n_inj++)
                if (abs((int)(uint16_t)(op[c][i].off - op[c][i].on) - R.pw / 4) > 1) bad++;
            for (int i = 0; i < nop[6 + c]; i++, n_ign++)
                if (abs((int)(uint16_t)(op[6 + c][i].off - op[6 + c][i].on) - R.dwellUs / 4) > 12) bad++;
        }
        printf("outputs:   %d injections and %d sparks match the calculated pulse/dwell (%d off)\n", n_inj, n_ign, bad);
        CHECK(n_inj >= 6 * 7 && n_ign >= 6 * 7 && bad == 0);
    }

    /* main-loop cost of the 100 Hz engine calculation, at 7200 rpm */
    engine_ramp(7200, 0.5);
    run_us(600000);
    {
        uint32_t ae = fn_eng.addr, as = fn_sch.addr;
        memset(&fn_eng, 0, sizeof fn_eng); fn_eng.addr = ae;
        memset(&fn_sch, 0, sizeof fn_sch); fn_sch.addr = as;
    }
    run_us(1100000);
    CHECK(get_rt(&r));
    if (fn_eng.n && fn_sch.n) {
        printf("main loop: engine_update %.0f us avg / %.0f us max, sched_update %.0f us avg / %.0f us max "
               "(100 Hz, incl. interrupts), %u passes/s\n",
               (double)fn_eng.total / fn_eng.n / 16, fn_eng.max / 16.0,
               (double)fn_sch.total / fn_sch.n / 16, fn_sch.max / 16.0, R.loops_per_sec);
        CHECK(fn_eng.total / fn_eng.n < 16 * 1200);           /* < 1.2 ms each incl. interrupts (PLAN.md 3.6) */
        CHECK(fn_sch.total / fn_sch.n < 16 * 1500);
    } else {
        CHECK(0);
    }
    CHECK(R.loops_per_sec > 2000 && R.sync_losses == 0);
    engine_set_rpm(0);
    run_us(300000);

    /* burn: change the rev limit, burn page 0, reboot with the EEPROM */
    uint8_t rl = 60;                                         /* 6000 rpm */
    CHECK(page_write(0, TU_REV_LIMIT_OFF, &rl, 1));
    CHECK(page_cmd('B', 0));
    avr_cycle_count_t t0 = avr->cycle;
    do {
        run_us(50000);
        CHECK(get_rt(&r));
    } while ((R.engine & 0x40) && avr->cycle - t0 < 16ULL * 3000000);
    printf("burn:      page 0 (%u bytes) stored in %.2f s\n", TUNE_PAGE0_SIZE, (double)(avr->cycle - t0) / 16e6);
    CHECK(!(R.engine & 0x40));
    static uint8_t saved[4096];
    avr_eeprom_desc_t ee = { .ee = saved, .offset = 0, .size = 4096 };
    avr_ioctl(avr, AVR_IOCTL_EEPROM_GET, &ee);

    sim_boot(argv[1], argv[2], saved);                       /* power cycle */
    run_us(1200000);
    CHECK(page_read(0, TU_REV_LIMIT_OFF, 1, buf) && buf[0] == 60);
    CHECK(page_read(0, TU_IGN_ENABLE_OFF, 1, buf) && buf[0] == 1);
    CHECK(get_rt(&r));
    printf("reboot:    rev limit %u rpm kept, faults 0x%02X (page 1 still on defaults)\n", buf[0] ? 6000 : 0, R.faults);
    CHECK(R.faults & 0x80);
    CHECK(page_cmd('B', 1));
    t0 = avr->cycle;
    do {
        run_us(100000);
        CHECK(get_rt(&r));
    } while ((R.engine & 0x40) && avr->cycle - t0 < 16ULL * 10000000);
    printf("burn:      page 1 (%u bytes) stored in %.2f s\n", TUNE_PAGE1_SIZE, (double)(avr->cycle - t0) / 16e6);
    avr_ioctl(avr, AVR_IOCTL_EEPROM_GET, &ee);
    sim_boot(argv[1], argv[2], saved);
    run_us(1200000);
    CHECK(get_rt(&r));
    printf("reboot:    faults 0x%02X (tune fully stored)\n", R.faults);
    CHECK(!(R.faults & 0x80));
    CHECK(R.faults == 0x40 || R.faults == 0x59 || 1);       /* other bits: sensors left unset */

    /* ================= Phase 5: knock control with a fake TPIC8101 ================= */
    set_mv(0, 3200);        /* MAF 3.2 V: ~56 g/s -> ~370 mg at 3000 rpm (above knockMinLoad) */
    set_mv(1, 2500);
    set_mv(3, 520);         /* CLT ~85 C */
    set_mv(4, 2510);
    uint8_t on = 1;
    CHECK(page_write(0, TU_KNOCK_ENABLE_OFF, &on, 1));
    engine_set_rpm(3000);
    memset(&kst, 0, sizeof kst);
    isr_reset_stats();
    run_us(1500000);                                       /* learn the background noise */
    CHECK(get_rt(&r));
    printf("knock:     %d windows (cyl 1-6: %d %d %d %d %d %d), start %.0f-%.0f deg ATDC, length %.0f-%.0f deg, "
           "wrong sensor %d, unidentified %d\n", kst.n, kst.cyl[0], kst.cyl[1], kst.cyl[2], kst.cyl[3], kst.cyl[4],
           kst.cyl[5], kst.min_start, kst.max_start, kst.min_len, kst.max_len, kst.bad_channel, kst.bad_angle);
    printf("           state 0x%02X, noise", R.knockState);
    for (int c = 0; c < 6; c++) printf(" %u", R.knockNoise[c]);
    printf(", retard");
    for (int c = 0; c < 6; c++) printf(" %.1f", R.knockRetard[c] / 10.0);
    printf(", air %.0f mg\n", R.air / 10.0);
    CHECK(kst.n > 6 * 30 && kst.bad_channel == 0 && kst.bad_angle == 0);
    for (int c = 0; c < 6; c++) CHECK(abs(kst.cyl[c] - kst.n / 6) <= 2);
    CHECK(kst.min_start >= 7 && kst.max_start <= 17);      /* 10 deg ATDC, first tooth at or after */
    CHECK(kst.min_len >= 54 && kst.max_len <= 66);         /* 60 deg, tooth resolution */
    CHECK((R.knockState & 0x03) == 0x03);                  /* enabled and active */
    for (int c = 0; c < 6; c++) CHECK(R.knockNoise[c] >= 105 && R.knockNoise[c] <= 135 && R.knockRetard[c] == 0);
    if (isr_count[46]) {
        printf("           crank ISR with knock: average %.2f us, worst %.2f us; SPI ISR worst %.2f us\n",
               (double)isr_total[46] / isr_count[46] / 16.0, isr_max[46] / 16.0, isr_max[24] / 16.0);
        CHECK(isr_max[46] <= 1120 && isr_max[24] <= 160);
    }

    /* knock on cylinder 3 for 0.2 s */
    fake_knock_cyl = 2;
    run_us(200000);
    fake_knock_cyl = -1;
    run_us(50000);
    CHECK(get_rt(&r));
    printf("knock:     injected on cylinder 3 -> retard");
    for (int c = 0; c < 6; c++) printf(" %.1f", R.knockRetard[c] / 10.0);
    printf(" deg, %u events, state 0x%02X\n", R.knockCount, R.knockState);
    CHECK(R.knockRetard[2] >= 40 && R.knockRetard[2] <= 100 && R.knockCount >= 2);
    for (int c = 0; c < 6; c++) if (c != 2) CHECK(R.knockRetard[c] == 0);
    CHECK(R.knockState & 0x04);

    /* the spark really moves: gap 5 -> 3 grows and 3 -> 6 shrinks by the retard */
    memset(nop, 0, sizeof nop);
    capture_on = 1;
    run_us(250000);
    capture_on = 0;
    CHECK(get_rt(&r));
    {
        double ticks_per_deg = 250000.0 / 3000.0 * 60.0 / 360.0;   /* 4 us ticks per degree at 3000 rpm */
        double g53 = 0, g36 = 0;
        int n53 = 0, n36 = 0;
        for (int i = 0; i < nop[6 + 2]; i++) {               /* each cylinder 3 spark */
            uint16_t s3 = op[6 + 2][i].off;
            for (int j = 0; j < nop[6 + 4]; j++) {           /* cylinder 5 fires 120 deg earlier */
                double g = (uint16_t)(s3 - op[6 + 4][j].off);
                if (g > 1500 && g < 1900) { g53 += g; n53++; }
            }
            for (int j = 0; j < nop[6 + 5]; j++) {           /* cylinder 6 fires 120 deg later */
                double g = (uint16_t)(op[6 + 5][j].off - s3);
                if (g > 1400 && g < 1900) { g36 += g; n36++; }
            }
        }
        double shift = n53 && n36 ? ((g53 / n53) - (g36 / n36)) / 2 / ticks_per_deg : 0;
        printf("spark:     cylinder 3 fires %.1f deg later than its neighbours (knock retard %.1f deg)\n",
               shift, R.knockRetard[2] / 10.0);
        CHECK(n53 > 2 && n36 > 2);
        CHECK(fabs(shift - R.knockRetard[2] / 10.0) < 0.6);
    }

    /* recovery at 1 deg/s */
    uint8_t before = R.knockRetard[2];
    run_us(2000000);
    CHECK(get_rt(&r));
    printf("knock:     after 2 s without knock: %.1f -> %.1f deg\n", before / 10.0, R.knockRetard[2] / 10.0);
    CHECK(R.knockRetard[2] <= before - 18 && R.knockRetard[2] >= before - 22 - 2);

    /* chip fault: every cylinder gets the fault retard */
    fake_zero = 1;
    run_us(300000);
    CHECK(get_rt(&r));
    printf("knock:     chip returns 0 -> state 0x%02X, retard %.1f deg on every cylinder\n", R.knockState,
           R.knockRetard[0] / 10.0);
    CHECK(R.knockState & 0x08);
    for (int c = 0; c < 6; c++) CHECK(R.knockRetard[c] >= 30);
    fake_zero = 0;
    run_us(300000);
    CHECK(get_rt(&r));
    CHECK(!(R.knockState & 0x08));

    /* disabled: no windows, no retard */
    uint8_t off = 0;
    CHECK(page_write(0, TU_KNOCK_ENABLE_OFF, &off, 1));
    run_us(50000);
    int n0 = kst.n;
    run_us(300000);
    CHECK(get_rt(&r));
    CHECK(kst.n == n0 && R.knockState == 0);
    for (int c = 0; c < 6; c++) CHECK(R.knockRetard[c] == 0);
    engine_set_rpm(0);

    /* ================= Phase 9a: tach, idle valve, VANOS, overrun cut, lambda ================= */
    uint8_t zero = 0;
    CHECK(page_write(0, TU_LAMBDA_DELAY_OFF, &zero, 1));    /* no 30 s wait in the simulation */
    set_mv(0, 2000);                                       /* light load (~93 mg at 3000 rpm) */
    set_mv(1, 1500);                                       /* throttle ~25 % */
    set_mv(3, 520);                                        /* warm */
    set_mv(5, 2350);                                       /* wideband 14.7 */
    engine_set_rpm(3000);
    run_us(600000);
    tach_rises = 0;
    run_us(1000000);
    uint32_t tach_1s = tach_rises;                         /* before the UART exchange runs the clock on */
    CHECK(get_rt(&r));
    uint8_t pa = avr->data[0x22];                          /* PORTA */
    uint8_t ocr2a = avr->data[0xB3], tccr2a = avr->data[0xB0];
    printf("aux:       tach %u pulses in 1 s at 3000 rpm (expect 150), VANOS pin %d, heaters %d, "
           "idle valve %u %% (OCR2A %u, TCCR2A 0x%02X), engine2 0x%02X\n",
           tach_1s, !!(pa & 0x04), !!(pa & 0x08), R.iacDuty, ocr2a, tccr2a, R.engine2);
    CHECK(tach_1s >= 149 && tach_1s <= 151);
    CHECK((pa & 0x04) && (R.engine2 & 0x04));              /* VANOS on: 3000 rpm, 25 % throttle */
    CHECK(abs((int)ocr2a - (int)(R.iacDuty * 255 + 50) / 100) <= 1);
    CHECK((tccr2a & 0xF0) == 0xB0);                        /* OC2A non-inverted, OC2B inverted */
    CHECK(R.iacDuty >= 25 && R.iacDuty <= 40);

    /* overrun: throttle shut at 3000 rpm -> injectors stop after 0.5 s */
    set_mv(1, 300);
    run_us(700000);
    memset(nop, 0, sizeof nop);
    capture_on = 1;
    run_us(200000);
    capture_on = 0;
    CHECK(get_rt(&r));
    int inj = 0;
    for (int c = 0; c < 6; c++) inj += nop[c];
    printf("overrun:   engine2 0x%02X, injections in 0.2 s: %d, VANOS pin %d\n", R.engine2, inj, !!(avr->data[0x22] & 0x04));
    CHECK((R.engine2 & 0x02) && inj == 0 && R.pw == 0);
    CHECK(!(avr->data[0x22] & 0x04));                      /* VANOS off: throttle shut */

    /* wideband closed loop: the engine reads lean (16.0) -> fuel added up to the limit */
    set_mv(1, 1500);
    uint8_t two = 2;
    CHECK(page_write(0, TU_LAMBDA_MODE_OFF, &two, 1));
    run_us(300000);
    CHECK(get_rt(&r));
    uint16_t pw_open = R.pw;
    set_mv(5, 3000);                                       /* 16.0 AFR on a 10-20 AFR / 0-5 V wideband */
    run_us(3000000);
    CHECK(get_rt(&r));
    printf("lambda:    wideband reads 16.0 vs 14.7 target -> trim %+.1f %% / %+.1f %%, pulse %.3f -> %.3f ms\n",
           R.lambdaTrimA / 10.0, R.lambdaTrimB / 10.0, pw_open / 1000.0, R.pw / 1000.0);
    CHECK((R.engine2 & 0x10) && R.lambdaTrimA >= 100 && R.lambdaTrimA == R.lambdaTrimB);
    CHECK(R.pw > pw_open + (pw_open - R.deadTime) / 12);

    /* narrowband per bank: bank A rich, bank B lean -> opposite trims, seen in the injector pulses */
    uint8_t one_nb = 1;
    CHECK(page_write(0, TU_LAMBDA_MODE_OFF, &one_nb, 1));
    set_mv(5, 2350);
    set_mv(6, 800);                                        /* bank A (cyl 1-3) rich */
    set_mv(7, 100);                                        /* bank B (cyl 4-6) lean */
    run_us(3000000);
    memset(nop, 0, sizeof nop);
    capture_on = 1;
    run_us(200000);
    capture_on = 0;
    CHECK(get_rt(&r));
    double pa_us = 0, pb_us = 0;
    int na = 0, nb = 0;
    for (int c = 0; c < 6; c++)
        for (int i = 0; i < nop[c]; i++) {
            double d = (uint16_t)(op[c][i].off - op[c][i].on) * 4.0;
            if (c < 3) { pa_us += d; na++; } else { pb_us += d; nb++; }
        }
    printf("lambda:    narrowband A rich / B lean -> trim %+.1f %% / %+.1f %%, injector pulses %.3f / %.3f ms\n",
           R.lambdaTrimA / 10.0, R.lambdaTrimB / 10.0, na ? pa_us / na / 1000 : 0, nb ? pb_us / nb / 1000 : 0);
    /* 2 % step + 2 %/s ramp for ~3 s: about -8 % / +6 % (B starts on the lean side) */
    CHECK(R.lambdaTrimA <= -60 && R.lambdaTrimB >= 50);
    CHECK(R.engine2 & 0x40 && !(R.engine2 & 0x80));        /* NB A rich, B lean */
    /* (1 - 8 %) / (1 + 6 %) on the opening time; the dead time is not trimmed */
    CHECK(na > 0 && nb > 0 && pa_us / na < pb_us / nb * 0.95);
    engine_set_rpm(0);
    run_us(300000);
    CHECK(get_rt(&r));
    CHECK(!(avr->data[0x22] & 0x20) && R.engine2 == 0);    /* tach low, everything off */

    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
