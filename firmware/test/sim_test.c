/*
 * End-to-end test: run the real firmware ELF in simavr, talk to it over the
 * emulated UART0 and check the replies. Drives simulated ADC voltages and a
 * simulated engine (60-2 crank + half-moon cam on the ICP5/ICP4 pins, made
 * with the trigger simulator's own pattern code), and times the crank ISR.
 *
 *   make sim-test
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sim_avr.h"
#include "sim_elf.h"
#include "sim_irq.h"
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

/* ---- crank ISR timing ---------------------------------------------------- */

static uint32_t isr_addr, isr_size;
static avr_cycle_count_t isr_start;
static int in_isr;
static uint32_t isr_max, isr_count;
static uint64_t isr_total;

static void isr_poll(void)
{
    if (!isr_size) return;
    uint32_t pc = avr->pc;
    int inside = pc >= isr_addr && pc < isr_addr + isr_size;
    if (inside && !in_isr) {
        in_isr = 1;
        isr_start = avr->cycle;
    } else if (!inside && in_isr) {
        in_isr = 0;
        /* + 5 cycles interrupt response + 3 for the vector-table jump */
        uint32_t c = (uint32_t)(avr->cycle - isr_start) + 8;
        if (c > isr_max) isr_max = c;
        isr_total += c;
        isr_count++;
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
    uint8_t  reset_cause, flags, sync, cam_pos, loss_reason;
} rt_t;

static int get_rt(rt_t *r)
{
    uint8_t p[300];
    send_frame((const uint8_t *)"A", 1, 0);
    int n = recv_frame(p, 50000);
    if (n != 47 || p[0] != 0) return 0;
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
    return 1;
}

static void load_isr_symbol(const char *symfile)
{
    FILE *f = fopen(symfile, "r");
    if (!f) return;
    char line[256];
    while (fgets(line, sizeof line, f)) {
        unsigned a, s;
        char type, name[128];
        if (sscanf(line, "%x %x %c %127s", &a, &s, &type, name) == 4 && !strcmp(name, "__vector_46")) {
            isr_addr = a;
            isr_size = s;
        }
    }
    fclose(f);
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: %s ecu.elf mcu [ecu.sym]\n", argv[0]); return 2; }
    if (argc > 3) load_isr_symbol(argv[3]);

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

    /* ---- 3. full-throttle rev to 7200 rpm, timing the crank ISR ---- */
    engine_ramp(7200, 0.6);
    run_us(700000);
    isr_max = 0;
    isr_count = 0;
    isr_total = 0;
    run_us(1100000);                             /* > 1 s so loops/s reflects load */
    CHECK(get_rt(&r));
    printf("7200 rpm:  rpm %u (tooth %u), sync %u, losses %u, main loop %u passes/s (idle %u)\n",
           r.rpm, r.rpm_inst, r.sync, r.sync_losses, r.loops, idle_loops);
    CHECK(r.sync == 3);
    CHECK(r.rpm >= 7195 && r.rpm <= 7205);
    CHECK(r.sync_losses == 0 && r.noise == 0 && r.cam_errors == 0);
    if (isr_size) {
        printf("crank ISR: %u calls, average %.0f cycles = %.2f us, worst %u cycles = %.2f us (limit 17 us)\n",
               isr_count, (double)isr_total / isr_count, (double)isr_total / isr_count / 16.0,
               isr_max, isr_max / 16.0);
        CHECK(isr_count > 7000);
        CHECK(isr_max <= 272);              /* 17 us: regression guard, see PLAN.md 3.6 */
    } else {
        printf("crank ISR: symbol not found, timing skipped\n");
        CHECK(0);
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

    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
