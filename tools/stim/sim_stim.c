/*
 * Runs the stim firmware in simavr, commands 6000 rpm over UART, and checks
 * the generated crank/cam waveform.
 *
 * Why register writes and not pins: simavr 1.6 mis-models output-compare pins
 * in Normal timer mode (it also drives them at overflow, as in PWM mode; the
 * datasheet says Normal mode only acts on compare match). The firmware arms
 * every edge by writing TCCR1A (COM bits = level after the match) followed by
 * OCR1A (match time). Capturing those writes gives exactly the waveform the
 * real Timer1 hardware produces.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sim_avr.h"
#include "sim_elf.h"
#include "sim_io.h"
#include "sim_irq.h"
#include "avr_uart.h"

#define F_CPU 16000000ULL

/* ATmega328P data-space addresses */
#define ADDR_TCCR1A 0x80
#define ADDR_OCR1AL 0x88
#define ADDR_OCR1AH 0x89

static avr_t *avr;
static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

typedef struct { uint64_t t; uint8_t crank, cam; } edge_t;   /* t in 0.5 us ticks, unwrapped */
static edge_t ev[4000];
static int n_ev, recording;
static uint8_t last_com, ocr_high;
static uint16_t last_ocr;
static uint64_t t_unwrapped;

static void on_tccr1a(struct avr_irq_t *irq, uint32_t v, void *p) { (void)irq; (void)p; last_com = (uint8_t)v; }
static void on_ocr1ah(struct avr_irq_t *irq, uint32_t v, void *p) { (void)irq; (void)p; ocr_high = (uint8_t)v; }

static void on_ocr1al(struct avr_irq_t *irq, uint32_t v, void *p)
{
    (void)irq; (void)p;
    uint16_t ocr = (uint16_t)((ocr_high << 8) | (v & 0xFF));
    t_unwrapped += (uint16_t)(ocr - last_ocr);
    last_ocr = ocr;
    if (recording && n_ev < 4000) {
        ev[n_ev].t = t_unwrapped;
        ev[n_ev].crank = (last_com & 0xC0) == 0xC0;   /* COM1A = 11: set */
        ev[n_ev].cam   = (last_com & 0x30) == 0x30;   /* COM1B = 11: set */
        n_ev++;
    }
}

static void run_us(uint64_t us)
{
    avr_cycle_count_t end = avr->cycle + us * (F_CPU / 1000000ULL);
    while (avr->cycle < end) {
        int s = avr_run(avr);
        if (s == cpu_Done || s == cpu_Crashed) { printf("CPU stopped\n"); exit(2); }
    }
}

static void send_line(const char *s)
{
    avr_irq_t *in = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT);
    for (; *s; s++) { avr_raise_irq(in, (uint8_t)*s); run_us(100); }
    avr_raise_irq(in, '\n');
    run_us(2000);
}

int main(int argc, char **argv)
{
    if (argc < 2) return 2;
    elf_firmware_t fw;
    memset(&fw, 0, sizeof fw);
    if (elf_read_firmware(argv[1], &fw)) return 2;
    avr = avr_make_mcu_by_name("atmega328p");
    avr_init(avr);
    fw.frequency = F_CPU;
    avr_load_firmware(avr, &fw);
    avr->frequency = F_CPU;
    avr->log = 0;

    uint32_t flags = 0;
    avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &flags);
    flags &= ~AVR_UART_FLAG_STDIO;
    avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &flags);

    avr_irq_register_notify(avr_iomem_getirq(avr, ADDR_TCCR1A, "tccr1a", 8), on_tccr1a, NULL);
    avr_irq_register_notify(avr_iomem_getirq(avr, ADDR_OCR1AH, "ocr1ah", 8), on_ocr1ah, NULL);
    avr_irq_register_notify(avr_iomem_getirq(avr, ADDR_OCR1AL, "ocr1al", 8), on_ocr1al, NULL);

    run_us(50000);                         /* boot at 1000 rpm */
    send_line("c1");                       /* single cam pulse per cycle */
    send_line("r6000");
    run_us(50000);                         /* settle */

    recording = 1;
    run_us(3 * 20000 + 1000);              /* 3+ cycles at 6000 rpm (20 ms each) */
    recording = 0;

    /* At 6000 rpm: tooth = 166.667 us = 333.33 ticks, gap = 3 teeth = 1000 ticks,
     * whole 720° cycle = 40000 ticks exactly. */
    uint64_t rises[400], cam_rises[8];
    int nr = 0, nc = 0;
    for (int i = 1; i < n_ev; i++) {
        if (ev[i].crank && !ev[i - 1].crank && nr < 400) rises[nr++] = ev[i].t;
        if (ev[i].cam && !ev[i - 1].cam && nc < 8) cam_rises[nc++] = ev[i].t;
    }

    int normal = 0, gaps = 0, bad = 0;
    double worst = 0;
    for (int i = 1; i < nr; i++) {
        double us = (double)(rises[i] - rises[i - 1]) / 2.0;
        if (us > 150 && us < 183) {
            normal++;
            double err = us > 166.667 ? us - 166.667 : 166.667 - us;
            if (err > worst) worst = err;
        } else if (us > 480 && us < 520) {
            gaps++;
        } else {
            bad++;
        }
    }
    printf("6000 rpm: %d tooth intervals, %d gaps, %d bad, worst tooth error %.2f us; %d cam pulses\n",
           normal, gaps, bad, worst, nc);
    CHECK(bad == 0);
    CHECK(gaps == 6);                      /* 2 per 720° × 3 cycles */
    CHECK(normal >= 3 * 114);
    CHECK(worst <= 0.5);                   /* one 0.5 us tick of fractional carry */
    CHECK(nc == 3);
    if (nc >= 3) {
        uint64_t two = cam_rises[2] - cam_rises[0];
        printf("2 cam cycles = %llu ticks (expect exactly 80000)\n", (unsigned long long)two);
        CHECK(two == 80000);
    }

    printf("sim: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
