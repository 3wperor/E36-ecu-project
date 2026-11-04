/*
 * End-to-end test: run the real firmware ELF in simavr, talk to it over the
 * emulated UART0 and check the replies. Also drives simulated ADC voltages.
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
#include "avr_uart.h"

#include "comms/crc32.h"

#define F_CPU 16000000UL

static avr_t *avr;
static avr_irq_t *uart_in;
static uint8_t rx[1024];
static size_t rx_len;
static int failures, checks;

#define CHECK(cond) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL line %d: %s\n", __LINE__, #cond); } \
} while (0)

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

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: %s ecu.elf mcu\n", argv[0]); return 2; }

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

    /* Sensor voltages (mV, AVCC = 5 V): MAF 2.5 V, CLT 1.25 V. */
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC0), 2500);
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC3), 1250);

    /* Let it boot and run for 1.2 s so the 1 Hz task has run. */
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

    send_frame((const uint8_t *)"A", 1, 0);
    n = recv_frame(p, 50000);
    CHECK(n == 33 && p[0] == 0x00);
    if (n == 33) {
        const uint8_t *d = &p[1];
        uint32_t up = le32(d);
        uint16_t loops = le16(d + 4);
        uint16_t maf = le16(d + 8), clt = le16(d + 8 + 2 * 3);
        printf("uptime %u ms, main loop %u passes/s, reset cause 0x%02X, flags 0x%02X, "
               "MAF adc %u, CLT adc %u\n", up, loops, d[6], d[7], maf, clt);
        CHECK(up >= 1200 && up < 1400);  /* no watchdog reset during the run */
        CHECK(loops > 1000);
        CHECK(maf >= 510 && maf <= 513);  /* 2.5 V -> 511/512 */
        CHECK(clt >= 254 && clt <= 257);  /* 1.25 V -> 255/256 */
    }

    send_frame((const uint8_t *)"A", 1, 1);   /* corrupted CRC */
    n = recv_frame(p, 50000);
    CHECK(n == 1 && p[0] == 0x82);

    send_frame((const uint8_t *)"?", 1, 0);   /* unknown command */
    n = recv_frame(p, 50000);
    CHECK(n == 1 && p[0] == 0x83);

    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
