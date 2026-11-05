/*
 * Runs the firmware in simavr with UART0 on a pseudo-terminal and a simulated
 * engine on the crank/cam pins, so PC tools (tools/ecu_cli.py, later the
 * tuner) can be tested against the real firmware without hardware.
 *
 *   make sim-pty RPM=3000 SECONDS=20
 *   python3 tools/ecu_cli.py /tmp/simavr-uart0 watch
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sim_avr.h"
#include "sim_elf.h"
#include "sim_irq.h"
#include "avr_adc.h"
#include "avr_ioport.h"
#include "parts/uart_pty.h"

#include "pattern.h"

#define F_CPU 16000000UL

int main(int argc, char **argv)
{
    if (argc < 4) { fprintf(stderr, "usage: %s ecu.elf rpm seconds\n", argv[0]); return 2; }
    double rpm = atof(argv[2]);
    double seconds = atof(argv[3]);

    elf_firmware_t fw;
    memset(&fw, 0, sizeof fw);
    if (elf_read_firmware(argv[1], &fw)) return 2;
    avr_t *avr = avr_make_mcu_by_name("atmega1280");
    avr_init(avr);
    fw.frequency = F_CPU;
    avr_load_firmware(avr, &fw);
    avr->frequency = F_CPU;
    avr->log = 0;
    avr->avcc = avr->aref = 5000;

    static uart_pty_t pty;
    uart_pty_init(avr, &pty);
    uart_pty_connect(&pty, '0');

    avr_irq_t *crank = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('L'), 1);
    avr_irq_t *cam = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('L'), 0);
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC0), 1800);   /* MAF  */
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC1), 600);    /* TPS  */
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC3), 1500);   /* CLT  */
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC4), 2500);   /* IAT  */

    printf("simulated ECU on /tmp/simavr-uart0, engine %.0f rpm, %.0f s\n", rpm, seconds);
    fflush(stdout);

    double next = 16000, end = seconds * F_CPU;
    uint8_t step = 0, out = 0;
    while ((double)avr->cycle < end) {
        int s = avr_run(avr);
        if (s == cpu_Done || s == cpu_Crashed) break;
        if (rpm > 0) {
            while ((double)avr->cycle >= next) {
                step = (uint8_t)((step + 1) % STIM_STEPS);
                uint8_t n = stim_pattern(step, CAM_HALFMOON, 41);
                if ((n ^ out) & STIM_OUT_CRANK) avr_raise_irq(crank, n & STIM_OUT_CRANK);
                if ((n ^ out) & STIM_OUT_CAM) avr_raise_irq(cam, !!(n & STIM_OUT_CAM));
                out = n;
                next += 8e6 / rpm;
            }
        }
    }
    uart_pty_stop(&pty);
    return 0;
}
