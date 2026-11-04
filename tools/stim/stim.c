/*
 * Trigger simulator for an Arduino Uno/Nano (ATmega328P @ 16 MHz).
 * Generates the M52 60-2 crank signal and a cam signal as 5 V square waves,
 * i.e. what the ECU sees after its VR/Hall conditioning.
 *
 * Wiring to the ECU dev board (Arduino Mega):
 *   stim D9  (PB1, OC1A) crank  ->  Mega D48 (ICP5)
 *   stim D10 (PB2, OC1B) cam    ->  Mega D49 (ICP4)
 *   GND                         ->  GND
 *
 * Edges are produced by Timer1 output-compare hardware (the same technique
 * the ECU uses for injectors and coils), so they have no interrupt jitter.
 * The ISR only prepares the next edge. Step periods carry their fractional
 * remainder, so the average speed is exact.
 *
 * Serial commands, 115200 baud, one per line:
 *   r<rpm>               constant speed (0 stops the engine)       r3000
 *   s<from>,<to>,<ms>    linear sweep, then hold at <to>           s800,7200,5000
 *   c<0|1|2>             cam: 0 none, 1 single pulse, 2 half-moon  c2
 *   o<step>              cam start position, half-steps 0..239     o40
 *   d                    drop one crank tooth (decoder fault test)
 *   ?                    print current settings
 */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <util/atomic.h>

#include "pattern.h"

#define OUT_MASK  ((1 << PB1) | (1 << PB2))
#define RPM_MIN   20u
#define RPM_MAX   9000u

/* Step period = 1,000,000 / rpm ticks of 0.5 us, as quotient + remainder. */
static volatile uint16_t per_q, per_r, per_den;
static uint16_t per_acc;
static volatile uint8_t  cam_mode = CAM_HALFMOON;
static volatile uint8_t  cam_start = 40;
static volatile uint8_t  drop_request;
static volatile uint32_t ms;

static uint8_t step;
static uint16_t rpm_now;

/* COM1A/COM1B: 0b11 = set pin on compare match, 0b10 = clear pin. */
static inline uint8_t com_bits(uint8_t out)
{
    return (uint8_t)((1 << COM1A1) | ((out & STIM_OUT_CRANK) ? (1 << COM1A0) : 0) |
                     (1 << COM1B1) | ((out & STIM_OUT_CAM) ? (1 << COM1B0) : 0));
}

static inline uint16_t next_period(void)
{
    uint16_t p = per_q;
    per_acc += per_r;
    if (per_acc >= per_den) {
        per_acc -= per_den;
        p++;
    }
    return p;
}

/* ---- pattern generation ------------------------------------------------ */

/* Fires when the hardware has just produced the edges for `step`.
 * Arm the edges for the following step. */
ISR(TIMER1_COMPA_vect)
{
    if (++step >= STIM_STEPS) {
        step = 0;
    }
    uint8_t o = stim_pattern(step, cam_mode, cam_start);
    if (drop_request && (o & STIM_OUT_CRANK)) {
        o &= (uint8_t)~STIM_OUT_CRANK;
        drop_request = 0;
    }
    uint16_t t = (uint16_t)(OCR1A + next_period());
    TCCR1A = com_bits(o);
    OCR1A = t;
    OCR1B = t;
}

ISR(TIMER0_COMPA_vect)
{
    ms++;
}

static uint32_t millis(void)
{
    uint32_t v;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = ms; }
    return v;
}

static void outputs_force_low(void)
{
    TCCR1A = com_bits(0);
    TCCR1C = (1 << FOC1A) | (1 << FOC1B);   /* apply "clear" immediately */
}

static void set_rpm(uint16_t rpm)
{
    if (rpm == 0) {
        TCCR1B = 0;                         /* clock off */
        outputs_force_low();
        rpm_now = 0;
        return;
    }
    if (rpm < RPM_MIN) rpm = RPM_MIN;
    if (rpm > RPM_MAX) rpm = RPM_MAX;

    uint16_t q = (uint16_t)(1000000UL / rpm);
    uint16_t r = (uint16_t)(1000000UL % rpm);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        per_q = q; per_r = r; per_den = rpm;
        if (per_acc >= per_den) per_acc = 0;
        if (rpm_now == 0) {                 /* (re)start: first edge = step 0 */
            outputs_force_low();
            step = 0;
            per_acc = 0;
            TCNT1 = 0;
            OCR1A = OCR1B = 100;            /* first edge 50 us from now */
            TCCR1A = com_bits(stim_pattern(0, cam_mode, cam_start));
            TCCR1B = (1 << CS11);           /* normal mode, /8 -> 0.5 us ticks */
        }
    }
    rpm_now = rpm;
}

/* ---- serial ------------------------------------------------------------ */

static void uart_init(void)
{
    UBRR0  = 16;                            /* 115200 with U2X @ 16 MHz */
    UCSR0A = (1 << U2X0);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void putc_(char c)
{
    while (!(UCSR0A & (1 << UDRE0))) { }
    UDR0 = (uint8_t)c;
}

static void puts_(const char *s)
{
    while (*s) putc_(*s++);
}

static void putu(uint32_t v)
{
    char b[11];
    ultoa(v, b, 10);
    puts_(b);
}

static void print_status(void)
{
    puts_("rpm=");  putu(rpm_now);
    puts_(" cam="); putu(cam_mode);
    puts_(" cam_start="); putu(cam_start);
    puts_("\r\n");
}

/* ---- main -------------------------------------------------------------- */

int main(void)
{
    DDRB  |= OUT_MASK;
    PORTB &= (uint8_t)~OUT_MASK;

    TCCR0A = (1 << WGM01);                  /* 1 kHz tick */
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A  = 249;
    TIMSK0 = (1 << OCIE0A);

    TCCR1B = 0;                             /* Timer1 stopped until set_rpm() */
    outputs_force_low();
    TIMSK1 = (1 << OCIE1A);

    uart_init();
    sei();

    set_rpm(1000);
    puts_("M52 trigger simulator ready\r\n");
    print_status();

    char line[24];
    uint8_t len = 0;
    uint8_t sweeping = 0;
    uint16_t sw_from = 0, sw_to = 0;
    uint32_t sw_start = 0, sw_dur = 1, last_ms = 0;

    for (;;) {
        uint32_t now = millis();
        if (sweeping && now != last_ms) {
            last_ms = now;
            uint32_t el = now - sw_start;
            if (el >= sw_dur) {
                set_rpm(sw_to);
                sweeping = 0;
            } else {
                int32_t d = (int32_t)sw_to - (int32_t)sw_from;
                set_rpm((uint16_t)((int32_t)sw_from + d * (int32_t)el / (int32_t)sw_dur));
            }
        }

        if (!(UCSR0A & (1 << RXC0))) {
            continue;
        }
        char c = (char)UDR0;
        if (c != '\n' && c != '\r') {
            if (len < sizeof line - 1) line[len++] = c;
            continue;
        }
        if (len == 0) {
            continue;
        }
        line[len] = 0;
        len = 0;

        char *p = &line[1];
        uint8_t ok = 1;
        switch (line[0]) {
        case 'r':
            sweeping = 0;
            set_rpm((uint16_t)strtoul(p, NULL, 10));
            break;
        case 's': {
            char *e;
            sw_from = (uint16_t)strtoul(p, &e, 10);
            if (*e != ',') { ok = 0; break; }
            sw_to = (uint16_t)strtoul(e + 1, &e, 10);
            if (*e != ',') { ok = 0; break; }
            sw_dur = strtoul(e + 1, NULL, 10);
            if (sw_dur == 0) sw_dur = 1;
            sw_start = millis();
            set_rpm(sw_from);
            sweeping = 1;
            break;
        }
        case 'c': {
            uint8_t m = (uint8_t)strtoul(p, NULL, 10);
            if (m > CAM_HALFMOON) { ok = 0; break; }
            cam_mode = m;
            break;
        }
        case 'o': {
            unsigned long s = strtoul(p, NULL, 10);
            if (s >= STIM_STEPS) { ok = 0; break; }
            cam_start = (uint8_t)s;
            break;
        }
        case 'd':
            drop_request = 1;
            break;
        case '?':
            break;
        default:
            ok = 0;
        }
        puts_(ok ? "ok " : "err ");
        print_status();
    }
}
