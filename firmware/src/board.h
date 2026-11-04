/*
 * Pin map for the ATmega1280/2560. The development target is a stock Arduino
 * Mega board; the custom PCB will keep the same MCU pins. See docs/PLAN.md §2.1.
 *
 * Outputs are numbered by CYLINDER, not by firing order.
 */
#ifndef BOARD_H
#define BOARD_H

#include <avr/io.h>

/* ---- Injectors: hardware output compare pins (Timer1, Timer3) ---------- */
/* INJ1 OC1A PB5 (D11), INJ2 OC1B PB6 (D12), INJ3 OC1C PB7 (D13)            */
/* INJ4 OC3A PE3 (D5),  INJ5 OC3B PE4 (D2),  INJ6 OC3C PE5 (D3)             */
#define INJ_1_3_DDR   DDRB
#define INJ_1_3_PORT  PORTB
#define INJ_1_3_MASK  ((1 << PB5) | (1 << PB6) | (1 << PB7))
#define INJ_4_6_DDR   DDRE
#define INJ_4_6_PORT  PORTE
#define INJ_4_6_MASK  ((1 << PE3) | (1 << PE4) | (1 << PE5))

/* ---- Ignition coils: hardware output compare pins (Timer4, Timer5) ----- */
/* IGN1 OC4A PH3 (D6),  IGN2 OC4B PH4 (D7),  IGN3 OC4C PH5 (D8)             */
/* IGN4 OC5A PL3 (D46), IGN5 OC5B PL4 (D45), IGN6 OC5C PL5 (D44)            */
#define IGN_1_3_DDR   DDRH
#define IGN_1_3_PORT  PORTH
#define IGN_1_3_MASK  ((1 << PH3) | (1 << PH4) | (1 << PH5))
#define IGN_4_6_DDR   DDRL
#define IGN_4_6_PORT  PORTL
#define IGN_4_6_MASK  ((1 << PL3) | (1 << PL4) | (1 << PL5))

/* ---- Trigger inputs ------------------------------------------------------ */
/* Crank: ICP5 PL1 (D48). Cam: ICP4 PL0 (D49). Conditioned externally.      */
#define TRIG_DDR      DDRL
#define TRIG_PORT     PORTL
#define TRIG_PIN      PINL
#define CRANK_BIT     PL1
#define CAM_BIT       PL0

/* ---- Idle valve: two coils on Timer2 OC2A PB4 (D10) / OC2B PH6 (D9) ---- */
#define IAC_OPEN_DDR   DDRB
#define IAC_OPEN_PORT  PORTB
#define IAC_OPEN_BIT   PB4
#define IAC_CLOSE_DDR  DDRH
#define IAC_CLOSE_PORT PORTH
#define IAC_CLOSE_BIT  PH6

/* ---- Low-side auxiliary outputs: Port A (D22..D29) ---------------------- */
#define AUX_DDR        DDRA
#define AUX_PORT       PORTA
#define AUX_FUEL_PUMP  0   /* PA0 D22 */
#define AUX_MAIN_RELAY 1   /* PA1 D23 */
#define AUX_VANOS      2   /* PA2 D24 */
#define AUX_NB_HEAT1   3   /* PA3 D25 */
#define AUX_NB_HEAT2   4   /* PA4 D26 */
#define AUX_TACH       5   /* PA5 D27 */
#define AUX_WARN_LAMP  6   /* PA6 D28 */
#define AUX_AC_CUT     7   /* PA7 D29 */

/* ---- More outputs: Port C ----------------------------------------------- */
#define AUX2_DDR       DDRC
#define AUX2_PORT      PORTC
#define AUX2_PURGE     0   /* PC0 D37 */
#define AUX2_TI_OUT    1   /* PC1 D36 fuel consumption signal to cluster */
#define AUX2_KNOCK_HOLD 2  /* PC2 D35 TPIC8101 INT/HOLD (low = hold) */

/* ---- Knock IC (TPIC8101) on SPI: CS = PB0 (D53, also SPI SS) ------------- */
#define KNOCK_CS_DDR   DDRB
#define KNOCK_CS_PORT  PORTB
#define KNOCK_CS_BIT   PB0

/* ---- Road speed input: INT0 PD0 (D21) ------------------------------------ */
#define VSS_DDR        DDRD
#define VSS_BIT        PD0

/* ---- Analogue inputs (index = ADC channel) ------------------------------- */
enum adc_channel {
    ADC_MAF   = 0,  /* A0 */
    ADC_TPS1  = 1,  /* A1 */
    ADC_TPS2  = 2,  /* A2 */
    ADC_CLT   = 3,  /* A3 */
    ADC_IAT   = 4,  /* A4 */
    ADC_WBO2  = 5,  /* A5 wideband controller 0-5 V */
    ADC_NBO2A = 6,  /* A6 narrowband bank 1 (cyl 1-3) */
    ADC_NBO2B = 7,  /* A7 narrowband bank 2 (cyl 4-6) */
    ADC_VBAT  = 8,  /* A8 battery divider */
    ADC_MAP   = 9,  /* A9 optional balance-rail MAP */
};

#endif
