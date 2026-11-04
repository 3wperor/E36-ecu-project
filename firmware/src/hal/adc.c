#include "hal/adc.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

static volatile uint16_t adc_raw[ADC_NUM_CH];
static volatile uint8_t  adc_idx;
static volatile uint16_t adc_scans;

static inline void adc_select(uint8_t ch)
{
    ADMUX  = (1 << REFS0) | (ch & 0x07);           /* AVCC reference */
    ADCSRB = (ch & 0x08) ? (1 << MUX5) : 0;        /* ADC8..15 */
}

ISR(ADC_vect)
{
    adc_raw[adc_idx] = ADC;
    uint8_t next = (uint8_t)(adc_idx + 1u);
    if (next >= ADC_NUM_CH) {
        next = 0;
        adc_scans++;
    }
    adc_idx = next;
    adc_select(next);
    ADCSRA |= (1 << ADSC);
}

void adc_init(void)
{
    adc_idx = 0;
    adc_select(0);
    /* Enable, interrupt, prescaler /128, start the first conversion. */
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADSC);
}

uint16_t adc_get(uint8_t channel)
{
    uint16_t v = 0;
    if (channel < ADC_NUM_CH) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = adc_raw[channel]; }
    }
    return v;
}

uint16_t adc_scan_count(void)
{
    uint16_t v;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = adc_scans; }
    return v;
}
