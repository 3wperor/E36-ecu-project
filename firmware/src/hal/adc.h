/*
 * Background ADC scanner. The conversion-complete ISR stores the result and
 * starts the next channel, so the main loop always reads the latest values
 * without waiting. ADC clock = 125 kHz -> ~104 us per conversion, every
 * channel refreshed at ~960 Hz with 10 channels.
 *
 * Reference = AVCC, which on the ECU board is the same 5 V rail that feeds
 * the sensors (ratiometric).
 */
#ifndef HAL_ADC_H
#define HAL_ADC_H

#include <stdint.h>
#include "config.h"

void     adc_init(void);
uint16_t adc_get(uint8_t channel);   /* 0..1023, latest sample */
uint16_t adc_scan_count(void);       /* completed full scans (wraps) */

#endif
