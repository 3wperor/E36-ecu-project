/*
 * Realtime data sent to the tuner. Packed, little-endian (native AVR order).
 * Phase 1 content: health and raw inputs. Engine channels (RPM, sync, PW,
 * advance, lambda...) are added in later phases and will move into the
 * generated definition (definitions/ecu.yaml).
 *
 * Any change here must change ECU_SIGNATURE in config.h.
 */
#ifndef STATUS_H
#define STATUS_H

#include <stdint.h>
#include "config.h"

#define RT_FLAG_TIMERS_SYNCED  0x01

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;         /*  0 */
    uint16_t loops_per_sec;     /*  4 main-loop passes in the last second */
    uint8_t  reset_cause;       /*  6 MCUSR at boot */
    uint8_t  flags;             /*  7 RT_FLAG_* */
    uint16_t adc[ADC_NUM_CH];   /*  8 raw 0..1023, index = enum adc_channel */
    uint16_t uart_overflows;    /* 28 */
    uint16_t frame_errors;      /* 30 bad CRC / bad length */
} realtime_t;                   /* 32 bytes */

extern realtime_t rt;

#endif
