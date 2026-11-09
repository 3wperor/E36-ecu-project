/*
 * Realtime data sent to the tuner. Packed, little-endian (native AVR order).
 * Health, raw inputs and decoder state. Fuel/ignition/lambda channels are
 * added in later phases and will move into the generated definition
 * (definitions/ecu.yaml).
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
    uint16_t rpm;               /* 32 averaged over the last revolution */
    uint16_t rpm_inst;          /* 34 from the last tooth */
    uint8_t  sync;              /* 36 0 none, 1 pending, 2 crank (360 deg), 3 full (720 deg) */
    uint8_t  cam_pos;           /* 37 tooth where the cam edge falls (+60 = rev 1), 255 = none */
    uint8_t  loss_reason;       /* 38 1 tooth count, 2 no gap, 3 engine stopped */
    uint8_t  out_enable;        /* 39 bit 0 ignition, bit 1 injection enabled */
    uint16_t sync_losses;       /* 40 */
    uint16_t noise_teeth;       /* 42 rejected spikes */
    uint16_t cam_errors;        /* 44 cam phase did not alternate */
    uint16_t sched_late;        /* 46 output edges pulled forward (should stay ~0) */
    uint16_t sched_overlap;     /* 48 starts skipped: output still on (duty too high) */
} realtime_t;                   /* 50 bytes */

extern realtime_t rt;

#endif
