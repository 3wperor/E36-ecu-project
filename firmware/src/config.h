/*
 * Build-wide configuration that is safe to include from host-side unit tests
 * (no AVR headers here).
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ECU_SIGNATURE / ECU_VERSION come from definitions/ecu.toml. The signature
 * changes whenever the tune/realtime layout changes, so a tuner never writes
 * to an ECU it does not understand. */
#include "generated/tune_def.h"

/* Number of ADC channels scanned continuously (ADC0..ADC9, see board.h). */
#define ADC_NUM_CH      10

/* Largest framed payload accepted or produced by the protocol. */
#define FRAME_MAX_PAYLOAD 260

/* A partially received frame is discarded after this much silence. */
#define FRAME_TIMEOUT_MS  50

#endif
