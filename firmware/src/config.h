/*
 * Build-wide configuration that is safe to include from host-side unit tests
 * (no AVR headers here).
 */
#ifndef CONFIG_H
#define CONFIG_H

/* Reported to the tuner. The signature changes whenever the tune/realtime
 * layout changes, so a tuner never writes to an ECU it does not understand. */
#define ECU_SIGNATURE   "E36ECU-dev-p2"
#define ECU_VERSION     "E36 ECU firmware 0.2.0 (phase 2: crank/cam decoder)"

/* Number of ADC channels scanned continuously (ADC0..ADC9, see board.h). */
#define ADC_NUM_CH      10

/* Largest framed payload accepted or produced by the protocol. */
#define FRAME_MAX_PAYLOAD 260

/* A partially received frame is discarded after this much silence. */
#define FRAME_TIMEOUT_MS  50

#endif
