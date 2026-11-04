/*
 * Glue between UART and the frame parser/command handler. Non-blocking: a
 * response is pushed into the UART buffer a chunk at a time on each main-loop
 * pass, so a long reply never stalls engine calculations.
 */
#ifndef COMMS_COMMS_H
#define COMMS_COMMS_H

#include <stdint.h>

void comms_init(void);
void comms_poll(uint32_t now_ms);

#endif
