#include "comms/crc32.h"

#ifdef __AVR__
#include <avr/pgmspace.h>
#define TBL(i) pgm_read_dword(&crc_nibble[(i)])
#else
#define PROGMEM
#define TBL(i) crc_nibble[(i)]
#endif

/* 16-entry nibble table: 64 bytes of flash, ~2x slower than a 1 KB byte
 * table. CRC work only happens in the main loop, never in an ISR. */
static const uint32_t crc_nibble[16] PROGMEM = {
    0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
    0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
    0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
    0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL,
};

uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint16_t len)
{
    while (len--) {
        crc ^= *data++;
        crc = (crc >> 4) ^ TBL(crc & 0x0F);
        crc = (crc >> 4) ^ TBL(crc & 0x0F);
    }
    return crc;
}

uint32_t crc32(const uint8_t *data, uint16_t len)
{
    return ~crc32_update(0xFFFFFFFFUL, data, len);
}
