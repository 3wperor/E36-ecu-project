#ifndef COMMS_CRC32_H
#define COMMS_CRC32_H

#include <stdint.h>

/* Standard CRC-32 (IEEE 802.3, reflected 0xEDB88320), as used by the
 * TunerStudio-style serial framing. crc32("123456789") == 0xCBF43926. */
uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint16_t len);
uint32_t crc32(const uint8_t *data, uint16_t len);

#endif
