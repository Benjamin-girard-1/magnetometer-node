#include "ad7779_crc.h"

/**
 * Bit-by-bit CRC-8/SMBUS implementation.
 * Polynomial 0x07, init 0x00, no reflection, no final XOR.
 *
 * For an 8-channel streaming frame with CRC enabled (40 bytes),
 * this is called 8 times on 4-byte chunks => 32 byte-iterations
 * = 256 inner-loop iterations per frame. At 1 kSPS that's 256k
 * inner iterations per second, easily a fraction of 1% CPU on
 * the ESP32-S3. A table-based version is unnecessary.
 */
uint8_t ad7779_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x07);
            else            crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}
