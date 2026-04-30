/**
 * @file ad7779_crc.h
 * @brief CRC-8 implementation for AD7779 SPI frames.
 *
 * Polynomial: x^8 + x^2 + x + 1 (0x07), seed 0x00, no reflection,
 * no final XOR.  Same as CRC-8/SMBUS.
 */

#ifndef AD7779_CRC_H_
#define AD7779_CRC_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Compute CRC-8 over @p data of length @p len. */
uint8_t ad7779_crc8(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* AD7779_CRC_H_ */
