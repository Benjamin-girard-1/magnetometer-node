/**
 * @file scl3300_port.h
 * @brief Hardware abstraction contract for the SCL3300 driver.
 *
 * Same three-function shape as the LSM6DSV port. Implementations:
 *   scl3300_port_esp32.c   — ESP-IDF SPI master
 *   scl3300_port_stm32.c   — (TODO when migrating to STM32WB5MMG)
 *
 * Note: SCL3300 has *different* SPI requirements than the LSM6DSV:
 *   - SPI mode 0 (CPOL=0, CPHA=0)         [LSM6DSV uses mode 3]
 *   - 2–4 MHz recommended, 8 MHz absolute max
 *   - 32-bit transactions only
 *   - Minimum 10 µs CSB-high time between transactions
 *
 * The implementation is responsible for enforcing the 10 µs gap.
 */

#ifndef SCL3300_PORT_H
#define SCL3300_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Configure the SPI peripheral and CS GPIO for the SCL3300.
 *
 * @param[out] out_ctx  opaque handle the implementation defines
 * @return 0 on success, negative on error
 */
int scl3300_port_init(void **out_ctx);

/**
 * Send one 32-bit frame and receive the 32-bit response that came back
 * during the same CSB-low window.
 *
 * The implementation MUST:
 *   1. Wait until at least 10 µs have elapsed since the previous CSB rise
 *   2. Assert CSB (drive low)
 *   3. Clock 32 bits MSB-first: TX = @p tx_frame, RX captured into @p rx_frame
 *   4. De-assert CSB (drive high)
 *
 * @param ctx        the value returned by scl3300_port_init()
 * @param tx_frame   32-bit frame to transmit (already includes CRC)
 * @param rx_frame   pointer to receive the 32-bit response
 * @return 0 on success, negative on error
 */
int scl3300_port_xfer(void *ctx, uint32_t tx_frame, uint32_t *rx_frame);

/** Block for at least @p ms milliseconds. */
void scl3300_port_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* SCL3300_PORT_H */
