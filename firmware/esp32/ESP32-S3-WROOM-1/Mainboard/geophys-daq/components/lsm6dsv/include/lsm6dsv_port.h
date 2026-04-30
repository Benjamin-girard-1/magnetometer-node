/**
 * @file lsm6dsv_port.h
 * @brief Hardware abstraction contract for the LSM6DSV driver.
 *
 * To support a new MCU, write ONE source file that implements these
 * three functions. The core driver (lsm6dsv.c) calls nothing else.
 *
 * Existing implementations:
 *   lsm6dsv_port_esp32.c   — ESP-IDF SPI master driver
 *   lsm6dsv_port_stm32.c   — (TODO when migrating to STM32WB5MMG)
 */

#ifndef LSM6DSV_PORT_H
#define LSM6DSV_PORT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize whatever is needed to talk SPI to the LSM6DSV.
 *
 * The implementation is responsible for:
 *   - Configuring the SPI peripheral (mode 3 — CPOL=1, CPHA=1; MSb-first;
 *     up to 10 MHz per datasheet §2 Table 5)
 *   - Configuring the CS GPIO as a push-pull output (or letting the SPI
 *     peripheral drive it as a hardware CS — both work)
 *   - Allocating any state it needs and returning an opaque pointer in
 *     @p out_ctx; the core driver passes that pointer back on every xfer.
 *
 * @param[out] out_ctx  opaque handle the implementation defines
 * @return 0 on success, negative on error
 */
int lsm6dsv_port_init(void **out_ctx);

/**
 * Full-duplex SPI transaction.
 *
 * The implementation MUST:
 *   1. Assert CS (drive low)
 *   2. Clock out @p len bytes from @p tx while clocking in @p len bytes
 *      into @p rx (either may be NULL — treat NULL tx as "send zeros",
 *      NULL rx as "discard read data")
 *   3. De-assert CS (drive high)
 *
 * The core driver places the command byte (R/W + address) at tx[0] and
 * the payload at tx[1..]. There is no implicit transfer size — pass
 * exactly what the driver gives you.
 *
 * @param ctx  the value returned by lsm6dsv_port_init()
 * @param tx   bytes to transmit (NULL = transmit zeros)
 * @param rx   buffer for received bytes (NULL = discard)
 * @param len  total number of bytes (always >= 2)
 * @return 0 on success, negative on error
 */
int lsm6dsv_port_xfer(void *ctx,
                      const uint8_t *tx,
                      uint8_t *rx,
                      size_t len);

/** Block for at least @p ms milliseconds. Used during reset/boot waits. */
void lsm6dsv_port_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSV_PORT_H */
