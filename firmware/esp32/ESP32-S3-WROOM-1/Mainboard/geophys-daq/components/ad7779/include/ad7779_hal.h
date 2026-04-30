/**
 * @file ad7779_hal.h
 * @brief Hardware abstraction for the AD7779 driver.
 *
 * To port to a new MCU, implement the functions declared here in a
 * platform-specific .c file (see port/ad7779_hal_esp32.c for the
 * reference ESP32-S3 implementation, and port/ad7779_hal_stm32.c
 * for the STM32WB5MMG stub).
 *
 * The driver core never calls any vendor SDK directly — it only
 * calls these functions. This is the entire portability boundary.
 */

#ifndef AD7779_HAL_H_
#define AD7779_HAL_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Status returned by HAL operations. */
typedef enum {
    AD7779_HAL_OK              = 0,
    AD7779_HAL_ERR_PARAM       = -1,
    AD7779_HAL_ERR_BUS         = -2,
    AD7779_HAL_ERR_TIMEOUT     = -3,
    AD7779_HAL_ERR_NOT_READY   = -4,
    AD7779_HAL_ERR_INTERNAL    = -5,
} ad7779_hal_status_t;

/** Opaque HAL handle. The actual struct is defined in the port file. */
typedef struct ad7779_hal_s ad7779_hal_t;

/** DRDY interrupt callback. Runs in ISR context — keep it short. */
typedef void (*ad7779_drdy_isr_cb_t)(void *user_ctx);

/* ===========================================================
 * Lifecycle
 * =========================================================== */

/**
 * Initialize the HAL: configure SPI peripheral and GPIOs.
 * Does NOT touch the AD7779 itself.
 */
ad7779_hal_status_t ad7779_hal_init(ad7779_hal_t *hal);

/** Release HAL resources. */
ad7779_hal_status_t ad7779_hal_deinit(ad7779_hal_t *hal);

/* ===========================================================
 * SPI register-mode transfers (CS asserted around each call)
 *
 * These are used for low-rate config R/W. SPI Mode 0 (CPOL=0,
 * CPHA=0). The AD7779 also supports Mode 3 — Mode 0 is fine.
 * =========================================================== */

/**
 * Full-duplex transfer with CS auto-managed.
 * @param tx  Bytes to send (must not be NULL).
 * @param rx  Buffer for received bytes (NULL = discard).
 * @param len Number of bytes to transfer.
 */
ad7779_hal_status_t ad7779_hal_spi_xfer(ad7779_hal_t *hal,
                                        const uint8_t *tx,
                                        uint8_t *rx,
                                        size_t len);

/* ===========================================================
 * SPI streaming (Σ-Δ readback) — DMA-friendly
 *
 * Triggered by DRDY ISR. Reads AD7779_FRAME_BYTES_TOTAL = 32
 * bytes per frame while sending the AD7779 NOP pattern (0x8000…)
 * to avoid spurious register writes. Implementation should use
 * DMA where available.
 * =========================================================== */

/**
 * Start a one-shot frame read (typically called from DRDY ISR or a
 * task woken by it). Buffer must remain valid until the completion
 * callback fires.
 *
 * @param rx_buf       Buffer to fill (size = AD7779_FRAME_BYTES_TOTAL).
 * @param len          Frame size in bytes.
 * @param done_cb      Optional callback invoked when transfer finishes.
 * @param done_ctx     User context passed to done_cb.
 */
typedef void (*ad7779_xfer_done_cb_t)(void *user_ctx,
                                      ad7779_hal_status_t status);

ad7779_hal_status_t ad7779_hal_spi_read_frame_async(ad7779_hal_t *hal,
                                                    uint8_t *rx_buf,
                                                    size_t len,
                                                    ad7779_xfer_done_cb_t done_cb,
                                                    void *done_ctx);

/* ===========================================================
 * DRDY interrupt
 * =========================================================== */

/** Register a callback fired on DRDY falling edge. */
ad7779_hal_status_t ad7779_hal_attach_drdy_isr(ad7779_hal_t *hal,
                                               ad7779_drdy_isr_cb_t cb,
                                               void *user_ctx);

ad7779_hal_status_t ad7779_hal_drdy_enable(ad7779_hal_t *hal, bool enable);

/* ===========================================================
 * Misc
 * =========================================================== */

/** Sleep / busy-wait. Implementation may use scheduler delay. */
void ad7779_hal_delay_us(ad7779_hal_t *hal, uint32_t us);
void ad7779_hal_delay_ms(ad7779_hal_t *hal, uint32_t ms);

/** Monotonic millisecond counter, for timeouts. */
uint32_t ad7779_hal_now_ms(ad7779_hal_t *hal);

/**
 * Returns the singleton HAL instance for this platform.
 * Implemented in the per-platform port file (ad7779_hal_esp32.c
 * or ad7779_hal_stm32.c).
 */
ad7779_hal_t *ad7779_hal_default_instance(void);

#ifdef __cplusplus
}
#endif

#endif /* AD7779_HAL_H_ */
