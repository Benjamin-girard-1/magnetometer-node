/**
 * @file lsm6dsv_port_stm32.c
 * @brief STM32 (HAL) implementation of the LSM6DSV port layer — STUB.
 *
 * Fill this in when migrating to the STM32WB5MMG. Then in your component's
 * CMakeLists.txt, swap which port source is compiled.
 *
 * What CubeMX should generate before you write this:
 *   - SPIx peripheral configured: master, full-duplex, 8-bit, MSB-first,
 *     CPOL=High, CPHA=2nd edge (= "SPI mode 3").
 *     Prescaler tuned so SCK <= 10 MHz (e.g. APB / 8 with a 64 MHz APB).
 *   - One GPIO output for CS (push-pull, no pull, high speed).
 *
 * Replace SPI_HANDLE / CS_PORT / CS_PIN with your CubeMX names.
 */

#if 0   /* <-- enable when building for STM32 */

#include "lsm6dsv_port.h"
#include "main.h"        /* CubeMX-generated handle macros                  */

#include <stdlib.h>

#define LSM6DSV_SPI_HANDLE   (&hspi1)
#define LSM6DSV_CS_PORT      GPIOA
#define LSM6DSV_CS_PIN       GPIO_PIN_4

typedef struct {
    int dummy; /* nothing per-instance to track on STM32 — kept for symmetry */
} lsm6dsv_stm32_ctx_t;

static inline void cs_low (void) { HAL_GPIO_WritePin(LSM6DSV_CS_PORT, LSM6DSV_CS_PIN, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(LSM6DSV_CS_PORT, LSM6DSV_CS_PIN, GPIO_PIN_SET);   }

int lsm6dsv_port_init(void **out_ctx)
{
    /* Bus + CS pin are configured by CubeMX-generated MX_SPIx_Init() and   */
    /* MX_GPIO_Init() — both run before lsm6dsv_init() is ever called. So   */
    /* there's nothing to do here other than allocate the context.          */
    lsm6dsv_stm32_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return -1;
    cs_high();
    *out_ctx = ctx;
    return 0;
}

int lsm6dsv_port_xfer(void *ctx_, const uint8_t *tx, uint8_t *rx, size_t len)
{
    (void)ctx_;
    if (len == 0) return -1;

    /* HAL_SPI_TransmitReceive needs both pointers non-NULL. If the caller   */
    /* passed NULL for one direction, swap in a scratch buffer. The driver  */
    /* never asks for >33 bytes (1 cmd + 32 data), so 64 is comfortable.    */
    uint8_t scratch_tx[64] = {0};
    uint8_t scratch_rx[64];
    const uint8_t *tx_ptr = tx ? tx : scratch_tx;
    uint8_t       *rx_ptr = rx ? rx : scratch_rx;

    cs_low();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(
        LSM6DSV_SPI_HANDLE, (uint8_t *)tx_ptr, rx_ptr, (uint16_t)len, 100);
    cs_high();
    return (st == HAL_OK) ? 0 : -1;
}

void lsm6dsv_port_delay_ms(uint32_t ms) { HAL_Delay(ms); }

#endif /* 0 */
