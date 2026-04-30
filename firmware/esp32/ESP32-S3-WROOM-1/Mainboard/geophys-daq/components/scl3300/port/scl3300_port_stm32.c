/**
 * @file scl3300_port_stm32.c
 * @brief STM32 (HAL) implementation of the SCL3300 port layer — STUB.
 *
 * Fill this in when migrating to the STM32WB5MMG. Then in your component's
 * CMakeLists.txt, swap which port source is compiled.
 *
 * What CubeMX should generate before you write this:
 *   - SPIx peripheral configured: master, full-duplex, 8-bit, MSB-first,
 *     CPOL=Low, CPHA=1st edge (= "SPI mode 0"). Note this is DIFFERENT
 *     from the LSM6DSV which uses mode 3 — if both share an STM32 SPI
 *     peripheral, you'll need to reconfigure mode between transactions
 *     or use two different peripherals.
 *     Prescaler tuned so SCK is in the 2–4 MHz range (e.g. APB / 16 with
 *     a 64 MHz APB gives 4 MHz).
 *   - One GPIO output for CS (push-pull, no pull, high speed).
 *   - A microsecond timebase (DWT cycle counter or TIMx) for the 10 µs
 *     inter-transaction gap.
 *
 * Replace SPI_HANDLE / CS_PORT / CS_PIN with your CubeMX names.
 */

#if 0   /* <-- enable when building for STM32 */

#include "scl3300_port.h"
#include "main.h"        /* CubeMX-generated handle macros                  */

#include <stdlib.h>

#define SCL3300_SPI_HANDLE   (&hspi1)
#define SCL3300_CS_PORT      GPIOA
#define SCL3300_CS_PIN       GPIO_PIN_5
#define SCL3300_TLH_US       (10)

typedef struct {
    uint32_t last_xfer_end_cyc;  /* DWT->CYCCNT at end of last xfer         */
} scl3300_stm32_ctx_t;

static inline void cs_low (void) { HAL_GPIO_WritePin(SCL3300_CS_PORT, SCL3300_CS_PIN, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(SCL3300_CS_PORT, SCL3300_CS_PIN, GPIO_PIN_SET);   }

/* Microsecond busy-wait using DWT. Make sure DWT->CTRL is enabled in main.c */
static void delay_us(uint32_t us)
{
    extern uint32_t SystemCoreClock;
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < cycles) { /* spin */ }
}

int scl3300_port_init(void **out_ctx)
{
    /* Bus + CS pin are configured by CubeMX before this function runs.      */
    /* Make sure DWT is enabled somewhere (typically in main.c after        */
    /* HAL_Init): CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;           */
    /*            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                      */
    scl3300_stm32_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return -1;
    cs_high();
    ctx->last_xfer_end_cyc = 0;
    *out_ctx = ctx;
    return 0;
}

int scl3300_port_xfer(void *ctx_, uint32_t tx_frame, uint32_t *rx_frame)
{
    scl3300_stm32_ctx_t *ctx = (scl3300_stm32_ctx_t *)ctx_;
    if (!ctx || !rx_frame) return -1;

    /* Enforce 10 µs CSB-high gap.                                           */
    if (ctx->last_xfer_end_cyc != 0) {
        extern uint32_t SystemCoreClock;
        uint32_t needed = SCL3300_TLH_US * (SystemCoreClock / 1000000U);
        while ((DWT->CYCCNT - ctx->last_xfer_end_cyc) < needed) { /* spin */ }
    }

    uint8_t tx[4] = {
        (uint8_t)(tx_frame >> 24),
        (uint8_t)(tx_frame >> 16),
        (uint8_t)(tx_frame >>  8),
        (uint8_t)(tx_frame      ),
    };
    uint8_t rx[4] = {0};

    cs_low();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(
        SCL3300_SPI_HANDLE, tx, rx, 4, 100);
    cs_high();
    if (st != HAL_OK) return -1;

    *rx_frame = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16)
              | ((uint32_t)rx[2] <<  8) |  (uint32_t)rx[3];
    ctx->last_xfer_end_cyc = DWT->CYCCNT;
    return 0;
}

void scl3300_port_delay_ms(uint32_t ms) { HAL_Delay(ms); }

#endif /* 0 */
