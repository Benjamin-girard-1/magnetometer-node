/**
 * @file ad7779_hal_stm32.c
 * @brief AD7779 HAL stub for STM32WB5MMG (HAL/LL drivers).
 *
 * This is a SKELETON. The function signatures match the ESP32 port so
 * the application code is identical across MCUs — only this file
 * (and the build system) changes.
 *
 * Recommended STM32WB5MMG mapping:
 *   SPI peripheral : SPI1 (or SPI2)
 *   DRDY pin       : EXTI line, falling edge IRQ
 *   CS pin         : GPIO output, software-managed (HAL_SPI_TransmitReceive
 *                    does not toggle NSS by default).
 *   DMA            : SPI1_RX + SPI1_TX channels for streaming
 *
 * Implementation notes:
 *  1. Use HAL_SPI_TransmitReceive (blocking) for register R/W.
 *  2. Use HAL_SPI_TransmitReceive_DMA for streaming, and post the
 *     completion callback from HAL_SPI_TxRxCpltCallback.
 *  3. The DRDY EXTI handler must call hal->drdy_cb from inside the
 *     HAL_GPIO_EXTI_Callback override.
 */

#include "ad7779_hal.h"
#include "ad7779_regs.h"

#include <string.h>

/* --- Replace with your CubeMX-generated includes when porting --- */
#if 0
#include "stm32wbxx_hal.h"
#include "main.h"          /* for SPIx and GPIO macros */
extern SPI_HandleTypeDef hspi1;
#define AD7779_SPI_HANDLE   (&hspi1)
#define AD7779_CS_PORT      ADC_CS_GPIO_Port
#define AD7779_CS_PIN       ADC_CS_Pin
#define AD7779_DRDY_PIN     ADC_DRDY_Pin
#endif

struct ad7779_hal_s {
    /* SPI_HandleTypeDef *spi; */
    ad7779_drdy_isr_cb_t  drdy_cb;
    void                 *drdy_ctx;
    ad7779_xfer_done_cb_t stream_done_cb;
    void                 *stream_done_ctx;
    volatile bool         stream_busy;
    uint8_t               nop_tx[AD7779_FRAME_BYTES_TOTAL];
};

static struct ad7779_hal_s s_hal_inst;

ad7779_hal_t *ad7779_hal_default_instance(void)
{
    return &s_hal_inst;
}

ad7779_hal_status_t ad7779_hal_init(ad7779_hal_t *hal)
{
    if (!hal) return AD7779_HAL_ERR_PARAM;
    memset(hal, 0, sizeof(*hal));
    for (size_t i = 0; i < AD7779_FRAME_BYTES_TOTAL; i += 2) {
        hal->nop_tx[i]     = AD7779_NOP_CMD_HI;
        hal->nop_tx[i + 1] = AD7779_NOP_CMD_LO;
    }

    /* TODO:
     *  - MX_SPIx_Init() must already have been called.
     *  - Configure CS GPIO (output, default high).
     *  - Configure DRDY EXTI (falling edge), but disable until streaming.
     */

    return AD7779_HAL_OK;
}

ad7779_hal_status_t ad7779_hal_deinit(ad7779_hal_t *hal)
{
    (void)hal;
    /* TODO: HAL_NVIC_DisableIRQ(EXTIx_IRQn); */
    return AD7779_HAL_OK;
}

ad7779_hal_status_t ad7779_hal_spi_xfer(ad7779_hal_t *hal,
                                        const uint8_t *tx,
                                        uint8_t *rx,
                                        size_t len)
{
    (void)hal; (void)tx; (void)rx; (void)len;
    /* TODO:
     *   HAL_GPIO_WritePin(AD7779_CS_PORT, AD7779_CS_PIN, GPIO_PIN_RESET);
     *   uint8_t scratch[8]; uint8_t *rxbuf = rx ? rx : scratch;
     *   HAL_StatusTypeDef hs = HAL_SPI_TransmitReceive(AD7779_SPI_HANDLE,
     *                          (uint8_t *)tx, rxbuf, len, HAL_MAX_DELAY);
     *   HAL_GPIO_WritePin(AD7779_CS_PORT, AD7779_CS_PIN, GPIO_PIN_SET);
     *   return (hs == HAL_OK) ? AD7779_HAL_OK : AD7779_HAL_ERR_BUS;
     */
    return AD7779_HAL_ERR_INTERNAL;
}

ad7779_hal_status_t ad7779_hal_spi_read_frame_async(ad7779_hal_t *hal,
                                                    uint8_t *rx_buf,
                                                    size_t len,
                                                    ad7779_xfer_done_cb_t done_cb,
                                                    void *done_ctx)
{
    if (!hal || !rx_buf || len == 0) return AD7779_HAL_ERR_PARAM;
    if (hal->stream_busy)             return AD7779_HAL_ERR_NOT_READY;

    hal->stream_done_cb  = done_cb;
    hal->stream_done_ctx = done_ctx;
    hal->stream_busy     = true;

    /* TODO:
     *   HAL_GPIO_WritePin(AD7779_CS_PORT, AD7779_CS_PIN, GPIO_PIN_RESET);
     *   HAL_SPI_TransmitReceive_DMA(AD7779_SPI_HANDLE,
     *                               hal->nop_tx, rx_buf, len);
     *
     * Then in HAL_SPI_TxRxCpltCallback:
     *   HAL_GPIO_WritePin(AD7779_CS_PORT, AD7779_CS_PIN, GPIO_PIN_SET);
     *   s_hal_inst.stream_busy = false;
     *   if (s_hal_inst.stream_done_cb)
     *       s_hal_inst.stream_done_cb(s_hal_inst.stream_done_ctx, AD7779_HAL_OK);
     */
    return AD7779_HAL_ERR_INTERNAL;
}

ad7779_hal_status_t ad7779_hal_attach_drdy_isr(ad7779_hal_t *hal,
                                               ad7779_drdy_isr_cb_t cb,
                                               void *user_ctx)
{
    if (!hal) return AD7779_HAL_ERR_PARAM;
    hal->drdy_cb  = cb;
    hal->drdy_ctx = user_ctx;
    /* The DRDY EXTI is wired in CubeMX. In HAL_GPIO_EXTI_Callback,
     * dispatch to s_hal_inst.drdy_cb when GPIO_Pin == AD7779_DRDY_PIN. */
    return AD7779_HAL_OK;
}

ad7779_hal_status_t ad7779_hal_drdy_enable(ad7779_hal_t *hal, bool enable)
{
    (void)hal; (void)enable;
    /* TODO: HAL_NVIC_EnableIRQ / DisableIRQ for the DRDY EXTI line. */
    return AD7779_HAL_OK;
}

void ad7779_hal_delay_us(ad7779_hal_t *hal, uint32_t us)
{
    (void)hal;
    /* TODO: replace with DWT cycle counter or HAL_Delay-based fallback. */
    volatile uint32_t i = us * 16;     /* very rough at 64 MHz */
    while (i--) { __asm__("nop"); }
}

void ad7779_hal_delay_ms(ad7779_hal_t *hal, uint32_t ms)
{
    (void)hal;
    /* TODO: HAL_Delay(ms); — or osDelay if using CMSIS-RTOS. */
    (void)ms;
}

uint32_t ad7779_hal_now_ms(ad7779_hal_t *hal)
{
    (void)hal;
    /* TODO: return HAL_GetTick(); */
    return 0;
}
