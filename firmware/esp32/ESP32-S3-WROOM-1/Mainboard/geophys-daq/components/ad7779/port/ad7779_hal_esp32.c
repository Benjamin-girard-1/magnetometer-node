/**
 * @file ad7779_hal_esp32.c
 * @brief AD7779 HAL implementation for ESP32-S3 / ESP-IDF.
 *
 * Pinout (per project schematic):
 *   ADC_DRDY -> GPIO10  (input, falling edge IRQ)
 *   ADC_CS   -> GPIO11  (output, active low — manually controlled)
 *   ADC_SCLK -> GPIO12
 *   ADC_SDI  <- GPIO13  (ESP MOSI -> AD7779 SDI)
 *   ADC_SDO  -> GPIO14  (ESP MISO <- AD7779 SDO)
 *
 * Architecture:
 *   - Manual CS via GPIO (the SPI peripheral's hardware CS routing
 *     proved unreliable on this board)
 *   - Register R/W: blocking spi_device_polling_transmit
 *   - Streaming: a dedicated FreeRTOS task waits on a semaphore that
 *     the DRDY ISR gives. The task does a polling SPI read of one
 *     full frame, then calls back. Simple, low-jitter, no DMA queue.
 */

#include "ad7779_hal.h"
#include "ad7779_regs.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <string.h>

#define TAG "ad7779_hal"

#define AD7779_PIN_DRDY   GPIO_NUM_10
#define AD7779_PIN_CS     GPIO_NUM_11
#define AD7779_PIN_SCLK   GPIO_NUM_12
#define AD7779_PIN_MOSI   GPIO_NUM_13
#define AD7779_PIN_MISO   GPIO_NUM_14

#define AD7779_SPI_HOST   SPI2_HOST

#define AD7779_SPI_CLOCK_REG_HZ    (8 * 1000 * 1000)
#define AD7779_SPI_CLOCK_STREAM_HZ (8 * 1000 * 1000)

#define NOP_BUF_LEN   AD7779_FRAME_BYTES_TOTAL

#define STREAM_TASK_STACK   4096
#define STREAM_TASK_PRIO    (configMAX_PRIORITIES - 2)

struct ad7779_hal_s {
    spi_device_handle_t   reg_dev;
    spi_device_handle_t   stream_dev;
    bool                  bus_owned;

    ad7779_drdy_isr_cb_t  drdy_cb;
    void                 *drdy_ctx;
    bool                  drdy_isr_installed;

    TaskHandle_t          stream_task;
    SemaphoreHandle_t     drdy_sem;
    SemaphoreHandle_t     bus_mutex;
    volatile bool         stream_enabled;

    uint8_t               nop_tx[NOP_BUF_LEN];
    uint8_t              *cur_rx_buf;
    size_t                cur_len;
    ad7779_xfer_done_cb_t cur_done_cb;
    void                 *cur_done_ctx;
};

static struct ad7779_hal_s s_hal_inst;

ad7779_hal_t *ad7779_hal_default_instance(void)
{
    return &s_hal_inst;
}

static void hal_release_resources(ad7779_hal_t *hal, bool clear)
{
    if (!hal) return;

    hal->stream_enabled = false;
    gpio_intr_disable(AD7779_PIN_DRDY);

    if (hal->stream_task) {
        vTaskDelete(hal->stream_task);
        hal->stream_task = NULL;
    }
    if (hal->drdy_sem) {
        vSemaphoreDelete(hal->drdy_sem);
        hal->drdy_sem = NULL;
    }
    if (hal->bus_mutex) {
        vSemaphoreDelete(hal->bus_mutex);
        hal->bus_mutex = NULL;
    }
    if (hal->drdy_isr_installed) {
        gpio_isr_handler_remove(AD7779_PIN_DRDY);
        hal->drdy_isr_installed = false;
    }
    if (hal->reg_dev) {
        spi_bus_remove_device(hal->reg_dev);
        hal->reg_dev = NULL;
    }
    if (hal->stream_dev) {
        spi_bus_remove_device(hal->stream_dev);
        hal->stream_dev = NULL;
    }
    if (hal->bus_owned) {
        spi_bus_free(AD7779_SPI_HOST);
        hal->bus_owned = false;
    }
    if (clear) {
        memset(hal, 0, sizeof(*hal));
    }
}

static void IRAM_ATTR drdy_isr_handler(void *arg)
{
    ad7779_hal_t *hal = (ad7779_hal_t *)arg;
    if (!hal || !hal->stream_enabled || !hal->drdy_sem) return;

    BaseType_t hpw = pdFALSE;
    /* If the semaphore is already given (previous frame not consumed),
     * xSemaphoreGiveFromISR returns pdFAIL — that's the dropped-frame case. */
    xSemaphoreGiveFromISR(hal->drdy_sem, &hpw);

    /* Forward to upper-layer ISR callback for compat. The driver core
     * uses this to set xfer_in_flight / dropped counters. */
    if (hal->drdy_cb) hal->drdy_cb(hal->drdy_ctx);

    portYIELD_FROM_ISR(hpw);
}

static void stream_task_fn(void *arg)
{
    ad7779_hal_t *hal = (ad7779_hal_t *)arg;

    while (1) {
        if (xSemaphoreTake(hal->drdy_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }
        if (!hal->stream_enabled || !hal->cur_rx_buf) continue;

        spi_transaction_t t = {
            .length    = hal->cur_len * 8,
            .tx_buffer = hal->nop_tx,
            .rx_buffer = hal->cur_rx_buf,
        };
        xSemaphoreTake(hal->bus_mutex, portMAX_DELAY);
        gpio_set_level(AD7779_PIN_CS, 0);
        esp_rom_delay_us(1);
        esp_err_t err = spi_device_polling_transmit(hal->stream_dev, &t);
        esp_rom_delay_us(1);
        gpio_set_level(AD7779_PIN_CS, 1);
        xSemaphoreGive(hal->bus_mutex);

        ad7779_hal_status_t st = (err == ESP_OK) ? AD7779_HAL_OK : AD7779_HAL_ERR_BUS;
        if (hal->cur_done_cb) {
            hal->cur_done_cb(hal->cur_done_ctx, st);
        }
    }
}

ad7779_hal_status_t ad7779_hal_init(ad7779_hal_t *hal)
{
    if (!hal) return AD7779_HAL_ERR_PARAM;
    hal_release_resources(hal, true);
    memset(hal, 0, sizeof(*hal));

    for (size_t i = 0; i < NOP_BUF_LEN; i += 2) {
        hal->nop_tx[i]     = AD7779_NOP_CMD_HI;
        hal->nop_tx[i + 1] = AD7779_NOP_CMD_LO;
    }

    {
        gpio_config_t cs_cfg = {
            .pin_bit_mask = (1ULL << AD7779_PIN_CS),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = 0,
            .pull_down_en = 0,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&cs_cfg);
        gpio_set_level(AD7779_PIN_CS, 1);
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num     = AD7779_PIN_MOSI,
        .miso_io_num     = AD7779_PIN_MISO,
        .sclk_io_num     = AD7779_PIN_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 256,
    };
    esp_err_t err = spi_bus_initialize(AD7779_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err == ESP_OK) {
        hal->bus_owned = true;
    } else if (err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %d", err);
        return AD7779_HAL_ERR_BUS;
    }

    spi_device_interface_config_t reg_devcfg = {
        .mode           = 0,
        .clock_speed_hz = AD7779_SPI_CLOCK_REG_HZ,
        .spics_io_num   = -1,
        .queue_size     = 1,
        .flags          = 0,
    };
    if (spi_bus_add_device(AD7779_SPI_HOST, &reg_devcfg, &hal->reg_dev) != ESP_OK) {
        hal_release_resources(hal, true);
        return AD7779_HAL_ERR_BUS;
    }

    spi_device_interface_config_t stream_devcfg = {
        .mode           = 0,
        .clock_speed_hz = AD7779_SPI_CLOCK_STREAM_HZ,
        .spics_io_num   = -1,
        .queue_size     = 1,
        .flags          = 0,
    };
    if (spi_bus_add_device(AD7779_SPI_HOST, &stream_devcfg,
                           &hal->stream_dev) != ESP_OK) {
        hal_release_resources(hal, true);
        return AD7779_HAL_ERR_BUS;
    }

    {
        gpio_config_t drdy_cfg = {
            .pin_bit_mask = (1ULL << AD7779_PIN_DRDY),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_NEGEDGE,
        };
        gpio_config(&drdy_cfg);
        gpio_intr_disable(AD7779_PIN_DRDY);
    }

    hal->drdy_sem = xSemaphoreCreateBinary();
    if (!hal->drdy_sem) {
        hal_release_resources(hal, true);
        return AD7779_HAL_ERR_INTERNAL;
    }

    hal->bus_mutex = xSemaphoreCreateMutex();
    if (!hal->bus_mutex) {
        hal_release_resources(hal, true);
        return AD7779_HAL_ERR_INTERNAL;
    }

    BaseType_t r = xTaskCreate(stream_task_fn, "ad7779_stream",
                               STREAM_TASK_STACK, hal,
                               STREAM_TASK_PRIO, &hal->stream_task);
    if (r != pdPASS) {
        hal_release_resources(hal, true);
        return AD7779_HAL_ERR_INTERNAL;
    }

    return AD7779_HAL_OK;
}

ad7779_hal_status_t ad7779_hal_deinit(ad7779_hal_t *hal)
{
    if (!hal) return AD7779_HAL_ERR_PARAM;

    hal_release_resources(hal, true);
    return AD7779_HAL_OK;
}

ad7779_hal_status_t ad7779_hal_spi_xfer(ad7779_hal_t *hal,
                                        const uint8_t *tx,
                                        uint8_t *rx,
                                        size_t len)
{
    if (!hal || !tx || len == 0 || !hal->reg_dev) return AD7779_HAL_ERR_PARAM;

    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    if (xSemaphoreTake(hal->bus_mutex, portMAX_DELAY) != pdTRUE) {
        return AD7779_HAL_ERR_INTERNAL;
    }
    gpio_set_level(AD7779_PIN_CS, 0);
    esp_rom_delay_us(1);
    esp_err_t err = spi_device_polling_transmit(hal->reg_dev, &t);
    esp_rom_delay_us(1);
    gpio_set_level(AD7779_PIN_CS, 1);
    xSemaphoreGive(hal->bus_mutex);

    return (err == ESP_OK) ? AD7779_HAL_OK : AD7779_HAL_ERR_BUS;
}

ad7779_hal_status_t ad7779_hal_spi_read_frame_async(ad7779_hal_t *hal,
                                                    uint8_t *rx_buf,
                                                    size_t len,
                                                    ad7779_xfer_done_cb_t done_cb,
                                                    void *done_ctx)
{
    if (!hal || !rx_buf || len == 0 || len > NOP_BUF_LEN) {
        return AD7779_HAL_ERR_PARAM;
    }
    if (!hal->stream_dev || !hal->stream_task || !hal->drdy_sem) {
        return AD7779_HAL_ERR_INTERNAL;
    }
    /* Record per-frame params; the streaming task picks them up on next DRDY. */
    hal->cur_rx_buf   = rx_buf;
    hal->cur_len      = len;
    hal->cur_done_cb  = done_cb;
    hal->cur_done_ctx = done_ctx;
    return AD7779_HAL_OK;
}

ad7779_hal_status_t ad7779_hal_attach_drdy_isr(ad7779_hal_t *hal,
                                               ad7779_drdy_isr_cb_t cb,
                                               void *user_ctx)
{
    if (!hal || !hal->drdy_sem) return AD7779_HAL_ERR_PARAM;

    hal->drdy_cb  = cb;
    hal->drdy_ctx = user_ctx;

    if (!hal->drdy_isr_installed) {
        esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            return AD7779_HAL_ERR_BUS;
        }
        if (gpio_isr_handler_add(AD7779_PIN_DRDY, drdy_isr_handler, hal) != ESP_OK) {
            return AD7779_HAL_ERR_BUS;
        }
        hal->drdy_isr_installed = true;
    }
    return AD7779_HAL_OK;
}

ad7779_hal_status_t ad7779_hal_drdy_enable(ad7779_hal_t *hal, bool enable)
{
    if (!hal) return AD7779_HAL_ERR_PARAM;
    if (enable && (!hal->drdy_sem || !hal->stream_task || !hal->stream_dev)) {
        return AD7779_HAL_ERR_INTERNAL;
    }
    hal->stream_enabled = enable;
    if (enable) gpio_intr_enable(AD7779_PIN_DRDY);
    else        gpio_intr_disable(AD7779_PIN_DRDY);
    return AD7779_HAL_OK;
}

void ad7779_hal_delay_us(ad7779_hal_t *hal, uint32_t us)
{
    (void)hal;
    if (us < 1000) {
        esp_rom_delay_us(us);
    } else {
        vTaskDelay(pdMS_TO_TICKS((us + 999) / 1000));
    }
}

void ad7779_hal_delay_ms(ad7779_hal_t *hal, uint32_t ms)
{
    (void)hal;
    if (ms == 0) return;
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t ad7779_hal_now_ms(ad7779_hal_t *hal)
{
    (void)hal;
    return (uint32_t)(esp_timer_get_time() / 1000);
}
