/**
 * @file lsm6dsv_port_esp32.c
 * @brief ESP-IDF (ESP32-S3) implementation of the LSM6DSV port layer.
 *
 * Wiring (from board schematic):
 *   GPIO 8  -> MOSI (LSM6DSV pin 14, SDA)
 *   GPIO 9  -> MISO (LSM6DSV pin 1,  SDO)
 *   GPIO 3  -> SCLK (LSM6DSV pin 13, SCL)
 *   GPIO 17 -> CS_LSM (LSM6DSV pin 12)
 *
 * The {IMU_SPI} bus is also wired to the SCL3300. This file initializes
 * SPI2_HOST as a *shared* bus, so a future SCL3300 driver only needs to
 * call spi_bus_add_device() with its own CS pin — it does NOT call
 * spi_bus_initialize() again.
 *
 * To prevent double-init, the SPI2_HOST setup is wrapped in a one-shot
 * guard so whichever driver runs first wins. If you'd rather centralize
 * this (recommended for a real app), pull the bus-init out into a small
 * "imu_bus.c" component and have both drivers just add their device handle.
 */

#include "lsm6dsv_port.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/*  Board-specific pinout (edit if you re-route)                             */
/* ------------------------------------------------------------------------- */
#define IMU_SPI_HOST       SPI3_HOST
#define IMU_PIN_MOSI       8
#define IMU_PIN_MISO       9
#define IMU_PIN_SCLK       3
#define LSM6DSV_PIN_CS     17

/* Datasheet §2 Table 5: SPI clock max = 10 MHz.                             */
#define LSM6DSV_SPI_HZ     (8 * 1000 * 1000)

static const char *TAG = "lsm6dsv_port";

/* ------------------------------------------------------------------------- */
/*  Shared-bus init guard                                                    */
/* ------------------------------------------------------------------------- */
static bool s_bus_initialized = false;

static esp_err_t imu_bus_init_once(void)
{
    if (s_bus_initialized) {
        return ESP_OK;
    }
    spi_bus_config_t buscfg = {
        .mosi_io_num     = IMU_PIN_MOSI,
        .miso_io_num     = IMU_PIN_MISO,
        .sclk_io_num     = IMU_PIN_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 64,
    };
    esp_err_t err = spi_bus_initialize(IMU_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err == ESP_ERR_INVALID_STATE) {
        /* Already initialized by another driver on this shared bus.        */
        s_bus_initialized = true;
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }
    s_bus_initialized = true;
    return ESP_OK;
}

/* ------------------------------------------------------------------------- */
/*  Port context                                                             */
/* ------------------------------------------------------------------------- */
typedef struct {
    spi_device_handle_t dev;
} lsm6dsv_esp32_ctx_t;

/* ------------------------------------------------------------------------- */
/*  Port API                                                                 */
/* ------------------------------------------------------------------------- */
int lsm6dsv_port_init(void **out_ctx)
{
    if (!out_ctx) return -1;

    if (imu_bus_init_once() != ESP_OK) return -1;

    lsm6dsv_esp32_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return -1;

    /* SPI mode 3: CPOL=1, CPHA=1 (datasheet §5.1.2 mode 3 — see Figure 7).  */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = LSM6DSV_SPI_HZ,
        .mode           = 3,
        .spics_io_num   = LSM6DSV_PIN_CS,
        .queue_size     = 1,
        /* No flags: full-duplex, MSb-first — matches LSM6DSV protocol.     */
    };
    esp_err_t err = spi_bus_add_device(IMU_SPI_HOST, &devcfg, &ctx->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        free(ctx);
        return -1;
    }

    *out_ctx = ctx;
    return 0;
}

int lsm6dsv_port_xfer(void *ctx_, const uint8_t *tx, uint8_t *rx, size_t len)
{
    lsm6dsv_esp32_ctx_t *ctx = (lsm6dsv_esp32_ctx_t *)ctx_;
    if (!ctx || len == 0) return -1;

    spi_transaction_t t = {
        .length    = len * 8,            /* in bits                          */
        .rxlength  = (rx ? len * 8 : 0), /* 0 means "same as length"         */
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(ctx->dev, &t);
    return (err == ESP_OK) ? 0 : -1;
}

void lsm6dsv_port_delay_ms(uint32_t ms)
{
    /* vTaskDelay rounds down to ticks; +1 guarantees at least @p ms.        */
    vTaskDelay((ms / portTICK_PERIOD_MS) + 1);
}
