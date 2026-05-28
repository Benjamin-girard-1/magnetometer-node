/**
 * @file scl3300_port_esp32.c
 * @brief ESP-IDF (ESP32-S3) implementation of the SCL3300 port layer.
 *
 * Wiring (from board schematic):
 *   GPIO 8  -> MOSI  (shared with LSM6DSV)
 *   GPIO 9  -> MISO  (shared with LSM6DSV)
 *   GPIO 3  -> SCLK  (shared with LSM6DSV)
 *   GPIO 46 -> CS_SCL (this device only)
 *
 * The {IMU_SPI} bus is shared with the LSM6DSV. Whichever driver runs
 * spi_bus_initialize() first wins; the other one just adds itself as a
 * device. That's what the imu_bus_init_once() one-shot guard is for.
 *
 * If both drivers are used in the same build, you should make sure both
 * port files agree on the bus host (SPI2_HOST) and the SCK/MOSI/MISO pins.
 * They do here.
 *
 * SPI specifics for SCL3300 (different from LSM6DSV!):
 *   - Mode 0 (CPOL=0, CPHA=0)
 *   - 2–4 MHz recommended for noise spec, 8 MHz absolute max
 *   - Min 10 µs CSB-high gap between transactions — enforced via cs_ena_pretrans
 *     and cs_ena_posttrans + an explicit timestamp check.
 */

#include "scl3300_port.h"

#include "imu_spi_bus.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/*  Board-specific pinout                                                    */
/* ------------------------------------------------------------------------- */
/* Datasheet §2.10.2: 2–4 MHz recommended for best noise. 4 MHz it is.       */
#define SCL3300_SPI_HZ     (4 * 1000 * 1000)

/* Datasheet §5.1.2: minimum 10 µs CSB high time between transactions.       */
#define SCL3300_TLH_US     (10)

static const char *TAG = "scl3300_port";

/* ------------------------------------------------------------------------- */
/*  Port context                                                             */
/* ------------------------------------------------------------------------- */
typedef struct {
    spi_device_handle_t dev;
    int64_t last_xfer_end_us;   /* esp_timer_get_time() at end of last xfer */
} scl3300_esp32_ctx_t;

/* ------------------------------------------------------------------------- */
/*  Port API                                                                 */
/* ------------------------------------------------------------------------- */
int scl3300_port_init(void **out_ctx)
{
    if (!out_ctx) return -1;
    if (imu_spi_bus_init_once() != ESP_OK) return -1;

    scl3300_esp32_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return -1;

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SCL3300_SPI_HZ,
        .mode           = 0,                  /* CPOL=0, CPHA=0             */
        .spics_io_num   = SCL3300_PIN_CS,
        .queue_size     = 1,
        /* The driver enforces TLH explicitly; cs_ena_posttrans gives a few  */
        /* extra SCK cycles of CS-low after the last bit, which the SCL3300  */
        /* tolerates fine.                                                   */
        .cs_ena_posttrans = 2,
    };
    esp_err_t err = spi_bus_add_device(IMU_SPI_HOST, &devcfg, &ctx->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        free(ctx);
        return -1;
    }

    ctx->last_xfer_end_us = 0;
    *out_ctx = ctx;
    return 0;
}

int scl3300_port_xfer(void *ctx_, uint32_t tx_frame, uint32_t *rx_frame)
{
    scl3300_esp32_ctx_t *ctx = (scl3300_esp32_ctx_t *)ctx_;
    if (!ctx || !rx_frame) return -1;

    /* Enforce TLH = 10 µs minimum between CSB rises.                        */
    int64_t now = esp_timer_get_time();
    int64_t since = now - ctx->last_xfer_end_us;
    if (ctx->last_xfer_end_us != 0 && since < SCL3300_TLH_US) {
        esp_rom_delay_us((uint32_t)(SCL3300_TLH_US - since));
    }

    /* SPI transmits MSB-first by default; we want the 32-bit frame to go   */
    /* out as bytes [b31..b24], [b23..b16], [b15..b8], [b7..b0]. We pack    */
    /* explicitly to be byte-order independent.                              */
    uint8_t tx[4] = {
        (uint8_t)(tx_frame >> 24),
        (uint8_t)(tx_frame >> 16),
        (uint8_t)(tx_frame >>  8),
        (uint8_t)(tx_frame      ),
    };
    uint8_t rx[4] = {0};

    spi_transaction_t t = {
        .length    = 32,
        .rxlength  = 32,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(ctx->dev, &t);
    if (err != ESP_OK) return -1;

    *rx_frame = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16)
              | ((uint32_t)rx[2] <<  8) |  (uint32_t)rx[3];

    ctx->last_xfer_end_us = esp_timer_get_time();
    return 0;
}

void scl3300_port_delay_ms(uint32_t ms)
{
    vTaskDelay((ms / portTICK_PERIOD_MS) + 1);
}
