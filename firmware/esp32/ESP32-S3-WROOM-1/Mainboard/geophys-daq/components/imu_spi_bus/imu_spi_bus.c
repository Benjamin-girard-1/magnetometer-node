#include "imu_spi_bus.h"

#include "esp_log.h"

static const char *TAG = "imu_spi_bus";
static bool s_bus_initialized;

esp_err_t imu_spi_bus_init_once(void)
{
    if (s_bus_initialized) {
        return ESP_OK;
    }

    gpio_config_t cs_cfg = {
        .pin_bit_mask = (1ULL << LSM6DSV_PIN_CS) | (1ULL << SCL3300_PIN_CS),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = 0,
        .pull_down_en = 0,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cs_cfg);
    gpio_set_level(LSM6DSV_PIN_CS, 1);
    gpio_set_level(SCL3300_PIN_CS, 1);

    spi_bus_config_t buscfg = {
        .mosi_io_num     = IMU_PIN_MOSI,
        .miso_io_num     = IMU_PIN_MISO,
        .sclk_io_num     = IMU_PIN_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 64,
    };
    esp_err_t err = spi_bus_initialize(IMU_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        s_bus_initialized = true;
        return ESP_OK;
    }

    ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
    return err;
}
