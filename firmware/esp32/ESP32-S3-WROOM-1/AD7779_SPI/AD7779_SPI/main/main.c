#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "ad7779.h"

// =========================
// Pin configuration – ADAPT
// =========================
#define PIN_NUM_MOSI   48   // ESP32-S3 GPIO connected to AD7779 SDI (pin 20)
#define PIN_NUM_MISO   47   // ESP32-S3 GPIO connected to AD7779 SDO (pin 21)
#define PIN_NUM_SCLK   45   // ESP32-S3 GPIO connected to AD7779 SCLK (pin 19)
#define PIN_NUM_CS     0    // ESP32-S3 GPIO connected to AD7779 CS  (pin 18)



// SPI host
#define AD7779_SPI_HOST   SPI2_HOST

static const char *TAG = "AD7779_SPI";

static spi_device_handle_t ad7779_handle;


static esp_err_t ad7779_spi_init(void)
{
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4,  // we only move 2 bytes at a time
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(AD7779_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Device configuration for AD7779
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,  // 1 MHz to start (well inside AD7779 limits)
        .mode = 0,                          // SPI mode 0
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
        .flags = 0,
    };

    ret = spi_bus_add_device(AD7779_SPI_HOST, &devcfg, &ad7779_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPI bus and AD7779 device initialized");
    return ESP_OK;
}


static esp_err_t ad7779_reg_read(uint8_t reg_addr, uint8_t *out)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2] = {0};

    // R/W = 1 (read), then 7-bit address
    tx_buf[0] = (1u << 7) | (reg_addr & 0x7F);
    tx_buf[1] = 0x00;   // don't care

    spi_transaction_t t = {
        .length   = 16,      // bits
        .rxlength = 16,      // bits
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    esp_err_t ret = spi_device_transmit(ad7779_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "reg_read 0x%02X failed: %s",
                 reg_addr, esp_err_to_name(ret));
        return ret;
    }

    // For debugging on the scope / logs
    ESP_LOGI(TAG, "Reg 0x%02X RX raw: 0x%02X 0x%02X",
             reg_addr, rx_buf[0], rx_buf[1]);

    // rx_buf[0] should be 0x20 in normal SPI read mode
    if (rx_buf[0] != 0x20) {
        ESP_LOGW(TAG, "Unexpected header 0x%02X when reading reg 0x%02X",
                 rx_buf[0], reg_addr);
    }

    *out = rx_buf[1];
    return ESP_OK;
}

// Range of registers to scan (adjust as you like)
#define AD7779_REG_FIRST   0x00
#define AD7779_REG_LAST    0x64   // last register you care about
void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Starting AD7779 SPI full register scan...");

    // Initialize SPI + device
    ret = ad7779_spi_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI init failed, aborting");
        return;
    }

    // Give the ADC some time after power-up
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t current_reg = AD7779_REG_FIRST;

    while (1) {
        uint8_t value = 0;

        // Read the current register
        ret = ad7779_reg_read(current_reg, &value);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Reg 0x%02X = 0x%02X", current_reg, value);
        } else {
            ESP_LOGE(TAG, "Failed to read reg 0x%02X: %s",
                     current_reg, esp_err_to_name(ret));
        }

        // Advance to next register, wrap at AD7779_REG_LAST
        if (current_reg >= AD7779_REG_LAST) {
            current_reg = AD7779_REG_FIRST;
            ESP_LOGI(TAG, "---- Wrapped back to 0x%02X ----", current_reg);
        } else {
            current_reg++;
        }

        // Wait 100 ms before reading the next register
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}