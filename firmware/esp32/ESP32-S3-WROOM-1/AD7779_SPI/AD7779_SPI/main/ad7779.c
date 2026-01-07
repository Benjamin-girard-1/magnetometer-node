#include "ad7779.h"
#include "esp_log.h"

static const char *TAG = "AD7779";

/* 
 * Internal helper:
 * AD7779 uses a 16-bit command word for register access.
 * We already validated in your tests that sending a 16-bit command
 * and then reading back 2 bytes returns the register value in the
 * second byte (rx[1]), with a header/status in rx[0] (0x20).
 *
 * To avoid breaking working behavior, we:
 *  - keep addresses as 8 bits
 *  - pack them into the command word as:
 *      [15]    = 1 for read, 0 for write
 *      [14:8]  = register address (7 bits, we use reg & 0x7F)
 *      [7:0]   = don't care (0)
 *
 * If you ever want to tweak this, check the SPI timing diagrams in the DS,
 * but this layout is consistent with what you already observed in practice.
 */

static esp_err_t ad7779_spi_transfer_16(ad7779_handle_t dev,
                                        uint16_t cmd,
                                        uint8_t *rx_hi,
                                        uint8_t *rx_lo)
{
    uint8_t tx[2] = {
        (uint8_t)(cmd >> 8),
        (uint8_t)(cmd & 0xFF)
    };
    uint8_t rx[2] = {0};

    spi_transaction_t t = {
        .flags     = 0,
        .length    = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t ret = spi_device_transmit(dev, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_transmit failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (rx_hi) *rx_hi = rx[0];
    if (rx_lo) *rx_lo = rx[1];

    return ESP_OK;
}

esp_err_t ad7779_reg_read8(ad7779_handle_t dev, uint16_t reg, uint8_t *value)
{
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Build a read command: bit 15 = 1, bits[14:8] = reg address */
    uint16_t cmd = (uint16_t)(0x8000u | ((reg & 0x7Fu) << 8));

    uint8_t header = 0, data = 0;
    esp_err_t ret = ad7779_spi_transfer_16(dev, cmd, &header, &data);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Header often reads as 0x20 when in Σ-Δ data mode; we can warn if it's weird. */
    if ((header & 0xE0) != 0x20) {
        ESP_LOGW(TAG, "Unexpected header 0x%02X when reading reg 0x%02X", header, (unsigned)reg);
    }

    *value = data;
    return ESP_OK;
}

esp_err_t ad7779_reg_write8(ad7779_handle_t dev, uint16_t reg, uint8_t value)
{
    /* Write command: bit 15 = 0, bits[14:8] = reg address. */
    uint16_t cmd = (uint16_t)(((reg & 0x7Fu) << 8) | value);

    /* For writes we don’t care about returned bytes, just send the frame. */
    return ad7779_spi_transfer_16(dev, cmd, NULL, NULL);
}

esp_err_t ad7779_reg_update_bits(ad7779_handle_t dev,
                                 uint16_t reg,
                                 uint8_t mask,
                                 uint8_t value)
{
    uint8_t old;
    esp_err_t ret = ad7779_reg_read8(dev, reg, &old);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t new_val = (old & ~mask) | (value & mask);

    if (new_val == old) {
        return ESP_OK;  /* no change */
    }

    return ad7779_reg_write8(dev, reg, new_val);
}

/* ============================================================
 * Convenience helpers
 * ============================================================
 */

esp_err_t ad7779_soft_reset(ad7779_handle_t dev)
{
    /* According to the DS, GENERAL_USER_CONFIG_1[1:0] are reset bits.  [oai_citation:16‡ad7779.pdf](sediment://file_000000000ca871f7b3cdaef4e420ceea) */
    /* Typical pattern: write 0b11, then return to 0b00. */
    esp_err_t ret = ad7779_reg_update_bits(dev,
                                           AD7779_REG_GENERAL_USER_CONFIG_1,
                                           AD7779_GUC1_SOFT_RESET_MASK,
                                           0x03);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Small delay could be appropriate here; you can also just rely on the analog init time. */
    vTaskDelay(pdMS_TO_TICKS(2));

    /* Clear soft reset bits back to 0 */
    ret = ad7779_reg_update_bits(dev,
                                 AD7779_REG_GENERAL_USER_CONFIG_1,
                                 AD7779_GUC1_SOFT_RESET_MASK,
                                 0x00);

    return ret;
}

esp_err_t ad7779_set_powermode(ad7779_handle_t dev, bool high_resolution)
{
    uint8_t val = high_resolution ? AD7779_GUC1_POWERMODE : 0;
    return ad7779_reg_update_bits(dev,
                                  AD7779_REG_GENERAL_USER_CONFIG_1,
                                  AD7779_GUC1_POWERMODE,
                                  val);
}

esp_err_t ad7779_set_dout_format(ad7779_handle_t dev,
                                 uint8_t dout_format,
                                 uint8_t dclk_div)
{
    if (dout_format > 3 || dclk_div > 7) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t fmt_bits = (uint8_t)(dout_format << AD7779_DOUTFMT_DOUT_FORMAT_SHIFT);
    uint8_t div_bits = (uint8_t)(dclk_div   << AD7779_DOUTFMT_DCLK_CLK_DIV_SHIFT);

    uint8_t mask = (uint8_t)(AD7779_DOUTFMT_DOUT_FORMAT_MASK |
                             AD7779_DOUTFMT_DCLK_CLK_DIV_MASK);

    return ad7779_reg_update_bits(dev,
                                  AD7779_REG_DOUT_FORMAT,
                                  mask,
                                  (fmt_bits | div_bits));
}