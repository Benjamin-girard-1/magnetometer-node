#include "adc_stream.h"

#include "ad7779.h"
#include "ad7779_crc.h"
#include "ad7779_hal.h"
#include "app_config.h"
#include "serial_control.h"
#include "shift_register.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "adc_stream";

static ad7779_t s_adc;
static bool s_adc_ready;
static bool s_adc_streaming;
static uint32_t s_last_dropped;

static void adc_forget_driver_state(void)
{
    s_adc_ready = false;
    s_adc_streaming = false;
    s_last_dropped = 0;
    memset(&s_adc, 0, sizeof(s_adc));
}

static void adc_reset_pin_pulse(void)
{
    sr_set_pin(SR_ADC_START, false);
    sr_set_pin(SR_ADC_CONVST_SAR, true);
    sr_set_pin(SR_ADC_MCLK_EN, true);
    vTaskDelay(pdMS_TO_TICKS(5));

    sr_set_pin(SR_ADC_RESET, false);
    vTaskDelay(pdMS_TO_TICKS(2));
    sr_set_pin(SR_ADC_RESET, true);
    vTaskDelay(pdMS_TO_TICKS(10));

    sr_set_pin(SR_ADC_START, true);
    vTaskDelay(pdMS_TO_TICKS(5));
}

static ad7779_gain_t gain_from_x(uint8_t gain_x, bool *ok)
{
    *ok = true;
    switch (gain_x) {
    case 1: return AD7779_GAIN_1;
    case 2: return AD7779_GAIN_2;
    case 4: return AD7779_GAIN_4;
    case 8: return AD7779_GAIN_8;
    default:
        *ok = false;
        return AD7779_GAIN_1;
    }
}

static ad7779_hal_status_t adc_diag_read_reg(ad7779_hal_t *hal, uint8_t addr, uint8_t *val)
{
    uint8_t tx[3];
    uint8_t rx[3] = { 0 };

    tx[0] = (uint8_t)(AD7779_SPI_READ | (addr & 0x7FU));
    tx[1] = 0x00U;
    tx[2] = ad7779_crc8(tx, 2);

    ad7779_hal_status_t st = ad7779_hal_spi_xfer(hal, tx, rx, sizeof(tx));
    ESP_LOGI(TAG,
             "ADC DIAG read reg 0x%02x: st=%d tx=[%02x %02x %02x] rx=[%02x %02x %02x]",
             addr, st, tx[0], tx[1], tx[2], rx[0], rx[1], rx[2]);
    if (val) {
        *val = rx[1];
    }
    if (rx[0] == 0xFFU && rx[1] == 0xFFU && rx[2] == 0xFFU) {
        ESP_LOGW(TAG, "ADC DIAG read reg 0x%02x: all-ones response means SDO/MISO is high or floating", addr);
    }
    return st;
}

static ad7779_hal_status_t adc_diag_write_reg(ad7779_hal_t *hal, uint8_t addr, uint8_t val)
{
    uint8_t tx[3];
    uint8_t rx[3] = { 0 };

    tx[0] = (uint8_t)(AD7779_SPI_WRITE | (addr & 0x7FU));
    tx[1] = val;
    tx[2] = ad7779_crc8(tx, 2);

    ad7779_hal_status_t st = ad7779_hal_spi_xfer(hal, tx, rx, sizeof(tx));
    ESP_LOGI(TAG,
             "ADC DIAG write reg 0x%02x=0x%02x: st=%d tx=[%02x %02x %02x] rx=[%02x %02x %02x]",
             addr, val, st, tx[0], tx[1], tx[2], rx[0], rx[1], rx[2]);
    return st;
}

static void adc_sample_cb(void *ctx, const int32_t *samples,
                          uint8_t status, uint32_t frame_idx)
{
    (void)ctx;

    if ((frame_idx % SERIAL_STUDIO_DECIMATION) != 0U) {
        return;
    }

    serial_write_adc_packet(samples, status, frame_idx);
}

esp_err_t adc_power_on(void)
{
    sr_set_pin(SR_ADC_MCLK_EN, true);
    sr_set_pin(SR_ADC_RESET, true);
    sr_set_pin(SR_ADC_START, true);
    sr_set_pin(SR_ADC_CONVST_SAR, true);
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

void adc_power_off(void)
{
    sr_set_pin(SR_ADC_START, false);
    sr_set_pin(SR_ADC_CONVST_SAR, false);
    sr_set_pin(SR_ADC_RESET, false);
    sr_set_pin(SR_ADC_MCLK_EN, false);
}

esp_err_t adc_recover(void)
{
    ESP_LOGW(TAG, "forcing AD7779 reset/recovery sequence");

    if (s_adc.hal != NULL) {
        (void)ad7779_deinit(&s_adc);
    } else {
        (void)ad7779_hal_deinit(ad7779_hal_default_instance());
    }
    adc_forget_driver_state();

    adc_power_off();
    vTaskDelay(pdMS_TO_TICKS(50));
    adc_reset_pin_pulse();

    ad7779_hal_t *hal = ad7779_hal_default_instance();
    if (ad7779_hal_init(hal) != AD7779_HAL_OK) {
        ESP_LOGW(TAG, "AD7779 HAL init failed during recovery");
        (void)ad7779_hal_deinit(hal);
        adc_forget_driver_state();
        return ESP_FAIL;
    }

    ad7779_t tmp = {
        .hal = hal,
        .cfg = AD7779_DEFAULT_CONFIG,
        .crc_enabled = false,
    };
    ad7779_status_t st = ad7779_soft_reset(&tmp);
    (void)ad7779_hal_deinit(hal);
    adc_forget_driver_state();

    if (st != AD7779_OK) {
        ESP_LOGW(TAG, "AD7779 SPI software reset failed during recovery: %d", st);
        return ESP_FAIL;
    }

    adc_reset_pin_pulse();
    ESP_LOGI(TAG, "AD7779 recovery complete; send MODE ADC to reinitialize");
    return ESP_OK;
}

esp_err_t adc_diag(void)
{
    ESP_LOGI(TAG, "starting AD7779 register/SPI diagnostic");

    if (s_adc.hal != NULL) {
        (void)ad7779_deinit(&s_adc);
    } else {
        (void)ad7779_hal_deinit(ad7779_hal_default_instance());
    }
    adc_forget_driver_state();

    adc_power_off();
    vTaskDelay(pdMS_TO_TICKS(50));
    adc_reset_pin_pulse();

    ad7779_hal_t *hal = ad7779_hal_default_instance();
    if (ad7779_hal_init(hal) != AD7779_HAL_OK) {
        ESP_LOGW(TAG, "ADC DIAG: HAL init failed");
        (void)ad7779_hal_deinit(hal);
        adc_forget_driver_state();
        return ESP_FAIL;
    }

    ad7779_t tmp = {
        .hal = hal,
        .cfg = AD7779_DEFAULT_CONFIG,
        .crc_enabled = false,
    };
    ESP_LOGI(TAG, "ADC DIAG: sending 64-SCLK SDI-high software reset");
    (void)ad7779_soft_reset(&tmp);

    for (int i = 0; i < 20; ++i) {
        uint8_t status3 = 0;
        ad7779_hal_status_t st = adc_diag_read_reg(hal, AD7779_REG_STATUS_REG_3, &status3);
        if (st == AD7779_HAL_OK && status3 != 0xFFU &&
            (status3 & AD7779_STAT3_INIT_COMPLETE)) {
            ESP_LOGI(TAG, "ADC DIAG: INIT_COMPLETE observed after %d ms", i);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint8_t unused = 0;
    (void)adc_diag_read_reg(hal, AD7779_REG_STATUS_REG_1, &unused);
    (void)adc_diag_read_reg(hal, AD7779_REG_STATUS_REG_2, &unused);
    (void)adc_diag_read_reg(hal, AD7779_REG_STATUS_REG_3, &unused);
    (void)adc_diag_read_reg(hal, AD7779_REG_GEN_ERR_REG_1, &unused);
    (void)adc_diag_read_reg(hal, AD7779_REG_GEN_ERR_REG_2, &unused);
    (void)adc_diag_read_reg(hal, AD7779_REG_GEN_ERR_REG_1_EN, &unused);

    (void)adc_diag_write_reg(hal, AD7779_REG_GEN_ERR_REG_1_EN,
                             AD7779_ERR1_EN_SPI_CRC_TEST);
    (void)adc_diag_read_reg(hal, AD7779_REG_GEN_ERR_REG_1_EN, &unused);
    (void)adc_diag_read_reg(hal, AD7779_REG_STATUS_REG_3, &unused);

    (void)ad7779_hal_deinit(hal);
    adc_forget_driver_state();
    ESP_LOGI(TAG, "ADC DIAG complete; send ADC RESET or MODE ADC next");
    return ESP_OK;
}

esp_err_t adc_stream_start(void)
{
    if (s_adc_streaming) {
        ESP_RETURN_ON_ERROR(adc_power_on(), TAG, "ADC power re-assert failed");
        serial_select_output(UART_OUTPUT_ADC_BINARY);
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(adc_power_on(), TAG, "ADC power-on failed");

    if (!s_adc_ready) {
        ad7779_config_t cfg = AD7779_DEFAULT_CONFIG;
        cfg.odr_hz = MAG_ODR_HZ;
        cfg.reference = AD7779_REF_INTERNAL;
        cfg.channels_enabled = 0xFFU;
        cfg.verify_writes = true;
        cfg.use_crc = true;

        ad7779_status_t st = ad7779_init(&s_adc, ad7779_hal_default_instance(), &cfg);
        if (st != AD7779_OK) {
            ESP_LOGE(TAG, "ad7779_init failed: %d", st);
            adc_power_off();
            adc_forget_driver_state();
            return ESP_FAIL;
        }

        ad7779_set_sample_callback(&s_adc, adc_sample_cb, NULL);
        s_adc_ready = true;
    }

    ad7779_status_t st = ad7779_start_streaming(&s_adc);
    if (st != AD7779_OK) {
        ESP_LOGE(TAG, "ad7779_start_streaming failed: %d", st);
        return ESP_FAIL;
    }

    s_adc_streaming = true;
    serial_select_output(UART_OUTPUT_ADC_BINARY);

    ESP_LOGI(TAG,
             "AD7779 OK. Magnetic expansion bring-up: +9V=%s, -5V=%s, "
             "ref = internal %.1f V, ODR = %lu Hz, gain = x%.0f. "
             "Binary serial output = %lu Hz at %lu baud.",
             (MAG_ENABLE_BRIDGE_9V != 0) ? "on" : "off",
             (MAG_ENABLE_NEG5V != 0) ? "on" : "off",
             ADC_REF_V, (unsigned long)MAG_ODR_HZ, MAG_ADC_GAIN,
             (unsigned long)SERIAL_STUDIO_RATE_HZ, (unsigned long)MAG_UART_BAUD);
    return ESP_OK;
}

bool adc_streaming(void)
{
    return s_adc_streaming;
}

esp_err_t adc_set_channel_gain(uint8_t ch, uint8_t gain_x)
{
    bool ok = false;
    ad7779_gain_t gain = gain_from_x(gain_x, &ok);
    if (!ok || ch >= AD7779_NUM_CHANNELS) {
        ESP_LOGW(TAG, "bad ADC gain request: ch=%u gain=x%u", ch, gain_x);
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_adc_ready) {
        ESP_LOGW(TAG, "ADC gain request ignored: ADC is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ad7779_status_t st = ad7779_set_channel_gain_writeonly(&s_adc, ch, gain);
    if (st != AD7779_OK) {
        ESP_LOGW(TAG, "ADC gain set failed: ch=%u gain=x%u st=%d", ch, gain_x, st);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ADC CH%u gain set to x%u", ch, gain_x);
    return ESP_OK;
}

void adc_log_drops_if_changed(void)
{
    if (!s_adc_streaming) {
        return;
    }

    uint32_t dropped = ad7779_frames_dropped(&s_adc);
    if (dropped != s_last_dropped) {
        ESP_LOGW(TAG, "frames=%lu dropped=%lu (+%lu)",
                 (unsigned long)ad7779_frame_count(&s_adc),
                 (unsigned long)dropped,
                 (unsigned long)(dropped - s_last_dropped));
        s_last_dropped = dropped;
    }
}
