#include "adc_stream.h"

#include "ad7779.h"
#include "ad7779_hal.h"
#include "app_config.h"
#include "serial_control.h"
#include "shift_register.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "adc_stream";

static ad7779_t s_adc;
static bool s_adc_ready;
static bool s_adc_streaming;
static uint32_t s_last_dropped;

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
