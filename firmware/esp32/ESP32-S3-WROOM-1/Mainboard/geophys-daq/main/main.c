/**
 * @file main.c
 * @brief AD7779 AIN3 thermistor bring-up.
 */

#include "ad7779.h"
#include "ad7779_hal.h"
#include "shift_register.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <math.h>

static const char *TAG = "therm";

#define THERM_CH                 3
#define THERM_R_TOP_OHM          27000.0f
#define THERM_R_NOMINAL_OHM      100000.0f
#define THERM_T_NOMINAL_K        298.15f
#define THERM_BETA_K             4250.0f
#define ADC_AVDD_V               3.3f
#define ADC_FULL_SCALE_CODE       8388608.0f
#define THERM_ADC_GAIN           2.0f
#define THERM_ODR_HZ             512U
#define THERM_LOG_EVERY_FRAMES    1024U

static float thermistor_temp_c_from_ohm(float r_ohm)
{
    float inv_t = (1.0f / THERM_T_NOMINAL_K) +
                  (logf(r_ohm / THERM_R_NOMINAL_OHM) / THERM_BETA_K);
    return (1.0f / inv_t) - 273.15f;
}

static void adc_sample_cb(void *ctx, const int32_t *samples,
                          uint8_t status, uint32_t frame_idx)
{
    (void)ctx;

    if ((frame_idx % THERM_LOG_EVERY_FRAMES) != 0U) {
        return;
    }

    int32_t code = samples[THERM_CH];
    float ratio = (float)code / (ADC_FULL_SCALE_CODE * THERM_ADC_GAIN);
    float v_27k = ratio * ADC_AVDD_V;
    float v_node = ADC_AVDD_V - v_27k;
    float r_ntc = NAN;
    float temp_c = NAN;
    uint8_t status_flags = (uint8_t)(status & 0x0FU);

    if (ratio > 0.0001f && ratio < 0.9999f) {
        r_ntc = THERM_R_TOP_OHM * (1.0f - ratio) / ratio;
        temp_c = thermistor_temp_c_from_ohm(r_ntc);
    }

    ESP_LOGI(TAG, "Rntc=%8.1f ohm  T=%7.2f C  node=%0.4f V  flags=0x%02X",
             r_ntc, temp_c, v_node, status_flags);
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(150));

    if (sr_init() != 0) {
        ESP_LOGE(TAG, "shift register init failed");
        return;
    }

    sr_set_pin(SR_EN_LDO_3V3, true);
    vTaskDelay(pdMS_TO_TICKS(20));
    sr_set_pin(SR_ADC_MCLK_EN, true);
    sr_set_pin(SR_ADC_RESET, true);
    sr_set_pin(SR_ADC_START, true);
    sr_set_pin(SR_ADC_CONVST_SAR, true);
    vTaskDelay(pdMS_TO_TICKS(100));

    ad7779_t adc = {0};
    ad7779_config_t cfg = AD7779_DEFAULT_CONFIG;
    cfg.odr_hz = THERM_ODR_HZ;
    cfg.reference = AD7779_REF_AVDD;
    cfg.gain[THERM_CH] = AD7779_GAIN_2;
    cfg.channels_enabled = 0xFFU;
    cfg.verify_writes = true;
    cfg.use_crc = true;

    ad7779_status_t st = ad7779_init(&adc, ad7779_hal_default_instance(), &cfg);
    if (st != AD7779_OK) {
        ESP_LOGE(TAG, "ad7779_init failed: %d", st);
        return;
    }

    ESP_LOGI(TAG,
             "AD7779 OK. Reading thermistor on AIN3: AIN3+ = 3.3VA, "
             "AIN3- = divider node, ref = AVDD, ODR = %lu Hz, gain = x%.0f, "
             "R25 = %.0f ohm, beta = %.0f K.",
             (unsigned long)THERM_ODR_HZ, THERM_ADC_GAIN,
             THERM_R_NOMINAL_OHM, THERM_BETA_K);

    ad7779_set_sample_callback(&adc, adc_sample_cb, NULL);
    st = ad7779_start_streaming(&adc);
    if (st != AD7779_OK) {
        ESP_LOGE(TAG, "ad7779_start_streaming failed: %d", st);
        return;
    }

    uint32_t last_dropped = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        uint32_t dropped = ad7779_frames_dropped(&adc);
        if (dropped != last_dropped) {
            ESP_LOGW(TAG, "frames=%lu dropped=%lu (+%lu)",
                     (unsigned long)ad7779_frame_count(&adc),
                     (unsigned long)dropped,
                     (unsigned long)(dropped - last_dropped));
            last_dropped = dropped;
        }
    }
}
