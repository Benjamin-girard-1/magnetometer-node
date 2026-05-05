/**
 * @file main.c
 * @brief AD7779 AIN3 thermistor bring-up.
 */

#include "ad7779.h"
#include "ad7779_hal.h"
#include "lsm6dsv.h"
#include "lsm6dsv_regs.h"
#include "scl3300.h"
#include "shift_register.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <math.h>

static const char *TAG = "therm";

#define THERM_CH                 3
#define THERM_R_TOP_OHM          2400.0f
#define THERM_R_NOMINAL_OHM      10000.0f
#define THERM_T_NOMINAL_K        298.15f
#define THERM_BETA_K             4250.0f
#define ADC_AVDD_V               3.2951f
#define ADC_FULL_SCALE_CODE       8388608.0f
#define THERM_ADC_GAIN           2.0f
#define THERM_ODR_HZ             512U
#define THERM_LOG_EVERY_FRAMES    1024U

typedef struct {
    lsm6dsv_t *lsm6dsv;
    scl3300_t *scl3300;
    bool lsm6dsv_ok;
    bool lsm6dsv_fallback;
    bool scl3300_ok;
} app_ctx_t;

static float thermistor_temp_c_from_ohm(float r_ohm)
{
    float inv_t = (1.0f / THERM_T_NOMINAL_K) +
                  (logf(r_ohm / THERM_R_NOMINAL_OHM) / THERM_BETA_K);
    return (1.0f / inv_t) - 273.15f;
}

static void adc_sample_cb(void *ctx, const int32_t *samples,
                          uint8_t status, uint32_t frame_idx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;

    if ((frame_idx % THERM_LOG_EVERY_FRAMES) != 0U) {
        return;
    }

    int32_t code = samples[THERM_CH];
    float ratio = (float)code / (ADC_FULL_SCALE_CODE * THERM_ADC_GAIN);
    float v_r64 = ratio * ADC_AVDD_V;
    float v_node = ADC_AVDD_V - v_r64;
    float r_ntc = NAN;
    float temp_c = NAN;
    float lsm_temp_c = NAN;
    float scl_temp_c = NAN;
    int16_t lsm_temp_raw = 0;
    uint8_t lsm_status = 0;
    const char *lsm_label = "LSM";
    uint8_t status_flags = (uint8_t)(status & 0x0FU);

    if (ratio > 0.0001f && ratio < 0.9999f) {
        r_ntc = THERM_R_TOP_OHM * (1.0f - ratio) / ratio;
        temp_c = thermistor_temp_c_from_ohm(r_ntc);
    }

    if (app && app->lsm6dsv_ok) {
        lsm_label = app->lsm6dsv_fallback ? "LSM?" : "LSM";
        (void)lsm6dsv_read_reg(app->lsm6dsv, LSM6DSV_REG_STATUS_REG,
                               &lsm_status, 1);
        if (lsm6dsv_read_temp_raw(app->lsm6dsv, &lsm_temp_raw) == LSM6DSV_OK) {
            lsm_temp_c = ((float)lsm_temp_raw / 256.0f) + 25.0f;
        }
    }

    if (app && app->scl3300_ok) {
        scl3300_sample_t s = {0};
        if (scl3300_read_sample(app->scl3300, &s) == SCL3300_OK) {
            scl_temp_c = s.temp_c;
        }
    }

    ESP_LOGI(TAG,
             "Rntc=%8.1f ohm  T=%7.2f C  %s=%7.2f C raw=%6d st=0x%02X  SCL=%7.2f C  "
             "node=%0.4f V  flags=0x%02X",
             r_ntc, temp_c, lsm_label, lsm_temp_c, (int)lsm_temp_raw,
             lsm_status, scl_temp_c, v_node, status_flags);
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

    lsm6dsv_t lsm = {0};
    lsm6dsv_config_t lsm_cfg = LSM6DSV_CONFIG_DEFAULT();
    lsm6dsv_status_t lsm_st = lsm6dsv_init(&lsm, &lsm_cfg);
    bool lsm_ok = (lsm_st == LSM6DSV_OK);
    bool lsm_fallback = false;
    if (lsm_ok) {
        ESP_LOGI(TAG, "LSM6DSV OK. WHO_AM_I=0x%02X. Using internal temperature as reference.",
                 lsm.last_who_am_i);
    } else if (lsm_st == LSM6DSV_ERR_WHO_AM_I &&
               lsm.last_who_am_i != 0x00U &&
               lsm.last_who_am_i != 0xFFU) {
        lsm_ok = true;
        lsm_fallback = true;
        ESP_LOGW(TAG,
                 "LSM6DSV WHO_AM_I mismatch: got 0x%02X. "
                 "Trying temperature-only fallback.",
                 lsm.last_who_am_i);
    } else {
        ESP_LOGW(TAG, "LSM6DSV init failed: %d, WHO_AM_I=0x%02X",
                 lsm_st, lsm.last_who_am_i);
    }

    scl3300_t scl = {0};
    scl3300_config_t scl_cfg = SCL3300_CONFIG_DEFAULT();
    scl3300_status_t scl_st = scl3300_init(&scl, &scl_cfg);
    bool scl_ok = (scl_st == SCL3300_OK);
    if (scl_ok) {
        ESP_LOGI(TAG, "SCL3300 OK. Using internal temperature as reference.");
    } else {
        ESP_LOGW(TAG, "SCL3300 init failed: %d", scl_st);
    }

    app_ctx_t app = {
        .lsm6dsv = &lsm,
        .scl3300 = &scl,
        .lsm6dsv_ok = lsm_ok,
        .lsm6dsv_fallback = lsm_fallback,
        .scl3300_ok = scl_ok,
    };

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

    ad7779_set_sample_callback(&adc, adc_sample_cb, &app);
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
