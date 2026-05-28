#include "imu_debug.h"

#include "lsm6dsv.h"
#include "scl3300.h"
#include "serial_control.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "imu_debug";

static lsm6dsv_t s_lsm6dsv;
static bool s_lsm6dsv_ready;
static scl3300_t s_scl3300;
static bool s_scl3300_ready;

static bool lsm6dsv_sample_is_zero_frame(const lsm6dsv_sample_t *s)
{
    return s->xl_x_g == 0.0f &&
           s->xl_y_g == 0.0f &&
           s->xl_z_g == 0.0f &&
           s->g_x_dps == 0.0f &&
           s->g_y_dps == 0.0f &&
           s->g_z_dps == 0.0f &&
           s->temp_c == 25.0f;
}

static lsm6dsv_status_t lsm6dsv_read_sample_resynced(lsm6dsv_sample_t *s)
{
    lsm6dsv_status_t st = LSM6DSV_ERR_PARAM;

    for (int attempt = 0; attempt < 3; ++attempt) {
        st = lsm6dsv_read_sample(&s_lsm6dsv, s);
        if (st != LSM6DSV_OK) {
            return st;
        }
        if (!lsm6dsv_sample_is_zero_frame(s)) {
            return LSM6DSV_OK;
        }

        uint8_t who = 0;
        (void)lsm6dsv_who_am_i(&s_lsm6dsv, &who);
        ESP_LOGW(TAG, "LSM6DSV all-zero sample; retrying after SPI resync (attempt %d, who=0x%02x)",
                 attempt + 1,
                 who);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    return st;
}

static esp_err_t lsm6dsv_ensure_ready(void)
{
    if (!s_lsm6dsv_ready) {
        lsm6dsv_config_t cfg = LSM6DSV_CONFIG_DEFAULT();
        lsm6dsv_status_t st = lsm6dsv_init(&s_lsm6dsv, &cfg);
        if (st != LSM6DSV_OK) {
            ESP_LOGE(TAG, "LSM6DSV init failed: st=%d who=0x%02x",
                     st,
                     s_lsm6dsv.last_who_am_i);
            return ESP_FAIL;
        }
        s_lsm6dsv_ready = true;
        ESP_LOGI(TAG, "LSM6DSV alive: who=0x%02x, XL=120 Hz +/-4 g, gyro=120 Hz +/-2000 dps",
                 s_lsm6dsv.last_who_am_i);
    }

    return ESP_OK;
}

esp_err_t lsm6dsv_debug_sample(void)
{
    serial_select_output(UART_OUTPUT_TEXT);
    ESP_RETURN_ON_ERROR(lsm6dsv_ensure_ready(), TAG, "LSM6DSV bring-up failed");

    lsm6dsv_sample_t s = {0};
    lsm6dsv_status_t st = lsm6dsv_read_sample_resynced(&s);
    if (st != LSM6DSV_OK) {
        ESP_LOGE(TAG, "LSM6DSV sample failed: %d", st);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG,
             "LSM6DSV sample: acc[g]=%.5f %.5f %.5f gyro[dps]=%.3f %.3f %.3f temp=%.2f C",
             s.xl_x_g, s.xl_y_g, s.xl_z_g,
             s.g_x_dps, s.g_y_dps, s.g_z_dps,
             s.temp_c);
    return ESP_OK;
}

static esp_err_t scl3300_ensure_ready(void)
{
    if (!s_scl3300_ready) {
        scl3300_config_t cfg = SCL3300_CONFIG_DEFAULT();
        cfg.mode = SCL3300_MODE_4;
        cfg.enable_angle_outputs = true;
        scl3300_status_t st = scl3300_init(&s_scl3300, &cfg);
        if (st != SCL3300_OK) {
            ESP_LOGE(TAG, "SCL3300 init failed: st=%d who=0x%02x",
                     st,
                     s_scl3300.last_whoami);
            return ESP_FAIL;
        }
        s_scl3300_ready = true;
        ESP_LOGI(TAG, "SCL3300 alive: who=0x%02x, mode 4 low-noise inclination, angles enabled",
                 s_scl3300.last_whoami);
    }

    return ESP_OK;
}

esp_err_t scl3300_debug_sample(void)
{
    serial_select_output(UART_OUTPUT_TEXT);
    ESP_RETURN_ON_ERROR(scl3300_ensure_ready(), TAG, "SCL3300 bring-up failed");

    scl3300_sample_t s = {0};
    scl3300_status_t st = scl3300_read_sample(&s_scl3300, &s);
    if (st != SCL3300_OK) {
        ESP_LOGE(TAG, "SCL3300 sample failed: %d", st);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG,
             "SCL3300 sample: acc[g]=%.5f %.5f %.5f angle[deg]=%.4f %.4f %.4f temp=%.2f C",
             s.acc_x_g, s.acc_y_g, s.acc_z_g,
             s.ang_x_deg, s.ang_y_deg, s.ang_z_deg,
             s.temp_c);
    return ESP_OK;
}
