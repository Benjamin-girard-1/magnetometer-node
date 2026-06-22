#include "board_control.h"

#include "adc_stream.h"
#include "app_config.h"
#include "hmc100x.h"
#include "sd_card_debug.h"
#include "shift_register.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "board_control";

esp_err_t board_outputs_init(uint32_t magnetic_slots)
{
    sr_set_pin(SR_EN_LDO_3V3, true);
    vTaskDelay(pdMS_TO_TICKS(20));
    sr_set_pin(SR_EN_BST_10V, MAG_ENABLE_BRIDGE_9V != 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    sr_set_pin(SR_EN_INV_NEG5V, MAG_ENABLE_NEG5V != 0);
    sr_set_pin(SR_EN_BST_18V, true);
    sr_set_pin(SR_SET_1, false);
    sr_set_pin(SR_RESET_1, false);
    sr_set_pin(SR_SET_2, false);
    sr_set_pin(SR_RESET_2, false);
    adc_power_on();
    sd_mux_select_usb2641();
    vTaskDelay(pdMS_TO_TICKS(100));

    hmc100x_config_t hmc_cfg = HMC100X_DEFAULT_CONFIG;
    hmc_cfg.active_slot_mask = magnetic_slots;
    if (hmc100x_init(&hmc_cfg) != 0) {
        ESP_LOGE(TAG, "hmc100x init failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void board_log_sr_state(const char *context)
{
    ESP_LOGI(TAG, "%s: shift-register state=0x%04x", context, (unsigned)sr_get_state());
}

void board_set_keepalive(bool enable)
{
    sr_set_pin(SR_EN_KEEPALIVE, enable);
    ESP_LOGI(TAG, "EN_KEEPALIVE %s", enable ? "on" : "off");
}

void board_set_9v(bool enable)
{
    sr_set_pin(SR_EN_BST_10V, enable);
    ESP_LOGI(TAG, "+9VA boost/LDO input enable %s", enable ? "on" : "off");
}

void board_set_neg5v(bool enable)
{
    sr_set_pin(SR_EN_INV_NEG5V, enable);
    ESP_LOGI(TAG, "-5V inverter enable %s", enable ? "on" : "off");
}

void board_set_sr_pin(int pin, bool level)
{
    sr_set_pin((sr_pin_t)pin, level);
    ESP_LOGI(TAG, "SR pin %d set to %d, state=0x%04x",
             pin,
             level ? 1 : 0,
             (unsigned)sr_get_state());
}
