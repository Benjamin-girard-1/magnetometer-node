/**
 * @file main.c
 * @brief Top-level bring-up orchestration.
 */

#include "adc_stream.h"
#include "app_config.h"
#include "board_control.h"
#include "board_detect.h"
#include "hmc100x.h"
#include "serial_control.h"
#include "shift_register.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mag";

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(150));

    ESP_ERROR_CHECK(serial_control_init());
    serial_select_output(UART_OUTPUT_TEXT);

    if (serial_command_task_start() != ESP_OK) {
        ESP_LOGE(TAG, "failed to start UART command task");
        return;
    }

    if (sr_init() != 0) {
        ESP_LOGE(TAG, "shift register init failed");
        return;
    }
    if (board_detect_init() != 0) {
        ESP_LOGE(TAG, "board detect init failed");
        return;
    }

    board_slot_info_t slot1 = board_detect_read_slot(BOARD_SLOT_1);
    board_slot_info_t slot2 = board_detect_read_slot(BOARD_SLOT_2);
    uint32_t magnetic_slots = board_detect_magnetic_slot_mask();
    ESP_LOGI(TAG,
             "Expansion slots: slot1=%s id=%d, slot2=%s id=%d, magnetic_mask=0x%lx",
             board_card_type_name(slot1.card),
             slot1.id_level,
             board_card_type_name(slot2.card),
             slot2.id_level,
             (unsigned long)magnetic_slots);

    if (board_outputs_init(magnetic_slots) != ESP_OK) {
        return;
    }

#if MAG_SET_RESET_ENABLE
    if (magnetic_slots != 0U && hmc100x_set_reset_sequence() != 0) {
        ESP_LOGE(TAG, "initial HMC100x set/reset failed");
        return;
    }
    if (hmc100x_start_periodic_task() != 0) {
        ESP_LOGE(TAG, "failed to start HMC100x set/reset task");
        return;
    }
#else
    ESP_LOGI(TAG, "HMC100x set/reset pulses disabled");
#endif

#if MAG_ENABLE_ADC_STREAM_ON_BOOT
    if (adc_stream_start() != ESP_OK) {
        return;
    }
#else
    ESP_LOGI(TAG, "ADC stream is off. Send 'ADC' for binary stream or 'SD' for SD test.");
#endif

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        adc_log_drops_if_changed();
    }
}
