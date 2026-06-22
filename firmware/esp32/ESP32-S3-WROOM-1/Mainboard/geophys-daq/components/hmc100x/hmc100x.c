/**
 * @file hmc100x.c
 * @brief Set/reset strap sequencing for the HMC1001/HMC1002 pair.
 */

#include "hmc100x.h"

#include "shift_register.h"

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "hmc100x";

#define HMC100X_SET_MASK      ((uint16_t)((1U << SR_SET_1) | (1U << SR_SET_2)))
#define HMC100X_RESET_MASK    ((uint16_t)((1U << SR_RESET_1) | (1U << SR_RESET_2)))
#define HMC100X_STRAP_MASK    ((uint16_t)(HMC100X_SET_MASK | HMC100X_RESET_MASK))
#define HMC100X_SLOT_1_MASK   (0x1U)
#define HMC100X_SLOT_2_MASK   (0x2U)
#define HMC100X_BOOST_18V     ((uint16_t)(1U << SR_EN_BST_18V))
#define HMC100X_BRIDGE_9VA    ((uint16_t)(1U << SR_EN_BST_10V))

#define HMC100X_TASK_STACK_WORDS  2048U
#define HMC100X_TASK_PRIORITY     5U

static hmc100x_config_t s_cfg = HMC100X_DEFAULT_CONFIG;
static TaskHandle_t s_periodic_task;

static void delay_ms(uint32_t ms)
{
    if (ms > 0U) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

static int write_masked(uint16_t clear_mask, uint16_t set_mask)
{
    return sr_update_pins(clear_mask, set_mask);
}

static int force_straps_idle(void)
{
    return write_masked(HMC100X_STRAP_MASK, 0U);
}

static uint16_t slot_set_mask(void)
{
    uint16_t mask = 0U;
    if ((s_cfg.active_slot_mask & HMC100X_SLOT_1_MASK) != 0U) {
        mask |= (uint16_t)(1U << SR_SET_1);
    }
    if ((s_cfg.active_slot_mask & HMC100X_SLOT_2_MASK) != 0U) {
        mask |= (uint16_t)(1U << SR_SET_2);
    }
    return mask;
}

static uint16_t slot_reset_mask(void)
{
    uint16_t mask = 0U;
    if ((s_cfg.active_slot_mask & HMC100X_SLOT_1_MASK) != 0U) {
        mask |= (uint16_t)(1U << SR_RESET_1);
    }
    if ((s_cfg.active_slot_mask & HMC100X_SLOT_2_MASK) != 0U) {
        mask |= (uint16_t)(1U << SR_RESET_2);
    }
    return mask;
}

static int pulse_straps(uint16_t active_mask, uint16_t opposite_mask)
{
    int err = write_masked(opposite_mask, 0U);
    if (err != 0) {
        return err;
    }

    err = write_masked((uint16_t)(opposite_mask | active_mask), active_mask);
    if (err != 0) {
        (void)force_straps_idle();
        return err;
    }

    esp_rom_delay_us(s_cfg.pulse_us);
    return force_straps_idle();
}

static int prepare_boost_for_sequence(void)
{
    int err = force_straps_idle();
    if (err != 0) {
        return err;
    }

    if (s_cfg.disable_bridge_during_sequence) {
        err = write_masked(HMC100X_BRIDGE_9VA, 0U);
        if (err != 0) {
            return err;
        }
        delay_ms(s_cfg.bridge_restore_ms);
    }

    if (!sr_get_pin(SR_EN_BST_18V)) {
        err = write_masked(0U, HMC100X_BOOST_18V);
        if (err != 0) {
            return err;
        }
        delay_ms(s_cfg.boost_settle_ms);
    }

    return 0;
}

static void finish_boost_sequence(bool restore_bridge)
{
    (void)force_straps_idle();

    if (s_cfg.disable_bridge_during_sequence && restore_bridge) {
        (void)write_masked(0U, HMC100X_BRIDGE_9VA);
        delay_ms(s_cfg.bridge_restore_ms);
    }
}

static int one_pulse_sequence(uint16_t active_mask, uint16_t opposite_mask,
                              const char *name)
{
    const bool restore_bridge = sr_get_pin(SR_EN_BST_10V);
    if ((active_mask | opposite_mask) == 0U) {
        return 0;
    }

    int err = prepare_boost_for_sequence();
    if (err == 0) {
        err = pulse_straps(active_mask, opposite_mask);
    }
    finish_boost_sequence(restore_bridge);

    if (err != 0) {
        ESP_LOGE(TAG, "%s sequence failed: %d", name, err);
        return err;
    }

    ESP_LOGI(TAG, "%s sequence complete", name);
    return 0;
}

int hmc100x_init(const hmc100x_config_t *cfg)
{
    s_cfg = (cfg != NULL) ? *cfg : HMC100X_DEFAULT_CONFIG;

    if (s_cfg.pulse_us == 0U) {
        s_cfg.pulse_us = HMC100X_DEFAULT_CONFIG.pulse_us;
    }
    if (s_cfg.period_ms == 0U) {
        s_cfg.period_ms = HMC100X_DEFAULT_CONFIG.period_ms;
    }
    s_cfg.active_slot_mask &= (HMC100X_SLOT_1_MASK | HMC100X_SLOT_2_MASK);

    const bool boost_was_on = sr_get_pin(SR_EN_BST_18V);
    int err = write_masked(HMC100X_STRAP_MASK, HMC100X_BOOST_18V);
    if (err != 0) {
        ESP_LOGE(TAG, "failed to idle set/reset straps and enable 18V boost");
        return err;
    }
    if (!boost_was_on) {
        delay_ms(s_cfg.boost_settle_ms);
    }

    ESP_LOGI(TAG, "set/reset driver ready: pulse=%lu us period=%lu ms slots=0x%lx bridge_off=%u",
             (unsigned long)s_cfg.pulse_us,
             (unsigned long)s_cfg.period_ms,
             (unsigned long)s_cfg.active_slot_mask,
             s_cfg.disable_bridge_during_sequence ? 1U : 0U);
    return 0;
}

int hmc100x_set_reset_sequence(void)
{
    const uint16_t set_mask = slot_set_mask();
    const uint16_t reset_mask = slot_reset_mask();
    const bool restore_bridge = sr_get_pin(SR_EN_BST_10V);
    if ((set_mask | reset_mask) == 0U) {
        return 0;
    }

    int err = prepare_boost_for_sequence();
    if (err == 0) {
        err = pulse_straps(reset_mask, set_mask);
    }
    if (err == 0) {
        delay_ms(s_cfg.inter_pulse_ms);
        err = pulse_straps(set_mask, reset_mask);
    }

    finish_boost_sequence(restore_bridge);

    if (err != 0) {
        ESP_LOGE(TAG, "set/reset sequence failed: %d", err);
        return err;
    }

    ESP_LOGI(TAG, "set/reset sequence complete");
    return 0;
}

int hmc100x_set_only_sequence(void)
{
    return one_pulse_sequence(slot_set_mask(), slot_reset_mask(), "set-only");
}

int hmc100x_reset_only_sequence(void)
{
    return one_pulse_sequence(slot_reset_mask(), slot_set_mask(), "reset-only");
}

static void periodic_task(void *arg)
{
    (void)arg;

    while (1) {
        delay_ms(s_cfg.period_ms);
        (void)hmc100x_set_reset_sequence();
    }
}

int hmc100x_start_periodic_task(void)
{
    if (s_periodic_task != NULL) {
        return 0;
    }

    BaseType_t ok = xTaskCreate(periodic_task,
                                "hmc100x_sr",
                                HMC100X_TASK_STACK_WORDS,
                                NULL,
                                HMC100X_TASK_PRIORITY,
                                &s_periodic_task);
    return (ok == pdPASS) ? 0 : -1;
}
