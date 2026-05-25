/**
 * @file hmc100x.h
 * @brief Set/reset strap driver for HMC1001/HMC1002 magnetic sensors.
 */

#ifndef HMC100X_H_
#define HMC100X_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t boost_settle_ms;
    uint32_t pulse_us;
    uint32_t inter_pulse_ms;
    uint32_t bridge_restore_ms;
    uint32_t period_ms;
    uint32_t active_slot_mask;
    bool disable_bridge_during_sequence;
} hmc100x_config_t;

#define HMC100X_DEFAULT_CONFIG                          \
    ((hmc100x_config_t){                                \
        .boost_settle_ms = 100U,                        \
        .pulse_us = 200U,                               \
        .inter_pulse_ms = 10U,                          \
        .bridge_restore_ms = 20U,                       \
        .period_ms = 60000U,                            \
        .active_slot_mask = 0x3U,                       \
        .disable_bridge_during_sequence = false,        \
    })

int hmc100x_init(const hmc100x_config_t *cfg);
int hmc100x_set_reset_sequence(void);
int hmc100x_set_only_sequence(void);
int hmc100x_reset_only_sequence(void);
int hmc100x_start_periodic_task(void);

#ifdef __cplusplus
}
#endif

#endif /* HMC100X_H_ */
