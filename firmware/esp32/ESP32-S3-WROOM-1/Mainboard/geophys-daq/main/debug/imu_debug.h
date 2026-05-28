#ifndef IMU_DEBUG_H
#define IMU_DEBUG_H

#include "esp_err.h"

esp_err_t lsm6dsv_debug_sample(void);
esp_err_t scl3300_debug_sample(void);

#endif /* IMU_DEBUG_H */
