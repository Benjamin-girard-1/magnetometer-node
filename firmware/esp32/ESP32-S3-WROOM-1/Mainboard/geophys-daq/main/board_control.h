#ifndef BOARD_CONTROL_H
#define BOARD_CONTROL_H

#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

esp_err_t board_outputs_init(uint32_t magnetic_slots);
void board_set_keepalive(bool enable);
void board_set_9v(bool enable);
void board_set_neg5v(bool enable);
void board_set_sr_pin(int pin, bool level);
void board_log_sr_state(const char *context);

#endif /* BOARD_CONTROL_H */
