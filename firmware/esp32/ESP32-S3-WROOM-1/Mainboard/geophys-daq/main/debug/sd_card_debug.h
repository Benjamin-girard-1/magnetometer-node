#ifndef SD_CARD_DEBUG_H
#define SD_CARD_DEBUG_H

#include "esp_err.h"

#include <stdbool.h>

void sd_log_shift_register_state(const char *context);
void sd_mux_select_esp32(void);
void sd_mux_select_esp32_level(bool sel_level);
void sd_mux_idle(void);
void sd_safe_idle(void);
void sd_mux_select_usb2641(void);
void usb2641_set_reset(bool released);
void usb2641_reset_pulse(void);
esp_err_t run_sd_card_ops(bool write_test, bool read_test);
esp_err_t run_sd_card_test(void);
esp_err_t run_sd_card_probe(bool sel_level);
esp_err_t run_sd_card_probe_all(void);
esp_err_t run_sd_card_spi_probe_all(void);
void run_sd_line_probe_all(void);

#endif /* SD_CARD_DEBUG_H */
