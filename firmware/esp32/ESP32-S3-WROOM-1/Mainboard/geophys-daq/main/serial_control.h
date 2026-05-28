#ifndef SERIAL_CONTROL_H
#define SERIAL_CONTROL_H

#include "esp_err.h"

#include <stdint.h>

typedef enum {
    UART_OUTPUT_TEXT = 0,
    UART_OUTPUT_ADC_BINARY,
} uart_output_t;

esp_err_t serial_control_init(void);
esp_err_t serial_command_task_start(void);
void serial_select_output(uart_output_t output);
void serial_write_adc_packet(const int32_t *samples, uint8_t status, uint32_t frame_idx);

#endif /* SERIAL_CONTROL_H */
