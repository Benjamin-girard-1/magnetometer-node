#ifndef ADC_STREAM_H
#define ADC_STREAM_H

#include "esp_err.h"

#include <stdbool.h>

esp_err_t adc_stream_start(void);
esp_err_t adc_power_on(void);
void adc_power_off(void);
bool adc_streaming(void);
void adc_log_drops_if_changed(void);

#endif /* ADC_STREAM_H */
