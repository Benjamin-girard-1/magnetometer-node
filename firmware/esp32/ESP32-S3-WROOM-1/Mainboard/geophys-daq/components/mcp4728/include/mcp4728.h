/**
 * @file mcp4728.h
 * @brief Minimal MCP4728 quad-DAC driver for magnetic offset trim.
 */

#ifndef MCP4728_H_
#define MCP4728_H_

#include <stdint.h>

#include "esp_err.h"
#include "hal/gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MCP4728_CH_A = 0,
    MCP4728_CH_B = 1,
    MCP4728_CH_C = 2,
    MCP4728_CH_D = 3,
    MCP4728_NUM_CHANNELS,
} mcp4728_channel_t;

typedef struct {
    gpio_num_t sda_gpio;
    gpio_num_t scl_gpio;
    uint8_t address;
    uint32_t scl_hz;
} mcp4728_config_t;

typedef struct {
    void *bus;
    void *dev;
} mcp4728_t;

#define MCP4728_DEFAULT_ADDRESS  0x60U

esp_err_t mcp4728_init(mcp4728_t *dac, const mcp4728_config_t *cfg);
esp_err_t mcp4728_configure_internal_ref_gain1(mcp4728_t *dac);
esp_err_t mcp4728_write_all_volatile(mcp4728_t *dac,
                                     const uint16_t code[MCP4728_NUM_CHANNELS]);
esp_err_t mcp4728_power_down_all_volatile(mcp4728_t *dac,
                                          const uint16_t code[MCP4728_NUM_CHANNELS]);
esp_err_t mcp4728_deinit(mcp4728_t *dac);

#ifdef __cplusplus
}
#endif

#endif /* MCP4728_H_ */
