/**
 * @file mcp4728.c
 * @brief Minimal volatile-write support for the MCP4728 quad DAC.
 */

#include "mcp4728.h"

#include "driver/i2c_master.h"

#define MCP4728_DEFAULT_SCL_HZ      100000U
#define MCP4728_TIMEOUT_MS          100
#define MCP4728_12BIT_MASK          0x0FFFU
#define MCP4728_FAST_WRITE_PD_500K  0x30U
#define MCP4728_CMD_VREF_ALL_INTERNAL  0x8FU
#define MCP4728_CMD_GAIN_ALL_X1        0xC0U

esp_err_t mcp4728_init(mcp4728_t *dac, const mcp4728_config_t *cfg)
{
    if (dac == NULL || cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = cfg->sda_gpio,
        .scl_io_num = cfg->scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) {
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = cfg->address ? cfg->address : MCP4728_DEFAULT_ADDRESS,
        .scl_speed_hz = cfg->scl_hz ? cfg->scl_hz : MCP4728_DEFAULT_SCL_HZ,
    };

    i2c_master_dev_handle_t dev = NULL;
    err = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (err != ESP_OK) {
        (void)i2c_del_master_bus(bus);
        return err;
    }

    dac->bus = bus;
    dac->dev = dev;
    return ESP_OK;
}

esp_err_t mcp4728_configure_internal_ref_gain1(mcp4728_t *dac)
{
    if (dac == NULL || dac->dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t vref_cmd = MCP4728_CMD_VREF_ALL_INTERNAL;
    esp_err_t err = i2c_master_transmit((i2c_master_dev_handle_t)dac->dev,
                                        &vref_cmd,
                                        sizeof(vref_cmd),
                                        MCP4728_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    const uint8_t gain_cmd = MCP4728_CMD_GAIN_ALL_X1;
    return i2c_master_transmit((i2c_master_dev_handle_t)dac->dev,
                               &gain_cmd,
                               sizeof(gain_cmd),
                               MCP4728_TIMEOUT_MS);
}

esp_err_t mcp4728_write_all_volatile(mcp4728_t *dac,
                                     const uint16_t code[MCP4728_NUM_CHANNELS])
{
    if (dac == NULL || dac->dev == NULL || code == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx[MCP4728_NUM_CHANNELS * 2];
    for (uint8_t ch = 0; ch < MCP4728_NUM_CHANNELS; ++ch) {
        uint16_t v = code[ch] & MCP4728_12BIT_MASK;
        tx[(2U * ch) + 0U] = (uint8_t)(v >> 8);
        tx[(2U * ch) + 1U] = (uint8_t)v;
    }

    return i2c_master_transmit((i2c_master_dev_handle_t)dac->dev,
                               tx,
                               sizeof(tx),
                               MCP4728_TIMEOUT_MS);
}

esp_err_t mcp4728_power_down_all_volatile(mcp4728_t *dac,
                                          const uint16_t code[MCP4728_NUM_CHANNELS])
{
    if (dac == NULL || dac->dev == NULL || code == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx[MCP4728_NUM_CHANNELS * 2];
    for (uint8_t ch = 0; ch < MCP4728_NUM_CHANNELS; ++ch) {
        uint16_t v = code[ch] & MCP4728_12BIT_MASK;
        tx[(2U * ch) + 0U] = (uint8_t)(MCP4728_FAST_WRITE_PD_500K | (uint8_t)(v >> 8));
        tx[(2U * ch) + 1U] = (uint8_t)v;
    }

    return i2c_master_transmit((i2c_master_dev_handle_t)dac->dev,
                               tx,
                               sizeof(tx),
                               MCP4728_TIMEOUT_MS);
}

esp_err_t mcp4728_deinit(mcp4728_t *dac)
{
    if (dac == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    if (dac->dev != NULL) {
        err = i2c_master_bus_rm_device((i2c_master_dev_handle_t)dac->dev);
        dac->dev = NULL;
    }
    if (dac->bus != NULL) {
        esp_err_t bus_err = i2c_del_master_bus((i2c_master_bus_handle_t)dac->bus);
        if (err == ESP_OK) {
            err = bus_err;
        }
        dac->bus = NULL;
    }
    return err;
}
