/**
 * @file main.c
 * @brief Magnetic expansion-card ADC bring-up.
 */

#include "ad7779.h"
#include "ad7779_hal.h"
#include "board_detect.h"
#include "hmc100x.h"
#include "mcp4728.h"
#include "shift_register.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "mag";

#define ADC_FULL_SCALE_CODE       8388608.0f
#define ADC_REF_V                 2.5f
#define MAG_ADC_GAIN              1.0f
#define MAG_ODR_HZ                1000U
#define SERIAL_STUDIO_DECIMATION  1U
#define SERIAL_STUDIO_RATE_HZ     (MAG_ODR_HZ / SERIAL_STUDIO_DECIMATION)
#define MAG_UART_BAUD             921600U
#define MAG_UART_RX_BUF_BYTES     1024
#define MAG_UART_TX_BUF_BYTES     16384
#define MAG_SLOT_1_I2C_SDA        GPIO_NUM_41
#define MAG_SLOT_1_I2C_SCL        GPIO_NUM_40
#define MAG_SLOT_2_I2C_SDA        GPIO_NUM_1
#define MAG_SLOT_2_I2C_SCL        GPIO_NUM_2
#define MAG_THERMAL_ISOLATION_TEST 1
#define MAG_ENABLE_BRIDGE_9V      0
#define MAG_ENABLE_NEG5V          0
#define MAG_ALLOW_BRIDGE_9V_COMMANDS 1
#define MAG_ALLOW_SET_RESET_COMMANDS 1
#define MAG_OFFSET_DAC_CENTER_CODE 2048
#define MAG_OFFSET_X_TRIM_LSB     0
#define MAG_OFFSET_Y_TRIM_LSB     0
#define MAG_OFFSET_Z_TRIM_LSB     0
#define MAG_OFFSET_MANUAL_ENABLE  1
#define MAG_OFFSET_DAC_LOG_MANUAL 0
#define MAG_OFFSET_DAC_DEFAULT_ENABLE 0
#define MAG_SET_RESET_ENABLE      0
#define MAG_OFFSET_AUTOZERO_ENABLE 0
#define MAG_OFFSET_AUTOZERO_WINDOW_MS 250
#define MAG_OFFSET_AUTOZERO_STEP_LSB 8
#define MAG_OFFSET_AUTOZERO_CLIPPED_STEP_LSB 64
#define MAG_OFFSET_AUTOZERO_MAX_DELTA_LSB 100
#define MAG_OFFSET_AUTOZERO_DEADBAND_CODE 50000
#define MAG_OFFSET_AUTOZERO_CLIP_CODE 7500000
#define MAG_OFFSET_AUTOZERO_IMPROVE_CODE 25000
#define MAG_OFFSET_AUTOZERO_MAX_ITER 160

#define MAG_PACKET_SYNC0          0xA5U
#define MAG_PACKET_SYNC1          0x5AU
#define MAG_PACKET_PAYLOAD_LEN    (4U + (AD7779_NUM_CHANNELS * 4U) + 1U)
#define MAG_PACKET_TOTAL_LEN      (2U + 1U + MAG_PACKET_PAYLOAD_LEN + 1U)

typedef struct {
    board_slot_t slot;
    mcp4728_t dac;
    uint16_t code[MCP4728_NUM_CHANNELS];
    int8_t direction[3];
    int64_t last_abs_error[3];
    bool have_last_error[3];
    bool ready;
    bool dac_enabled;
} offset_dac_ctx_t;

static offset_dac_ctx_t s_offset_dac_ctx[BOARD_SLOT_COUNT];
static offset_dac_ctx_t *s_autozero_ctx;
#if MAG_OFFSET_AUTOZERO_ENABLE
static portMUX_TYPE s_adc_accum_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_adc_accum[3];
static uint32_t s_adc_accum_count;
static volatile bool s_autozero_requested;
#endif

static uint8_t checksum8(const uint8_t *data, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum = (uint8_t)(sum + data[i]);
    }
    return (uint8_t)(0U - sum);
}

static void put_u32_le(uint8_t *dst, uint32_t v)
{
    dst[0] = (uint8_t)(v);
    dst[1] = (uint8_t)(v >> 8);
    dst[2] = (uint8_t)(v >> 16);
    dst[3] = (uint8_t)(v >> 24);
}

static uint16_t dac_code_from_trim(int trim_lsb)
{
    /* MCP4728 internal 2.048 V reference, gain x1: 0.5 mV/LSB. */
    int code = MAG_OFFSET_DAC_CENTER_CODE + trim_lsb;
    if (code < 0) {
        code = 0;
    } else if (code > 4095) {
        code = 4095;
    }
    return (uint16_t)code;
}

#if MAG_OFFSET_AUTOZERO_ENABLE
static int64_t abs_i64(int64_t v)
{
    return v < 0 ? -v : v;
}
#endif

static uint16_t clamp_dac_code(int code)
{
    const int min_code = MAG_OFFSET_DAC_CENTER_CODE - MAG_OFFSET_AUTOZERO_MAX_DELTA_LSB;
    const int max_code = MAG_OFFSET_DAC_CENTER_CODE + MAG_OFFSET_AUTOZERO_MAX_DELTA_LSB;

    if (code < min_code) {
        code = min_code;
    } else if (code > max_code) {
        code = max_code;
    }
    if (code < 0) {
        code = 0;
    } else if (code > 4095) {
        code = 4095;
    }
    return (uint16_t)code;
}

#if MAG_OFFSET_AUTOZERO_ENABLE
static bool adc_accum_snapshot(int64_t avg[3], uint32_t *count)
{
    portENTER_CRITICAL(&s_adc_accum_lock);
    uint32_t n = s_adc_accum_count;
    if (n == 0U) {
        portEXIT_CRITICAL(&s_adc_accum_lock);
        return false;
    }

    for (uint8_t ch = 0; ch < 3U; ++ch) {
        avg[ch] = s_adc_accum[ch] / (int64_t)n;
        s_adc_accum[ch] = 0;
    }
    s_adc_accum_count = 0;
    portEXIT_CRITICAL(&s_adc_accum_lock);

    *count = n;
    return true;
}

static void adc_accum_reset(void)
{
    portENTER_CRITICAL(&s_adc_accum_lock);
    s_adc_accum[0] = 0;
    s_adc_accum[1] = 0;
    s_adc_accum[2] = 0;
    s_adc_accum_count = 0;
    portEXIT_CRITICAL(&s_adc_accum_lock);
}

static void offset_autozero_reset_state(offset_dac_ctx_t *ctx)
{
    for (uint8_t ch = 0; ch < 3U; ++ch) {
        ctx->direction[ch] = 0;
        ctx->last_abs_error[ch] = 0;
        ctx->have_last_error[ch] = false;
    }
}
#endif

static void offset_set_manual_codes(uint16_t x, uint16_t y, uint16_t z)
{
#if MAG_THERMAL_ISOLATION_TEST
    (void)x;
    (void)y;
    (void)z;
    ESP_LOGW(TAG, "DAC manual command ignored: thermal isolation test keeps offset DAC off");
    return;
#else
    offset_dac_ctx_t *ctx = s_autozero_ctx;
    if (ctx == NULL || !ctx->ready) {
        ESP_LOGW(TAG, "DAC manual command ignored: no DAC ready");
        return;
    }

    ctx->code[MCP4728_CH_A] = clamp_dac_code((int)x);
    ctx->code[MCP4728_CH_B] = clamp_dac_code((int)y);
    ctx->code[MCP4728_CH_C] = clamp_dac_code((int)z);
    ctx->code[MCP4728_CH_D] = MAG_OFFSET_DAC_CENTER_CODE;

    if (!ctx->dac_enabled) {
        ESP_LOGI(TAG, "DAC manual codes stored while DAC outputs are disabled");
        return;
    }

    esp_err_t err = mcp4728_write_all_volatile(&ctx->dac, ctx->code);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DAC manual write failed: %s", esp_err_to_name(err));
        return;
    }

#if MAG_OFFSET_DAC_LOG_MANUAL
    ESP_LOGI(TAG, "DAC manual codes: X=%u Y=%u Z=%u REF=%u",
             ctx->code[MCP4728_CH_A],
             ctx->code[MCP4728_CH_B],
             ctx->code[MCP4728_CH_C],
             ctx->code[MCP4728_CH_D]);
#endif
#endif
}

static void offset_set_dac_enabled(bool enable)
{
    offset_dac_ctx_t *ctx = s_autozero_ctx;
    if (ctx == NULL || !ctx->ready) {
        ESP_LOGW(TAG, "DAC enable command ignored: no DAC ready");
        return;
    }

#if MAG_THERMAL_ISOLATION_TEST
    if (enable) {
        ESP_LOGW(TAG, "DAC enable ignored: thermal isolation test keeps offset DAC off");
    }
    enable = false;
#endif

    esp_err_t err = enable
        ? mcp4728_write_all_volatile(&ctx->dac, ctx->code)
        : mcp4728_power_down_all_volatile(&ctx->dac, ctx->code);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DAC %s command failed: %s",
                 enable ? "enable" : "disable",
                 esp_err_to_name(err));
        return;
    }

    ctx->dac_enabled = enable;
    ESP_LOGI(TAG, "DAC outputs %s", enable ? "enabled" : "disabled");
}

#if MAG_OFFSET_AUTOZERO_ENABLE
static void offset_autozero_task(void *arg)
{
    offset_dac_ctx_t *ctx = (offset_dac_ctx_t *)arg;

    while (1) {
        while (!s_autozero_requested) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        s_autozero_requested = false;
        offset_autozero_reset_state(ctx);
        adc_accum_reset();
        ESP_LOGI(TAG, "offset auto-zero started");

        bool converged = false;
        for (uint32_t iter = 0U; iter < MAG_OFFSET_AUTOZERO_MAX_ITER; ++iter) {
        vTaskDelay(pdMS_TO_TICKS(MAG_OFFSET_AUTOZERO_WINDOW_MS));

        int64_t avg[3] = {0};
        uint32_t count = 0;
        if (!ctx->ready || !adc_accum_snapshot(avg, &count)) {
            continue;
        }

        bool changed = false;
        converged = true;
        for (uint8_t ch = 0; ch < 3U; ++ch) {
            int64_t err_abs = abs_i64(avg[ch]);
            if (err_abs <= MAG_OFFSET_AUTOZERO_DEADBAND_CODE) {
                continue;
            }
            converged = false;

            bool clipped = err_abs >= MAG_OFFSET_AUTOZERO_CLIP_CODE;
            if (!ctx->have_last_error[ch]) {
                ctx->direction[ch] = avg[ch] > 0 ? -1 : 1;
                ctx->have_last_error[ch] = true;
            } else if (err_abs + MAG_OFFSET_AUTOZERO_IMPROVE_CODE >= ctx->last_abs_error[ch]) {
                ctx->direction[ch] = (int8_t)-ctx->direction[ch];
            }

            int step = clipped ? MAG_OFFSET_AUTOZERO_CLIPPED_STEP_LSB :
                       MAG_OFFSET_AUTOZERO_STEP_LSB;
            int next = (int)ctx->code[ch] +
                       ((int)ctx->direction[ch] * step);
            uint16_t clamped = clamp_dac_code(next);
            if (clamped == ctx->code[ch] && clipped) {
                ctx->direction[ch] = (int8_t)-ctx->direction[ch];
                next = (int)ctx->code[ch] + ((int)ctx->direction[ch] * step);
                clamped = clamp_dac_code(next);
            }
            if (clamped != ctx->code[ch]) {
                ctx->code[ch] = clamped;
                changed = true;
            }
            ctx->last_abs_error[ch] = err_abs;
        }

        if (changed && ctx->dac_enabled) {
            esp_err_t err = mcp4728_write_all_volatile(&ctx->dac, ctx->code);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "offset auto-zero DAC write failed: %s",
                         esp_err_to_name(err));
            }
        }

        if ((iter % 5U) == 0U || converged) {
            ESP_LOGI(TAG,
                     "autozero n=%lu avg=[%lld,%lld,%lld] dac=[%u,%u,%u]",
                     (unsigned long)count,
                     (long long)avg[0],
                     (long long)avg[1],
                     (long long)avg[2],
                     ctx->code[MCP4728_CH_A],
                     ctx->code[MCP4728_CH_B],
                     ctx->code[MCP4728_CH_C]);
        }

        if (converged) {
            break;
        }
        }

        ESP_LOGI(TAG, "offset auto-zero %s dac=[%u,%u,%u]",
                 converged ? "complete" : "stopped at iteration limit",
                 ctx->code[MCP4728_CH_A],
                 ctx->code[MCP4728_CH_B],
                 ctx->code[MCP4728_CH_C]);
    }
}
#endif

static void uart_command_task(void *arg)
{
    (void)arg;
    uint8_t rx[32];
    char line[32];
    size_t len = 0;

    while (1) {
        int n = uart_read_bytes(UART_NUM_0, rx, sizeof(rx), pdMS_TO_TICKS(100));
        for (int i = 0; i < n; ++i) {
            char c = (char)rx[i];
            if (c == '\r' || c == '\n') {
                line[len] = '\0';
                bool handled = false;
#if MAG_OFFSET_AUTOZERO_ENABLE
                if (len > 0U && (strcmp(line, "Z") == 0 || strcmp(line, "ZERO") == 0)) {
                    s_autozero_requested = true;
                    ESP_LOGI(TAG, "received ZERO command");
                    handled = true;
                }
#else
                if (len > 0U && (strcmp(line, "Z") == 0 || strcmp(line, "ZERO") == 0)) {
                    ESP_LOGI(TAG, "ZERO command ignored: auto-zero firmware task is disabled");
                    handled = true;
                }
#endif
                if (!handled && strncmp(line, "DAC ", 4) == 0) {
                    unsigned x = 0, y = 0, z = 0;
                    if (sscanf(&line[4], "%u %u %u", &x, &y, &z) == 3) {
                        offset_set_manual_codes((uint16_t)x, (uint16_t)y, (uint16_t)z);
                    } else {
                        ESP_LOGW(TAG, "bad DAC command: %s", line);
                    }
                    handled = true;
                }
                if (!handled && strncmp(line, "9V ", 3) == 0) {
#if MAG_THERMAL_ISOLATION_TEST && !MAG_ALLOW_BRIDGE_9V_COMMANDS
                    if (strcmp(&line[3], "1") == 0 ||
                        strcmp(&line[3], "ON") == 0 ||
                        strcmp(&line[3], "on") == 0 ||
                        strcmp(&line[3], "0") == 0 ||
                        strcmp(&line[3], "OFF") == 0 ||
                        strcmp(&line[3], "off") == 0) {
                        handled = true;
                    }
                    if (handled) {
                        sr_set_pin(SR_EN_BST_10V, false);
                        ESP_LOGW(TAG, "+9VA command ignored: thermal isolation test keeps bridge off");
                    } else {
                        ESP_LOGW(TAG, "bad 9V command: %s", line);
                        handled = true;
                    }
#else
                    bool enable = false;
                    if (strcmp(&line[3], "1") == 0 ||
                        strcmp(&line[3], "ON") == 0 ||
                        strcmp(&line[3], "on") == 0) {
                        enable = true;
                        handled = true;
                    } else if (strcmp(&line[3], "0") == 0 ||
                               strcmp(&line[3], "OFF") == 0 ||
                               strcmp(&line[3], "off") == 0) {
                        enable = false;
                        handled = true;
                    }
                    if (handled) {
                        sr_set_pin(SR_EN_BST_10V, enable);
                        ESP_LOGI(TAG, "+9VA boost/LDO input enable %s",
                                 enable ? "on" : "off");
                    } else {
                        ESP_LOGW(TAG, "bad 9V command: %s", line);
                        handled = true;
                    }
#endif
                }
                if (!handled && strncmp(line, "DACEN ", 6) == 0) {
                    bool enable = false;
                    if (strcmp(&line[6], "1") == 0 ||
                        strcmp(&line[6], "ON") == 0 ||
                        strcmp(&line[6], "on") == 0) {
                        enable = true;
                        handled = true;
                    } else if (strcmp(&line[6], "0") == 0 ||
                               strcmp(&line[6], "OFF") == 0 ||
                               strcmp(&line[6], "off") == 0) {
                        enable = false;
                        handled = true;
                    }
                    if (handled) {
                        offset_set_dac_enabled(enable);
                    } else {
                        ESP_LOGW(TAG, "bad DACEN command: %s", line);
                        handled = true;
                    }
                }
                if (!handled && (strcmp(line, "SR") == 0 ||
                                 strcmp(line, "SETRESET") == 0)) {
#if MAG_THERMAL_ISOLATION_TEST && !MAG_ALLOW_SET_RESET_COMMANDS
                    ESP_LOGW(TAG, "SETRESET ignored: thermal isolation test keeps 18V set/reset off");
#else
                    ESP_LOGI(TAG, "received SETRESET command");
                    if (hmc100x_set_reset_sequence() != 0) {
                        ESP_LOGW(TAG, "SETRESET command failed");
                    }
#endif
                    handled = true;
                }
                if (!handled && strcmp(line, "SET") == 0) {
#if MAG_THERMAL_ISOLATION_TEST && !MAG_ALLOW_SET_RESET_COMMANDS
                    ESP_LOGW(TAG, "SET ignored: thermal isolation test keeps 18V set/reset off");
#else
                    ESP_LOGI(TAG, "received SET command");
                    if (hmc100x_set_only_sequence() != 0) {
                        ESP_LOGW(TAG, "SET command failed");
                    }
#endif
                    handled = true;
                }
                if (!handled && strcmp(line, "RESET") == 0) {
#if MAG_THERMAL_ISOLATION_TEST && !MAG_ALLOW_SET_RESET_COMMANDS
                    ESP_LOGW(TAG, "RESET ignored: thermal isolation test keeps 18V set/reset off");
#else
                    ESP_LOGI(TAG, "received RESET command");
                    if (hmc100x_reset_only_sequence() != 0) {
                        ESP_LOGW(TAG, "RESET command failed");
                    }
#endif
                }
                len = 0;
            } else if (len < (sizeof(line) - 1U)) {
                line[len++] = c;
            } else {
                len = 0;
            }
        }
    }
}

static void init_offset_dac_for_slot(board_slot_t slot)
{
    mcp4728_config_t dac_cfg = {
        .address = MCP4728_DEFAULT_ADDRESS,
        .scl_hz = 100000U,
    };

    switch (slot) {
        case BOARD_SLOT_1:
            dac_cfg.sda_gpio = MAG_SLOT_1_I2C_SDA;
            dac_cfg.scl_gpio = MAG_SLOT_1_I2C_SCL;
            break;
        case BOARD_SLOT_2:
            dac_cfg.sda_gpio = MAG_SLOT_2_I2C_SDA;
            dac_cfg.scl_gpio = MAG_SLOT_2_I2C_SCL;
            break;
        default:
            return;
    }

    offset_dac_ctx_t *ctx = &s_offset_dac_ctx[slot];
    ctx->slot = slot;
    esp_err_t err = mcp4728_init(&ctx->dac, &dac_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "slot %u offset DAC init failed: %s",
                 (unsigned)slot + 1U, esp_err_to_name(err));
        return;
    }
    err = mcp4728_configure_internal_ref_gain1(&ctx->dac);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "slot %u offset DAC config failed: %s",
                 (unsigned)slot + 1U, esp_err_to_name(err));
        (void)mcp4728_deinit(&ctx->dac);
        return;
    }

    ctx->code[MCP4728_CH_A] = dac_code_from_trim(MAG_OFFSET_X_TRIM_LSB);
    ctx->code[MCP4728_CH_B] = dac_code_from_trim(MAG_OFFSET_Y_TRIM_LSB);
    ctx->code[MCP4728_CH_C] = dac_code_from_trim(MAG_OFFSET_Z_TRIM_LSB);
    ctx->code[MCP4728_CH_D] = MAG_OFFSET_DAC_CENTER_CODE;
    ctx->direction[0] = 1;
    ctx->direction[1] = 1;
    ctx->direction[2] = 1;
    ctx->ready = false;
    ctx->dac_enabled = MAG_OFFSET_DAC_DEFAULT_ENABLE != 0;

    const uint16_t code[MCP4728_NUM_CHANNELS] = {
        dac_code_from_trim(MAG_OFFSET_X_TRIM_LSB),
        dac_code_from_trim(MAG_OFFSET_Y_TRIM_LSB),
        dac_code_from_trim(MAG_OFFSET_Z_TRIM_LSB),
        MAG_OFFSET_DAC_CENTER_CODE,
    };

    err = ctx->dac_enabled
        ? mcp4728_write_all_volatile(&ctx->dac, ctx->code)
        : mcp4728_power_down_all_volatile(&ctx->dac, ctx->code);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "slot %u offset DAC init output command failed: %s",
                 (unsigned)slot + 1U, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "slot %u offset DAC codes: X=%u Y=%u Z=%u REF=%u outputs=%s",
                 (unsigned)slot + 1U,
                 code[MCP4728_CH_A],
                 code[MCP4728_CH_B],
                 code[MCP4728_CH_C],
                 code[MCP4728_CH_D],
                 ctx->dac_enabled ? "enabled" : "powered down");
    }
    ctx->ready = true;

    if (s_autozero_ctx == NULL) {
        s_autozero_ctx = ctx;
    }
}

static void adc_sample_cb(void *ctx, const int32_t *samples,
                          uint8_t status, uint32_t frame_idx)
{
    (void)ctx;
#if MAG_OFFSET_AUTOZERO_ENABLE
    portENTER_CRITICAL(&s_adc_accum_lock);
    s_adc_accum[0] += samples[0];
    s_adc_accum[1] += samples[1];
    s_adc_accum[2] += samples[2];
    s_adc_accum_count++;
    portEXIT_CRITICAL(&s_adc_accum_lock);
#endif

    if ((frame_idx % SERIAL_STUDIO_DECIMATION) != 0U) {
        return;
    } 
    

    uint8_t pkt[MAG_PACKET_TOTAL_LEN] = {
        MAG_PACKET_SYNC0,
        MAG_PACKET_SYNC1,
        MAG_PACKET_PAYLOAD_LEN,
    };
    size_t off = 3;
    put_u32_le(&pkt[off], frame_idx);
    off += 4;
    for (uint8_t ch = 0; ch < AD7779_NUM_CHANNELS; ++ch) {
        put_u32_le(&pkt[off], (uint32_t)samples[ch]);
        off += 4;
    }
    pkt[off++] = (uint8_t)(status & 0x0FU);
    pkt[off] = checksum8(&pkt[2], MAG_PACKET_PAYLOAD_LEN + 1U);

    uart_write_bytes(UART_NUM_0, pkt, sizeof(pkt));
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(150));

    esp_err_t uart_err = uart_driver_install(UART_NUM_0,
                                             MAG_UART_RX_BUF_BYTES,
                                             MAG_UART_TX_BUF_BYTES,
                                             0,
                                             NULL,
                                             0);
    if (uart_err != ESP_OK && uart_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(uart_err));
        return;
    }
    ESP_ERROR_CHECK(uart_set_baudrate(UART_NUM_0, MAG_UART_BAUD));
    if (xTaskCreate(uart_command_task,
                    "uart_cmd",
                    2048,
                    NULL,
                    4,
                    NULL) != pdPASS) {
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

    sr_set_pin(SR_EN_LDO_3V3, true);
    vTaskDelay(pdMS_TO_TICKS(20));
    sr_set_pin(SR_EN_BST_10V, MAG_ENABLE_BRIDGE_9V != 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    sr_set_pin(SR_EN_INV_NEG5V, MAG_ENABLE_NEG5V != 0);
    sr_set_pin(SR_EN_BST_18V, false);
    sr_set_pin(SR_SET_1, false);
    sr_set_pin(SR_RESET_1, false);
    sr_set_pin(SR_SET_2, false);
    sr_set_pin(SR_RESET_2, false);
    vTaskDelay(pdMS_TO_TICKS(100));

#if MAG_THERMAL_ISOLATION_TEST
    ESP_LOGW(TAG,
             "THERMAL ISOLATION TEST: +9VA bridge off, -5VA off, "
             "18V set/reset off, offset DAC forced powered down");
#endif

    hmc100x_config_t hmc_cfg = HMC100X_DEFAULT_CONFIG;
    hmc_cfg.active_slot_mask = magnetic_slots;
    if (hmc100x_init(&hmc_cfg) != 0) {
        ESP_LOGE(TAG, "hmc100x init failed");
        return;
    }
#if MAG_SET_RESET_ENABLE
    if (magnetic_slots != 0U && hmc100x_set_reset_sequence() != 0) {
        ESP_LOGE(TAG, "initial HMC100x set/reset failed");
        return;
    }
#else
    ESP_LOGI(TAG, "HMC100x set/reset pulses disabled for offset auto-zero test");
#endif
    if ((magnetic_slots & (1UL << BOARD_SLOT_1)) != 0U) {
        init_offset_dac_for_slot(BOARD_SLOT_1);
    }
    if ((magnetic_slots & (1UL << BOARD_SLOT_2)) != 0U) {
        init_offset_dac_for_slot(BOARD_SLOT_2);
    }

    sr_set_pin(SR_ADC_MCLK_EN, true);
    sr_set_pin(SR_ADC_RESET, true);
    sr_set_pin(SR_ADC_START, true);
    sr_set_pin(SR_ADC_CONVST_SAR, true);
    vTaskDelay(pdMS_TO_TICKS(100));

    ad7779_t adc = {0};
    ad7779_config_t cfg = AD7779_DEFAULT_CONFIG;
    cfg.odr_hz = MAG_ODR_HZ;
    cfg.reference = AD7779_REF_INTERNAL;
    cfg.channels_enabled = 0xFFU;
    cfg.verify_writes = true;
    cfg.use_crc = true;

    ad7779_status_t st = ad7779_init(&adc, ad7779_hal_default_instance(), &cfg);
    if (st != AD7779_OK) {
        ESP_LOGE(TAG, "ad7779_init failed: %d", st);
        return;
    }

    ESP_LOGI(TAG,
             "AD7779 OK. Magnetic expansion bring-up: +3V3A enabled, "
             "+9VA=%s, -5VA=%s, "
             "HMC100x set/reset disabled, ref = internal %.1f V, "
             "ODR = %lu Hz, gain = x%.0f. Binary serial output = %lu Hz at %lu baud.",
             (MAG_ENABLE_BRIDGE_9V != 0) ? "on" : "off",
             (MAG_ENABLE_NEG5V != 0) ? "on" : "off",
             ADC_REF_V, (unsigned long)MAG_ODR_HZ, MAG_ADC_GAIN,
             (unsigned long)SERIAL_STUDIO_RATE_HZ, (unsigned long)MAG_UART_BAUD);

    ad7779_set_sample_callback(&adc, adc_sample_cb, NULL);
    st = ad7779_start_streaming(&adc);
    if (st != AD7779_OK) {
        ESP_LOGE(TAG, "ad7779_start_streaming failed: %d", st);
        return;
    }

#if MAG_OFFSET_AUTOZERO_ENABLE
    if (s_autozero_ctx != NULL) {
        BaseType_t ok = xTaskCreate(offset_autozero_task,
                                    "offset_zero",
                                    3072,
                                    s_autozero_ctx,
                                    4,
                                    NULL);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "failed to start offset auto-zero task");
            return;
        }
        ESP_LOGI(TAG,
                 "offset auto-zero enabled: window=%u ms step=%u/%u LSB deadband=%u counts",
                 MAG_OFFSET_AUTOZERO_WINDOW_MS,
                 MAG_OFFSET_AUTOZERO_STEP_LSB,
                 MAG_OFFSET_AUTOZERO_CLIPPED_STEP_LSB,
                 MAG_OFFSET_AUTOZERO_DEADBAND_CODE);
    } else {
        ESP_LOGW(TAG, "offset auto-zero not started: no offset DAC ready");
    }
#endif

#if MAG_SET_RESET_ENABLE
    if (hmc100x_start_periodic_task() != 0) {
        ESP_LOGE(TAG, "failed to start HMC100x set/reset task");
        return;
    }
#endif

    uint32_t last_dropped = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        uint32_t dropped = ad7779_frames_dropped(&adc);
        if (dropped != last_dropped) {
            ESP_LOGW(TAG, "frames=%lu dropped=%lu (+%lu)",
                     (unsigned long)ad7779_frame_count(&adc),
                     (unsigned long)dropped,
                     (unsigned long)(dropped - last_dropped));
            last_dropped = dropped;
        }
    }
}
