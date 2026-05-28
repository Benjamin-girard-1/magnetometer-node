#include "serial_control.h"

#include "adc_stream.h"
#include "ad7779.h"
#include "app_config.h"
#include "board_control.h"
#include "hmc100x.h"
#include "imu_debug.h"
#include "sd_card_debug.h"
#include "shift_register.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "serial";

#define MAG_PACKET_PAYLOAD_LEN    (4U + (AD7779_NUM_CHANNELS * 4U) + 1U)
#define MAG_PACKET_TOTAL_LEN      (2U + 1U + MAG_PACKET_PAYLOAD_LEN + 1U)

static volatile uart_output_t s_uart_output = UART_OUTPUT_TEXT;

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

esp_err_t serial_control_init(void)
{
    esp_err_t err = uart_driver_install(UART_NUM_0,
                                        MAG_UART_RX_BUF_BYTES,
                                        MAG_UART_TX_BUF_BYTES,
                                        0,
                                        NULL,
                                        0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    return uart_set_baudrate(UART_NUM_0, MAG_UART_BAUD);
}

void serial_select_output(uart_output_t output)
{
    s_uart_output = output;
    ESP_LOGI(TAG, "UART output = %s",
             output == UART_OUTPUT_ADC_BINARY ? "ADC binary" : "text/control");
}

static bool serial_adc_output_enabled(void)
{
    return s_uart_output == UART_OUTPUT_ADC_BINARY;
}

void serial_write_adc_packet(const int32_t *samples,
                             uint8_t status,
                             uint32_t frame_idx)
{
    if (!serial_adc_output_enabled()) {
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

static bool parse_on_off_arg(const char *arg, bool *enable)
{
    if (strcmp(arg, "1") == 0 || strcmp(arg, "ON") == 0 || strcmp(arg, "on") == 0) {
        *enable = true;
        return true;
    }
    if (strcmp(arg, "0") == 0 || strcmp(arg, "OFF") == 0 || strcmp(arg, "off") == 0) {
        *enable = false;
        return true;
    }
    return false;
}

static void uart_handle_command(const char *line)
{
    if (strcmp(line, "OUT ADC") == 0 ||
        strcmp(line, "MODE ADC") == 0 ||
        strcmp(line, "ADC") == 0) {
        if (adc_stream_start() != ESP_OK) {
            serial_select_output(UART_OUTPUT_TEXT);
        }
        return;
    }

    if (strcmp(line, "OUT TEXT") == 0 ||
        strcmp(line, "MODE TEXT") == 0 ||
        strcmp(line, "MODE CTRL") == 0 ||
        strcmp(line, "CTRL") == 0 ||
        strcmp(line, "TEXT") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        return;
    }

    if (strcmp(line, "IMU LSM") == 0 ||
        strcmp(line, "IMU LSM ON") == 0 ||
        strcmp(line, "LSM6DSV") == 0 ||
        strcmp(line, "LSM6DSV ON") == 0) {
        (void)lsm6dsv_debug_sample();
        return;
    }

    if (strcmp(line, "IMU SCL") == 0 ||
        strcmp(line, "IMU SCL ON") == 0 ||
        strcmp(line, "SCL3300") == 0 ||
        strcmp(line, "SCL3300 ON") == 0) {
        (void)scl3300_debug_sample();
        return;
    }

    if (strcmp(line, "SD") == 0 || strcmp(line, "SDTEST") == 0) {
        esp_err_t err = run_sd_card_test();
        ESP_LOGI(TAG, "SD test %s", err == ESP_OK ? "complete" : "failed");
        return;
    }

    if (strcmp(line, "SD WRITE_TEST") == 0 || strcmp(line, "SD WRITE") == 0) {
        esp_err_t err = run_sd_card_ops(true, false);
        ESP_LOGI(TAG, "SD write test %s", err == ESP_OK ? "complete" : "failed");
        return;
    }

    if (strcmp(line, "SD READ_TEST") == 0 || strcmp(line, "SD READ") == 0) {
        esp_err_t err = run_sd_card_ops(false, true);
        ESP_LOGI(TAG, "SD read test %s", err == ESP_OK ? "complete" : "failed");
        return;
    }

    if (strcmp(line, "SD ESP") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        sd_mux_select_esp32();
        ESP_LOGI(TAG, "SD mux set to ESP32");
        return;
    }

    if (strcmp(line, "SD ESP0") == 0 || strcmp(line, "SD SEL 0") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        sd_mux_select_esp32_level(false);
        ESP_LOGI(TAG, "SD mux set to ESP32 candidate SEL=0");
        return;
    }

    if (strcmp(line, "SD ESP1") == 0 || strcmp(line, "SD SEL 1") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        sd_mux_select_esp32_level(true);
        ESP_LOGI(TAG, "SD mux set to ESP32 candidate SEL=1");
        return;
    }

    if (strcmp(line, "MUX EN 0") == 0 || strcmp(line, "SD EN 0") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        sr_set_pin(SR_EN_SD_MUX, false);
        sd_log_shift_register_state("MUX EN 0");
        return;
    }

    if (strcmp(line, "MUX EN 1") == 0 || strcmp(line, "SD EN 1") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        sr_set_pin(SR_EN_SD_MUX, true);
        sd_log_shift_register_state("MUX EN 1");
        return;
    }

    if (strcmp(line, "MUX SEL 0") == 0 || strcmp(line, "SD SEL RAW 0") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        sr_set_pin(SR_SD_MUX_SEL, false);
        sd_log_shift_register_state("MUX SEL 0");
        return;
    }

    if (strcmp(line, "MUX SEL 1") == 0 || strcmp(line, "SD SEL RAW 1") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        sr_set_pin(SR_SD_MUX_SEL, true);
        sd_log_shift_register_state("MUX SEL 1");
        return;
    }

    if (strcmp(line, "MUX STATE") == 0 || strcmp(line, "SD MUX STATE") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        sd_log_shift_register_state("MUX STATE");
        return;
    }

    if (strcmp(line, "SR STATE") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        board_log_sr_state("SR STATE");
        return;
    }

    int sr_pin = -1;
    int sr_level = -1;
    if (sscanf(line, "SR PIN %d %d", &sr_pin, &sr_level) == 2) {
        serial_select_output(UART_OUTPUT_TEXT);
        if (sr_pin < 0 || sr_pin > 15 || (sr_level != 0 && sr_level != 1)) {
            ESP_LOGW(TAG, "bad SR PIN command: %s", line);
            return;
        }
        board_set_sr_pin(sr_pin, sr_level != 0);
        return;
    }

    if (strcmp(line, "SD PROBE0") == 0 || strcmp(line, "SD PROBE 0") == 0) {
        esp_err_t err = run_sd_card_probe(false);
        ESP_LOGI(TAG, "SD probe0 %s", err == ESP_OK ? "complete" : "failed");
        return;
    }

    if (strcmp(line, "SD PROBE1") == 0 || strcmp(line, "SD PROBE 1") == 0) {
        esp_err_t err = run_sd_card_probe(true);
        ESP_LOGI(TAG, "SD probe1 %s", err == ESP_OK ? "complete" : "failed");
        return;
    }

    if (strcmp(line, "SD PROBEALL") == 0 || strcmp(line, "SD PROBE ALL") == 0) {
        esp_err_t err = run_sd_card_probe_all();
        ESP_LOGI(TAG, "SD probe-all %s", err == ESP_OK ? "complete" : "failed");
        return;
    }

    if (strcmp(line, "SD SPIPROBE") == 0 || strcmp(line, "SD SPI PROBE") == 0) {
        esp_err_t err = run_sd_card_spi_probe_all();
        ESP_LOGI(TAG, "SD SPI probe %s", err == ESP_OK ? "complete" : "failed");
        return;
    }

    if (strcmp(line, "SD LINEPROBE") == 0 || strcmp(line, "SD LINE PROBE") == 0) {
        run_sd_line_probe_all();
        ESP_LOGI(TAG, "SD line probe complete");
        return;
    }

    if (strcmp(line, "SD IDLE") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        sd_mux_idle();
        return;
    }

    if (strcmp(line, "SD HIZ") == 0 || strcmp(line, "SD HIGHZ") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        sd_safe_idle();
        return;
    }

    if (strcmp(line, "SD USB") == 0 || strcmp(line, "SD USB2641") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        sd_mux_select_usb2641();
        ESP_LOGI(TAG, "SD mux set to USB2641");
        return;
    }

    if (strcmp(line, "USB2641 RESET0") == 0 || strcmp(line, "USB RESET0") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        usb2641_set_reset(false);
        return;
    }

    if (strcmp(line, "USB2641 RESET1") == 0 || strcmp(line, "USB RESET1") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        usb2641_set_reset(true);
        return;
    }

    if (strcmp(line, "USB2641 PULSE") == 0 || strcmp(line, "USB PULSE") == 0) {
        serial_select_output(UART_OUTPUT_TEXT);
        usb2641_reset_pulse();
        return;
    }

    if (strncmp(line, "KEEPALIVE ", 10) == 0) {
        bool enable = false;
        if (!parse_on_off_arg(&line[10], &enable)) {
            ESP_LOGW(TAG, "bad KEEPALIVE command: %s", line);
            return;
        }
        serial_select_output(UART_OUTPUT_TEXT);
        board_set_keepalive(enable);
        sd_log_shift_register_state("KEEPALIVE");
        return;
    }

    if (strncmp(line, "9V ", 3) == 0) {
        bool enable = false;
        if (!parse_on_off_arg(&line[3], &enable)) {
            ESP_LOGW(TAG, "bad 9V command: %s", line);
            return;
        }
        board_set_9v(enable);
        return;
    }

    if (strncmp(line, "NEG5V ", 6) == 0 || strncmp(line, "-5V ", 4) == 0) {
        const char *arg = (line[0] == '-') ? &line[4] : &line[6];
        bool enable = false;
        if (!parse_on_off_arg(arg, &enable)) {
            ESP_LOGW(TAG, "bad NEG5V command: %s", line);
            return;
        }
        board_set_neg5v(enable);
        return;
    }

    if (strcmp(line, "SR") == 0 || strcmp(line, "SETRESET") == 0) {
        if (hmc100x_set_reset_sequence() != 0) {
            ESP_LOGW(TAG, "SETRESET command failed");
        }
        return;
    }

    if (strcmp(line, "SET") == 0) {
        if (hmc100x_set_only_sequence() != 0) {
            ESP_LOGW(TAG, "SET command failed");
        }
        return;
    }

    if (strcmp(line, "RESET") == 0) {
        if (hmc100x_reset_only_sequence() != 0) {
            ESP_LOGW(TAG, "RESET command failed");
        }
        return;
    }

    ESP_LOGW(TAG, "unknown command: %s", line);
}

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
                if (len > 0U) {
                    uart_handle_command(line);
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

esp_err_t serial_command_task_start(void)
{
    if (xTaskCreate(uart_command_task,
                    "uart_cmd",
                    MAG_UART_CMD_STACK_BYTES,
                    NULL,
                    4,
                    NULL) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
