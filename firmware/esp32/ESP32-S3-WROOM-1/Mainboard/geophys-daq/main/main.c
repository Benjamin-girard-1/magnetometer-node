/**
 * @file main.c
 * @brief Magnetic ADC stream and SD-card handoff bring-up.
 */

#include "ad7779.h"
#include "ad7779_hal.h"
#include "board_detect.h"
#include "hmc100x.h"
#include "shift_register.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

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
#define MAG_UART_CMD_STACK_BYTES  8192
#define MAG_ENABLE_BRIDGE_9V      0
#define MAG_ENABLE_NEG5V          0
#define MAG_SET_RESET_ENABLE      0

/* Set this to 1 when you want the ADC binary stream as the default boot mode. */
#define MAG_ENABLE_ADC_STREAM_ON_BOOT 0

#define MAG_PACKET_SYNC0          0xA5U
#define MAG_PACKET_SYNC1          0x5AU
#define MAG_PACKET_PAYLOAD_LEN    (4U + (AD7779_NUM_CHANNELS * 4U) + 1U)
#define MAG_PACKET_TOTAL_LEN      (2U + 1U + MAG_PACKET_PAYLOAD_LEN + 1U)

#define SD_MOUNT_POINT            "/sdcard"
#define SD_TEST_FILE              SD_MOUNT_POINT "/esp32_sd_test.txt"

#define SD_PIN_CLK                GPIO_NUM_5
#define SD_PIN_CMD                GPIO_NUM_16
#define SD_PIN_D0                 GPIO_NUM_6
#define SD_PIN_D1                 GPIO_NUM_4
#define SD_PIN_D2                 GPIO_NUM_7
#define SD_PIN_D3                 GPIO_NUM_15
#define SD_INIT_BUS_WIDTH         1
#define SD_INIT_FREQ_KHZ          SDMMC_FREQ_PROBING

/*
 * U16 TS3A27518EPWR:
 *   COMx -> microSD card
 *   NOx  -> ESP32 SDMMC bus
 *   NCx  -> USB2641 SD bus
 *   ~EN  <- EN_SD_MUX
 */
#define SD_MUX_ENABLE_LEVEL       false
#define SD_MUX_DISABLE_LEVEL      true
#define SD_MUX_SEL_ESP32_LEVEL    true
#define SD_MUX_SEL_USB2641_LEVEL  false

typedef enum {
    UART_OUTPUT_TEXT = 0,
    UART_OUTPUT_ADC_BINARY,
} uart_output_t;

static volatile uart_output_t s_uart_output = UART_OUTPUT_TEXT;
static ad7779_t s_adc;
static bool s_adc_ready;
static bool s_adc_streaming;

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

static esp_err_t serial_init(void)
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

static void serial_select_output(uart_output_t output)
{
    s_uart_output = output;
    ESP_LOGI(TAG, "UART output = %s",
             output == UART_OUTPUT_ADC_BINARY ? "ADC binary" : "text/control");
}

static bool serial_adc_output_enabled(void)
{
    return s_uart_output == UART_OUTPUT_ADC_BINARY;
}

static void serial_write_adc_packet(const int32_t *samples,
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

static void sd_log_shift_register_state(const char *context)
{
    ESP_LOGI(TAG, "%s: shift-register state=0x%04x EN_SD_MUX=%u SD_MUX_SEL=%u USB2641_NRESET=%u",
             context,
             (unsigned)sr_get_state(),
             sr_get_pin(SR_EN_SD_MUX) ? 1U : 0U,
             sr_get_pin(SR_SD_MUX_SEL) ? 1U : 0U,
             sr_get_pin(SR_USB2641_NRESET) ? 1U : 0U);
}

static void sd_log_line_levels(const char *label);

static void usb2641_set_reset(bool released)
{
    sr_set_pin(SR_USB2641_NRESET, released);
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "USB2641 reset %s", released ? "released" : "asserted");
    sd_log_shift_register_state("USB2641 reset");
}

static void usb2641_reset_pulse(void)
{
    usb2641_set_reset(false);
    vTaskDelay(pdMS_TO_TICKS(100));
    usb2641_set_reset(true);
}

static void sd_mux_select_esp32_level(bool sel_level)
{
    sr_set_pin(SR_USB2641_NRESET, false);
    sr_set_pin(SR_EN_SD_MUX, SD_MUX_DISABLE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(5));
    sr_set_pin(SR_SD_MUX_SEL, sel_level);
    sr_set_pin(SR_EN_SD_MUX, SD_MUX_ENABLE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "SD mux selected: ESP32 candidate SEL=%u", sel_level ? 1U : 0U);
    sd_log_shift_register_state("SD mux ESP32");
}

static void sd_mux_select_esp32_combo(bool en_level, bool sel_level)
{
    sr_set_pin(SR_USB2641_NRESET, false);
    sr_set_pin(SR_EN_SD_MUX, SD_MUX_DISABLE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(5));
    sr_set_pin(SR_SD_MUX_SEL, sel_level);
    sr_set_pin(SR_EN_SD_MUX, en_level);
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "SD mux probe candidate: EN=%u SEL=%u",
             en_level ? 1U : 0U,
             sel_level ? 1U : 0U);
    sd_log_shift_register_state("SD mux probe");
}

static void sd_mux_select_esp32(void)
{
    sd_mux_select_esp32_level(SD_MUX_SEL_ESP32_LEVEL);
}

static void sd_mux_idle(void)
{
    sr_set_pin(SR_USB2641_NRESET, false);
    sr_set_pin(SR_EN_SD_MUX, SD_MUX_DISABLE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "SD mux idle: USB2641 reset asserted, mux disabled");
    sd_log_shift_register_state("SD mux idle");
}

static void sd_lines_high_z(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << SD_PIN_CLK) |
                        (1ULL << SD_PIN_CMD) |
                        (1ULL << SD_PIN_D0) |
                        (1ULL << SD_PIN_D1) |
                        (1ULL << SD_PIN_D2) |
                        (1ULL << SD_PIN_D3),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_cfg);
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "SD ESP32 pins set to high-Z input/no-pull");
    sd_log_line_levels("SD high-Z levels");
}

static void sd_safe_idle(void)
{
    sd_lines_high_z();
    sd_mux_idle();
}

static void sd_mux_select_usb2641(void)
{
    sr_set_pin(SR_EN_SD_MUX, SD_MUX_DISABLE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(5));
    sr_set_pin(SR_SD_MUX_SEL, SD_MUX_SEL_USB2641_LEVEL);
    sr_set_pin(SR_EN_SD_MUX, SD_MUX_ENABLE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(50));
    sr_set_pin(SR_USB2641_NRESET, true);
    ESP_LOGI(TAG, "SD mux selected: USB2641");
    sd_log_shift_register_state("SD mux USB2641");
}

static esp_err_t sd_write_test_file(void)
{
    FILE *f = fopen(SD_TEST_FILE, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "failed to open %s for write", SD_TEST_FILE);
        return ESP_FAIL;
    }

    fprintf(f, "ESP32 SD-card test\r\n");
    fprintf(f, "If you can read this over USB-C, the USB2641 handoff works.\r\n");
    fprintf(f, "Build timestamp: %s %s\r\n", __DATE__, __TIME__);

    if (fclose(f) != 0) {
        ESP_LOGE(TAG, "failed to close %s", SD_TEST_FILE);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "wrote %s", SD_TEST_FILE);
    return ESP_OK;
}

static esp_err_t sd_read_test_file_back(void)
{
    FILE *f = fopen(SD_TEST_FILE, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "failed to open %s for readback", SD_TEST_FILE);
        return ESP_FAIL;
    }

    char line[96];
    while (fgets(line, sizeof(line), f) != NULL) {
        line[strcspn(line, "\r\n")] = '\0';
        ESP_LOGI(TAG, "readback: %s", line);
    }

    fclose(f);
    return ESP_OK;
}

static esp_err_t sd_mount(sdmmc_card_t **out_card)
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SD_INIT_FREQ_KHZ;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = SD_INIT_BUS_WIDTH;
    slot_config.clk = SD_PIN_CLK;
    slot_config.cmd = SD_PIN_CMD;
    slot_config.d0 = SD_PIN_D0;
#if SD_INIT_BUS_WIDTH > 1
    slot_config.d1 = SD_PIN_D1;
    slot_config.d2 = SD_PIN_D2;
    slot_config.d3 = SD_PIN_D3;
#else
    slot_config.d1 = GPIO_NUM_NC;
    slot_config.d2 = GPIO_NUM_NC;
    slot_config.d3 = GPIO_NUM_NC;
#endif
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    ESP_LOGI(TAG, "SD mount attempt: width=%d freq=%d kHz pins CLK=%d CMD=%d D0=%d D1=%d D2=%d D3=%d",
             SD_INIT_BUS_WIDTH,
             SD_INIT_FREQ_KHZ,
             SD_PIN_CLK,
             SD_PIN_CMD,
             SD_PIN_D0,
             SD_PIN_D1,
             SD_PIN_D2,
             SD_PIN_D3);

    esp_err_t err = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT,
                                            &host,
                                            &slot_config,
                                            &mount_config,
                                            out_card);
    if (err != ESP_OK) {
        sd_log_shift_register_state("SD mount failed");
    }
    return err;
}

static esp_err_t run_sd_card_ops(bool write_test, bool read_test)
{
    serial_select_output(UART_OUTPUT_TEXT);

    sd_mux_select_esp32();

    sdmmc_card_t *card = NULL;
    esp_err_t err = sd_mount(&card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(err));
        sd_mux_idle();
        return err;
    }

    sdmmc_card_print_info(stdout, card);

    if (write_test) {
        err = sd_write_test_file();
    }
    if (err == ESP_OK && read_test) {
        err = sd_read_test_file_back();
    }

    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
    ESP_LOGI(TAG, "SD card unmounted");

    sd_mux_idle();
    return err;
}

static esp_err_t run_sd_card_test(void)
{
    return run_sd_card_ops(true, true);
}

static esp_err_t run_sd_card_probe(bool sel_level)
{
    serial_select_output(UART_OUTPUT_TEXT);
    sd_mux_select_esp32_level(sel_level);

    sdmmc_card_t *card = NULL;
    esp_err_t err = sd_mount(&card);
    if (err == ESP_OK) {
        sdmmc_card_print_info(stdout, card);
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
        ESP_LOGI(TAG, "SD probe SEL=%u succeeded", sel_level ? 1U : 0U);
    } else {
        ESP_LOGE(TAG, "SD probe SEL=%u failed: %s", sel_level ? 1U : 0U, esp_err_to_name(err));
    }

    sd_mux_idle();
    return err;
}

static esp_err_t run_sd_card_probe_all(void)
{
    serial_select_output(UART_OUTPUT_TEXT);

    esp_err_t first_err = ESP_FAIL;
    for (int en = 0; en <= 1; ++en) {
        for (int sel = 0; sel <= 1; ++sel) {
            ESP_LOGI(TAG, "SD probe all: trying EN=%d SEL=%d", en, sel);
            sd_mux_select_esp32_combo(en != 0, sel != 0);

            sdmmc_card_t *card = NULL;
            esp_err_t err = sd_mount(&card);
            if (err == ESP_OK) {
                sdmmc_card_print_info(stdout, card);
                esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
                ESP_LOGI(TAG, "SD probe all succeeded with EN=%d SEL=%d", en, sel);
                sd_mux_idle();
                return ESP_OK;
            }

            if (first_err == ESP_FAIL) {
                first_err = err;
            }
            ESP_LOGE(TAG, "SD probe all failed with EN=%d SEL=%d: %s",
                     en, sel, esp_err_to_name(err));
        }
    }

    sd_mux_idle();
    return first_err;
}

static esp_err_t run_sd_card_spi_probe_all(void)
{
    serial_select_output(UART_OUTPUT_TEXT);

    esp_err_t first_err = ESP_FAIL;
    for (int en = 0; en <= 1; ++en) {
        for (int sel = 0; sel <= 1; ++sel) {
            ESP_LOGI(TAG, "SD SPI probe: trying EN=%d SEL=%d", en, sel);
            sd_mux_select_esp32_combo(en != 0, sel != 0);

            spi_bus_config_t bus_cfg = {
                .mosi_io_num = SD_PIN_CMD,
                .miso_io_num = SD_PIN_D0,
                .sclk_io_num = SD_PIN_CLK,
                .quadwp_io_num = GPIO_NUM_NC,
                .quadhd_io_num = GPIO_NUM_NC,
                .max_transfer_sz = 4096,
            };

            esp_err_t err = spi_bus_initialize(SPI3_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "SD SPI probe bus init failed: %s", esp_err_to_name(err));
                if (first_err == ESP_FAIL) {
                    first_err = err;
                }
                continue;
            }

            sdmmc_host_t host = SDSPI_HOST_DEFAULT();
            host.slot = SPI3_HOST;
            host.max_freq_khz = SD_INIT_FREQ_KHZ;

            sdspi_device_config_t dev_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
            dev_cfg.host_id = SPI3_HOST;
            dev_cfg.gpio_cs = SD_PIN_D3;

            esp_vfs_fat_sdmmc_mount_config_t mount_config = {
                .format_if_mount_failed = false,
                .max_files = 4,
                .allocation_unit_size = 16 * 1024,
            };

            sdmmc_card_t *card = NULL;
            ESP_LOGI(TAG, "SD SPI mount attempt: freq=%d kHz CLK=%d MOSI/CMD=%d MISO/D0=%d CS/D3=%d",
                     SD_INIT_FREQ_KHZ,
                     SD_PIN_CLK,
                     SD_PIN_CMD,
                     SD_PIN_D0,
                     SD_PIN_D3);
            err = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT,
                                           &host,
                                           &dev_cfg,
                                           &mount_config,
                                           &card);
            if (err == ESP_OK) {
                sdmmc_card_print_info(stdout, card);
                esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
                spi_bus_free(SPI3_HOST);
                ESP_LOGI(TAG, "SD SPI probe succeeded with EN=%d SEL=%d", en, sel);
                sd_mux_idle();
                return ESP_OK;
            }

            ESP_LOGE(TAG, "SD SPI probe failed with EN=%d SEL=%d: %s",
                     en, sel, esp_err_to_name(err));
            sd_log_shift_register_state("SD SPI probe failed");
            if (first_err == ESP_FAIL) {
                first_err = err;
            }
            spi_bus_free(SPI3_HOST);
        }
    }

    sd_mux_idle();
    return first_err;
}

static void sd_config_line_probe_inputs(gpio_pullup_t pullup)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << SD_PIN_CLK) |
                        (1ULL << SD_PIN_CMD) |
                        (1ULL << SD_PIN_D0) |
                        (1ULL << SD_PIN_D1) |
                        (1ULL << SD_PIN_D2) |
                        (1ULL << SD_PIN_D3),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = pullup,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_cfg);
    vTaskDelay(pdMS_TO_TICKS(5));
}

static void sd_log_line_levels(const char *label)
{
    ESP_LOGI(TAG, "%s: CLK=%d CMD=%d D0=%d D1=%d D2=%d D3=%d",
             label,
             gpio_get_level(SD_PIN_CLK),
             gpio_get_level(SD_PIN_CMD),
             gpio_get_level(SD_PIN_D0),
             gpio_get_level(SD_PIN_D1),
             gpio_get_level(SD_PIN_D2),
             gpio_get_level(SD_PIN_D3));
}

static void run_sd_line_probe_all(void)
{
    serial_select_output(UART_OUTPUT_TEXT);

    for (int en = 0; en <= 1; ++en) {
        for (int sel = 0; sel <= 1; ++sel) {
            ESP_LOGI(TAG, "SD line probe: EN=%d SEL=%d", en, sel);
            sd_mux_select_esp32_combo(en != 0, sel != 0);

            sd_config_line_probe_inputs(GPIO_PULLUP_DISABLE);
            sd_log_line_levels("SD lines no internal pullup");

            sd_config_line_probe_inputs(GPIO_PULLUP_ENABLE);
            sd_log_line_levels("SD lines with internal pullup");
        }
    }

    sd_mux_idle();
}

static void adc_sample_cb(void *ctx, const int32_t *samples,
                          uint8_t status, uint32_t frame_idx)
{
    (void)ctx;

    if ((frame_idx % SERIAL_STUDIO_DECIMATION) != 0U) {
        return;
    }

    serial_write_adc_packet(samples, status, frame_idx);
}

static esp_err_t adc_power_on(void)
{
    sr_set_pin(SR_ADC_MCLK_EN, true);
    sr_set_pin(SR_ADC_RESET, true);
    sr_set_pin(SR_ADC_START, true);
    sr_set_pin(SR_ADC_CONVST_SAR, true);
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

static void adc_power_off(void)
{
    sr_set_pin(SR_ADC_START, false);
    sr_set_pin(SR_ADC_CONVST_SAR, false);
    sr_set_pin(SR_ADC_RESET, false);
    sr_set_pin(SR_ADC_MCLK_EN, false);
}

static esp_err_t adc_stream_start(void)
{
    if (s_adc_streaming) {
        ESP_RETURN_ON_ERROR(adc_power_on(), TAG, "ADC power re-assert failed");
        serial_select_output(UART_OUTPUT_ADC_BINARY);
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(adc_power_on(), TAG, "ADC power-on failed");

    if (!s_adc_ready) {
        ad7779_config_t cfg = AD7779_DEFAULT_CONFIG;
        cfg.odr_hz = MAG_ODR_HZ;
        cfg.reference = AD7779_REF_INTERNAL;
        cfg.channels_enabled = 0xFFU;
        cfg.verify_writes = true;
        cfg.use_crc = true;

        ad7779_status_t st = ad7779_init(&s_adc, ad7779_hal_default_instance(), &cfg);
        if (st != AD7779_OK) {
            ESP_LOGE(TAG, "ad7779_init failed: %d", st);
            adc_power_off();
            return ESP_FAIL;
        }

        ad7779_set_sample_callback(&s_adc, adc_sample_cb, NULL);
        s_adc_ready = true;
    }

    ad7779_status_t st = ad7779_start_streaming(&s_adc);
    if (st != AD7779_OK) {
        ESP_LOGE(TAG, "ad7779_start_streaming failed: %d", st);
        return ESP_FAIL;
    }

    s_adc_streaming = true;
    serial_select_output(UART_OUTPUT_ADC_BINARY);

    ESP_LOGI(TAG,
             "AD7779 OK. Magnetic expansion bring-up: +9V=%s, -5V=%s, "
             "ref = internal %.1f V, ODR = %lu Hz, gain = x%.0f. "
             "Binary serial output = %lu Hz at %lu baud.",
             (MAG_ENABLE_BRIDGE_9V != 0) ? "on" : "off",
             (MAG_ENABLE_NEG5V != 0) ? "on" : "off",
             ADC_REF_V, (unsigned long)MAG_ODR_HZ, MAG_ADC_GAIN,
             (unsigned long)SERIAL_STUDIO_RATE_HZ, (unsigned long)MAG_UART_BAUD);
    return ESP_OK;
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
        ESP_LOGI(TAG, "shift-register state=0x%04x", (unsigned)sr_get_state());
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
        sr_set_pin((sr_pin_t)sr_pin, sr_level != 0);
        ESP_LOGI(TAG, "SR pin %d set to %d, state=0x%04x",
                 sr_pin,
                 sr_level,
                 (unsigned)sr_get_state());
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
        if (strcmp(&line[10], "1") == 0 ||
            strcmp(&line[10], "ON") == 0 ||
            strcmp(&line[10], "on") == 0) {
            enable = true;
        } else if (strcmp(&line[10], "0") != 0 &&
                   strcmp(&line[10], "OFF") != 0 &&
                   strcmp(&line[10], "off") != 0) {
            ESP_LOGW(TAG, "bad KEEPALIVE command: %s", line);
            return;
        }

        serial_select_output(UART_OUTPUT_TEXT);
        sr_set_pin(SR_EN_KEEPALIVE, enable);
        ESP_LOGI(TAG, "EN_KEEPALIVE %s", enable ? "on" : "off");
        sd_log_shift_register_state("KEEPALIVE");
        return;
    }

    if (strncmp(line, "9V ", 3) == 0) {
        bool enable = false;
        if (strcmp(&line[3], "1") == 0 ||
            strcmp(&line[3], "ON") == 0 ||
            strcmp(&line[3], "on") == 0) {
            enable = true;
        } else if (strcmp(&line[3], "0") != 0 &&
                   strcmp(&line[3], "OFF") != 0 &&
                   strcmp(&line[3], "off") != 0) {
            ESP_LOGW(TAG, "bad 9V command: %s", line);
            return;
        }

        sr_set_pin(SR_EN_BST_10V, enable);
        ESP_LOGI(TAG, "+9VA boost/LDO input enable %s", enable ? "on" : "off");
        return;
    }

    if (strncmp(line, "NEG5V ", 6) == 0 || strncmp(line, "-5V ", 4) == 0) {
        const char *arg = (line[0] == '-') ? &line[4] : &line[6];
        bool enable = false;
        if (strcmp(arg, "1") == 0 ||
            strcmp(arg, "ON") == 0 ||
            strcmp(arg, "on") == 0) {
            enable = true;
        } else if (strcmp(arg, "0") != 0 &&
                   strcmp(arg, "OFF") != 0 &&
                   strcmp(arg, "off") != 0) {
            ESP_LOGW(TAG, "bad NEG5V command: %s", line);
            return;
        }

        sr_set_pin(SR_EN_INV_NEG5V, enable);
        ESP_LOGI(TAG, "-5V inverter enable %s", enable ? "on" : "off");
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

static esp_err_t board_outputs_init(uint32_t magnetic_slots)
{
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
    adc_power_off();
    sd_mux_select_usb2641();
    vTaskDelay(pdMS_TO_TICKS(100));

    hmc100x_config_t hmc_cfg = HMC100X_DEFAULT_CONFIG;
    hmc_cfg.active_slot_mask = magnetic_slots;
    if (hmc100x_init(&hmc_cfg) != 0) {
        ESP_LOGE(TAG, "hmc100x init failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(150));

    ESP_ERROR_CHECK(serial_init());
    serial_select_output(UART_OUTPUT_TEXT);

    if (xTaskCreate(uart_command_task,
                    "uart_cmd",
                    MAG_UART_CMD_STACK_BYTES,
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

    if (board_outputs_init(magnetic_slots) != ESP_OK) {
        return;
    }

#if MAG_SET_RESET_ENABLE
    if (magnetic_slots != 0U && hmc100x_set_reset_sequence() != 0) {
        ESP_LOGE(TAG, "initial HMC100x set/reset failed");
        return;
    }
    if (hmc100x_start_periodic_task() != 0) {
        ESP_LOGE(TAG, "failed to start HMC100x set/reset task");
        return;
    }
#else
    ESP_LOGI(TAG, "HMC100x set/reset pulses disabled");
#endif

#if MAG_ENABLE_ADC_STREAM_ON_BOOT
    if (adc_stream_start() != ESP_OK) {
        return;
    }
#else
    ESP_LOGI(TAG, "ADC stream is off. Send 'ADC' for binary stream or 'SD' for SD test.");
#endif

    uint32_t last_dropped = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        if (s_adc_streaming) {
            uint32_t dropped = ad7779_frames_dropped(&s_adc);
            if (dropped != last_dropped) {
                ESP_LOGW(TAG, "frames=%lu dropped=%lu (+%lu)",
                         (unsigned long)ad7779_frame_count(&s_adc),
                         (unsigned long)dropped,
                         (unsigned long)(dropped - last_dropped));
                last_dropped = dropped;
            }
        }
    }
}
