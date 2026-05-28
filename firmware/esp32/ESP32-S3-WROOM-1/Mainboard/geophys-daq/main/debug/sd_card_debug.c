#include "sd_card_debug.h"

#include "app_config.h"
#include "serial_control.h"
#include "shift_register.h"

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "sd_debug";

static void sd_log_line_levels(const char *label);

void sd_log_shift_register_state(const char *context)
{
    ESP_LOGI(TAG, "%s: shift-register state=0x%04x EN_SD_MUX=%u SD_MUX_SEL=%u USB2641_NRESET=%u",
             context,
             (unsigned)sr_get_state(),
             sr_get_pin(SR_EN_SD_MUX) ? 1U : 0U,
             sr_get_pin(SR_SD_MUX_SEL) ? 1U : 0U,
             sr_get_pin(SR_USB2641_NRESET) ? 1U : 0U);
}

void usb2641_set_reset(bool released)
{
    sr_set_pin(SR_USB2641_NRESET, released);
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "USB2641 reset %s", released ? "released" : "asserted");
    sd_log_shift_register_state("USB2641 reset");
}

void usb2641_reset_pulse(void)
{
    usb2641_set_reset(false);
    vTaskDelay(pdMS_TO_TICKS(100));
    usb2641_set_reset(true);
}

void sd_mux_select_esp32_level(bool sel_level)
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

void sd_mux_select_esp32(void)
{
    sd_mux_select_esp32_level(SD_MUX_SEL_ESP32_LEVEL);
}

void sd_mux_idle(void)
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

void sd_safe_idle(void)
{
    sd_lines_high_z();
    sd_mux_idle();
}

void sd_mux_select_usb2641(void)
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

esp_err_t run_sd_card_ops(bool write_test, bool read_test)
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

esp_err_t run_sd_card_test(void)
{
    return run_sd_card_ops(true, true);
}

esp_err_t run_sd_card_probe(bool sel_level)
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

esp_err_t run_sd_card_probe_all(void)
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

esp_err_t run_sd_card_spi_probe_all(void)
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

void run_sd_line_probe_all(void)
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
