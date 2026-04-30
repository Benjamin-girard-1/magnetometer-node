#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

#include "driver/uart.h"
#include "driver/gpio.h"

// =================== Config ===================
static const char *TAG = "DAN_F10N";

// UART pins (you did not change these)
#define GPS_UART                UART_NUM_1
#define GPS_UART_TX_GPIO        2   // ESP -> GNSS RXD
#define GPS_UART_RX_GPIO        1   // ESP <- GNSS TXD

// New GPIO mapping
#define GPS_RESET_N_GPIO        35  // active-low
#define GPS_EXTINT_GPIO         21  // ESP -> GNSS input
#define GPS_SAFEBOOT_N_GPIO     20  // active-low
#define GPS_TIMEPULSE_GPIO      19  // GNSS -> ESP output

#define GPS_UART_BUF_SIZE       2048

// TIMEPULSE pulse counter
static volatile uint32_t s_timepulse_count = 0;
static volatile int64_t  s_last_timepulse_us = 0;

// UART debug counter
static volatile uint32_t s_uart_rx_bytes = 0;

// Simple "how long have we been waiting" counter
static int64_t s_start_us = 0;

// =================== ISR ===================
static void IRAM_ATTR timepulse_isr_handler(void *arg)
{
    (void)arg;
    s_timepulse_count++;
    s_last_timepulse_us = esp_timer_get_time(); // microseconds since boot
}

// =================== GPIO ===================
static void gps_gpio_init(void)
{
    // RESET_N, EXTINT, SAFEBOOT_N as outputs
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << GPS_RESET_N_GPIO) |
                        (1ULL << GPS_EXTINT_GPIO)  |
                        (1ULL << GPS_SAFEBOOT_N_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));

    // Normal boot: SAFEBOOT_N must be HIGH (active-low)
    gpio_set_level(GPS_SAFEBOOT_N_GPIO, 1);

    // EXTINT default low
    gpio_set_level(GPS_EXTINT_GPIO, 0);

    // Keep module out of reset
    gpio_set_level(GPS_RESET_N_GPIO, 1);

    // TIMEPULSE as input with rising-edge interrupt
    gpio_config_t tp_cfg = {
        .pin_bit_mask = (1ULL << GPS_TIMEPULSE_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,   // enable if your board needs it
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&tp_cfg));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPS_TIMEPULSE_GPIO, timepulse_isr_handler, NULL));
}

static void gps_reset_pulse(void)
{
    // Active-low reset pulse
    gpio_set_level(GPS_RESET_N_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(GPS_RESET_N_GPIO, 1);

    // Let GNSS boot a bit
    vTaskDelay(pdMS_TO_TICKS(400));
}

// =================== UART ===================
static void gps_uart_init(int baud)
{
    const uart_config_t uart_cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(GPS_UART, GPS_UART_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART, GPS_UART_TX_GPIO, GPS_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_flush(GPS_UART));
}

// Sniff for "$....\n" patterns (very light NMEA detection)
static bool sniff_nmea_for_ms(int ms)
{
    uint8_t b;
    int dollar = 0, newline = 0;
    int64_t t_end = esp_timer_get_time() + (int64_t)ms * 1000;

    while (esp_timer_get_time() < t_end) {
        int n = uart_read_bytes(GPS_UART, &b, 1, pdMS_TO_TICKS(20));
        if (n > 0) {
            s_uart_rx_bytes++;
            if (b == '$') dollar++;
            if (b == '\n') newline++;
        }
    }
    return (dollar >= 1 && newline >= 1);
}

static int gps_try_baudrates(void)
{
    const int bauds[] = {9600, 38400, 115200};

    for (int i = 0; i < (int)(sizeof(bauds)/sizeof(bauds[0])); i++) {
        int br = bauds[i];
        ESP_ERROR_CHECK(uart_set_baudrate(GPS_UART, br));
        ESP_ERROR_CHECK(uart_flush(GPS_UART));
        s_uart_rx_bytes = 0;

        ESP_LOGI(TAG, "Trying baud %d...", br);
        if (sniff_nmea_for_ms(900)) {
            ESP_LOGI(TAG, "Detected NMEA-like traffic at %d baud.", br);
            return br;
        }
        ESP_LOGW(TAG, "No NMEA at %d baud (rx_bytes=%" PRIu32 ").", br, (uint32_t)s_uart_rx_bytes);
    }

    ESP_LOGE(TAG, "No NMEA detected at 9600/38400/115200. Likely wiring or GNSS UART config.");
    return 9600; // keep something set
}

// =================== UBX helpers ===================
static void gps_uart_write_bytes(const uint8_t *data, size_t len)
{
    // Best-effort write
    if (!data || len == 0) return;
    uart_write_bytes(GPS_UART, (const char *)data, (int)len);
    uart_wait_tx_done(GPS_UART, pdMS_TO_TICKS(100));
}

static bool gps_wait_for_ubx_ack(uint8_t cls, uint8_t id, int timeout_ms)
{
    // Wait for UBX-ACK-ACK (0x05 0x01) or UBX-ACK-NAK (0x05 0x00)
    // Payload of ACK messages includes the class+id being acknowledged.
    const int64_t t_end = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    uint8_t b;
    uint8_t state = 0;
    uint8_t buf[10];
    size_t idx = 0;

    while (esp_timer_get_time() < t_end) {
        int n = uart_read_bytes(GPS_UART, &b, 1, pdMS_TO_TICKS(20));
        if (n <= 0) continue;

        // Minimal UBX framing parser for ACK packets only
        switch (state) {
        case 0: state = (b == 0xB5) ? 1 : 0; break;
        case 1: state = (b == 0x62) ? 2 : 0; idx = 0; break;
        case 2:
            // class
            buf[idx++] = b;
            state = 3;
            break;
        case 3:
            // id
            buf[idx++] = b;
            state = 4;
            break;
        case 4:
            // length LSB
            buf[idx++] = b;
            state = 5;
            break;
        case 5:
            // length MSB
            buf[idx++] = b;
            // Expect length = 2 for ACK packets
            state = 6;
            break;
        case 6:
            // payload byte 0 (cls)
            buf[idx++] = b;
            state = 7;
            break;
        case 7:
            // payload byte 1 (id)
            buf[idx++] = b;
            // next 2 bytes are checksum; we ignore checksum for this quick test
            state = 8;
            break;
        case 8:
            // CK_A
            state = 9;
            break;
        case 9:
            // CK_B -> evaluate
            state = 0;

            if (buf[0] == 0x05 && (buf[1] == 0x01 || buf[1] == 0x00)) {
                bool is_ack = (buf[1] == 0x01);
                uint8_t acked_cls = buf[4];
                uint8_t acked_id  = buf[5];

                if (acked_cls == cls && acked_id == id) {
                    return is_ack;
                }
            }
            break;
        default:
            state = 0;
            break;
        }
    }
    return false;
}

static void gps_enable_gps_l5_ignore_health_ram(bool enable)
{
    // From DAN-F10N Integration Manual Table 3 and Table 4 (RAM layer)
    // Enable: override GPS L5 health status with GPS L1 health status
    static const uint8_t ubx_valset_enable[]  = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0x01,0x00,0x32,0x10,0x01,0xDF,0xF6};
    // Disable: revert to default monitoring
    static const uint8_t ubx_valset_disable[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0x01,0x00,0x32,0x10,0x00,0xDE,0xF5};

    const uint8_t *msg = enable ? ubx_valset_enable : ubx_valset_disable;

    // Flush any pending UART bytes so ACK parsing is cleaner
    uart_flush(GPS_UART);

    ESP_LOGI(TAG, "Sending UBX-CFG-VALSET to %s GPS L5 health override (RAM)...", enable ? "ENABLE" : "DISABLE");
    gps_uart_write_bytes(msg, enable ? sizeof(ubx_valset_enable) : sizeof(ubx_valset_disable));

    // Wait for ACK of class 0x06 id 0x8A (VALSET)
    bool ack = gps_wait_for_ubx_ack(0x06, 0x8A, 800);
    ESP_LOGI(TAG, "UBX-CFG-VALSET ACK: %s", ack ? "OK" : "NO/NAK");
}

// =================== Tasks ===================
static void gps_read_task(void *arg)
{
    (void)arg;

    uint8_t byte;
    char line[256];
    size_t idx = 0;

    while (1) {
        int n = uart_read_bytes(GPS_UART, &byte, 1, pdMS_TO_TICKS(100));
        if (n <= 0) {
            // Periodic status print
            static int64_t last_print = 0;
            int64_t now = esp_timer_get_time();
            if (now - last_print > 1000000) { // 1s
                last_print = now;

                int64_t elapsed_s = (s_start_us > 0) ? ((now - s_start_us) / 1000000) : 0;

                ESP_LOGI(TAG,
                    "WAIT=%" PRIi64 " s | TIMEPULSE count=%" PRIu32 ", last=%" PRIi64 " us, level=%d | UART rx_bytes=%" PRIu32,
                    elapsed_s,
                    (uint32_t)s_timepulse_count,
                    (int64_t)s_last_timepulse_us,
                    gpio_get_level(GPS_TIMEPULSE_GPIO),
                    (uint32_t)s_uart_rx_bytes
                );
            }
            continue;
        }

        s_uart_rx_bytes++;

        // Assemble NMEA-ish lines (\r\n terminated)
        if (byte == '\n') {
            line[idx] = '\0';
            if (idx > 0) {
                ESP_LOGI(TAG, "NMEA: %s", line);
            }
            idx = 0;
            continue;
        }

        if (byte == '\r') {
            continue;
        }

        if (idx < sizeof(line) - 1) {
            line[idx++] = (char)byte;
        } else {
            // overflow -> reset buffer
            idx = 0;
        }
    }
}

void app_main(void)
{
    s_start_us = esp_timer_get_time();

    gps_gpio_init();
    gps_uart_init(9600);

    // Optional but helpful on custom hardware
    gps_reset_pulse();

    ESP_LOGI(TAG, "GNSS UART on TX=%d RX=%d (start @ 9600). GPIO: RST=%d EXTINT=%d SAFEBOOT=%d TIMEPULSE=%d",
             GPS_UART_TX_GPIO, GPS_UART_RX_GPIO,
             GPS_RESET_N_GPIO, GPS_EXTINT_GPIO, GPS_SAFEBOOT_N_GPIO, GPS_TIMEPULSE_GPIO);

    // Try common baud rates and leave UART set to the detected one
    int detected = gps_try_baudrates();
    ESP_LOGI(TAG, "Using baud %d for main read loop.", detected);

    // Optionally allow using GPS L5 even when satellites flag it unhealthy
    // (recommended only for evaluation / non-safety-critical use)
    gps_enable_gps_l5_ignore_health_ram(true);

    xTaskCreate(gps_read_task, "gps_read_task", 4096, NULL, 10, NULL);
}