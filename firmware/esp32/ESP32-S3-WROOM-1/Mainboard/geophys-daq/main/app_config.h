#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "driver/gpio.h"

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

#endif /* APP_CONFIG_H */
