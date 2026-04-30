/**
 * @file sr_port_esp32.c
 * @brief ESP32-S3 implementation of sr_port.h.
 */

#include "sr_port.h"

#include "driver/gpio.h"
#include "rom/ets_sys.h"

/* Pin assignment — matches the schematic. */
#define PIN_DATA       GPIO_NUM_19
#define PIN_SHIFT_CLK  GPIO_NUM_47
#define PIN_LATCH      GPIO_NUM_21
#define PIN_OE_N       GPIO_NUM_20

static gpio_num_t pin_of(sr_line_t line)
{
    switch (line) {
        case SR_LINE_DATA:      return PIN_DATA;
        case SR_LINE_SHIFT_CLK: return PIN_SHIFT_CLK;
        case SR_LINE_LATCH:     return PIN_LATCH;
        case SR_LINE_OE_N:      return PIN_OE_N;
        default:                return GPIO_NUM_NC;
    }
}

int sr_port_init(void)
{
    /* Step 1: set desired initial levels on the output latches
     * BEFORE switching the pins to output mode. The ESP32 latches
     * a drive level per GPIO that takes effect the instant the pin
     * becomes an output. If we skip this, OE_N goes low for a few
     * microseconds and whatever garbage is in the shift register
     * gets latched to the outputs. */
    gpio_set_level(PIN_OE_N,      1);  /* outputs disabled */
    gpio_set_level(PIN_LATCH,     0);
    gpio_set_level(PIN_SHIFT_CLK, 0);
    gpio_set_level(PIN_DATA,      0);

    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_DATA)      |
                        (1ULL << PIN_SHIFT_CLK) |
                        (1ULL << PIN_LATCH)     |
                        (1ULL << PIN_OE_N),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    return gpio_config(&cfg);
}

void sr_port_write(sr_line_t line, bool level)
{
    gpio_set_level(pin_of(line), level ? 1 : 0);
}

void sr_port_delay_short(void)
{
    ets_delay_us(1);
}
