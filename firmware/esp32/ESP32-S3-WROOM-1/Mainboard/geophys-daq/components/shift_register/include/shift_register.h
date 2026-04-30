/**
 * @file shift_register.h
 * @brief Driver for the daisy-chained 74HC595 pair (U4 + U5).
 *
 * This header is 100% platform-independent. Porting to a new MCU only
 * requires providing an implementation of sr_port.h.
 *
 */

#ifndef SHIFT_REGISTER_H
#define SHIFT_REGISTER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    /* U5 outputs — bits 0..7 (bit 0 travels farthest) */
    SR_SET_1          = 0,
    SR_RESET_1        = 1,
    SR_ADC_CONVST_SAR = 2,
    SR_ADC_MCLK_EN    = 3,
    SR_ADC_START      = 4,
    SR_ADC_RESET      = 5,
    SR_SET_2          = 6,
    SR_RESET_2        = 7,

    /* U4 outputs — bits 8..15 */
    SR_EN_KEEPALIVE   = 8,
    SR_EN_SD_MUX      = 9,
    SR_USB2641_NRESET = 10,
    SR_EN_INV_NEG5V   = 11,
    SR_EN_BST_18V     = 12,
    SR_EN_BST_10V     = 13,
    SR_EN_LDO_3V3     = 14,
    SR_SD_MUX_SEL     = 15,
} sr_pin_t;

int      sr_init(void);
int      sr_set_pin(sr_pin_t pin, bool level);
bool     sr_get_pin(sr_pin_t pin);
int      sr_write_all(uint16_t value);
void     sr_output_enable(bool enable);

#endif /* SHIFT_REGISTER_H */
