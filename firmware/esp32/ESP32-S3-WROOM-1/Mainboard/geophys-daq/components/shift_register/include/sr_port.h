/**
 * @file sr_port.h
 * @brief Platform abstraction for the shift register driver.
 *
 * Each supported MCU implements these four functions in its own .c file
 * (sr_port_esp32.c, sr_port_stm32.c, ...). The rest of the driver is
 * platform-agnostic and never calls vendor HAL code directly.
 */

#ifndef SR_PORT_H
#define SR_PORT_H

#include <stdbool.h>
#include <stdint.h>

/** Logical line identifiers — the port implementation maps these to real pins. */
typedef enum {
    SR_LINE_DATA = 0,
    SR_LINE_SHIFT_CLK,
    SR_LINE_LATCH,
    SR_LINE_OE_N,
} sr_line_t;

/** Configure the four GPIOs as push-pull outputs. Called once at startup. */
int  sr_port_init(void);

/** Drive a single line high or low. */
void sr_port_write(sr_line_t line, bool level);

/** Short busy-wait, used between clock edges. A few hundred ns is enough. */
void sr_port_delay_short(void);

#endif /* SR_PORT_H */
