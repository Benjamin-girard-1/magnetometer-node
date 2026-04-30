/**
 * @file shift_register.c
 * @brief Platform-agnostic logic. Only talks to sr_port.h — no vendor HAL.
 */

#include "shift_register.h"
#include "sr_port.h"

/* Cached state. Bit 0 is shifted out first (reaches U5); bit 15 stays in U4. */
static uint16_t s_sr_state = 0x0000;

static void sr_pulse_clock(void)
{
    sr_port_write(SR_LINE_SHIFT_CLK, true);
    sr_port_delay_short();
    sr_port_write(SR_LINE_SHIFT_CLK, false);
    sr_port_delay_short();
}

static void sr_pulse_latch(void)
{
    sr_port_write(SR_LINE_LATCH, true);
    sr_port_delay_short();
    sr_port_write(SR_LINE_LATCH, false);
}

/*
 * Shift 16 bits out, MSB first. Because U4 is upstream of U5, the bits
 * destined for U5 (0..7) must travel the farthest, so we push them first.
 * Walking from bit 15 down to bit 0 achieves that.
 */
static void sr_shift_out(uint16_t value)
{
    for (int i = 0; i < 16; ++i) {
        sr_port_write(SR_LINE_DATA, (value >> i) & 0x1);
        sr_pulse_clock();
    }
    sr_pulse_latch();
}

int sr_init(void)
{
    int err = sr_port_init();
    if (err != 0) {
        return err;
    }

    /* Register contents are random at power-on. Shift in zeros
     * to get a known state, then enable outputs. OE_N is already
     * high from sr_port_init(), so outputs stay Hi-Z until now. */
    s_sr_state = 0x0000;
    sr_shift_out(s_sr_state);

    sr_port_write(SR_LINE_OE_N, false);   /* enable outputs (active low) */
    return 0;
}

int sr_set_pin(sr_pin_t pin, bool level)
{
    if ((unsigned)pin > 15) {
        return -1;
    }

    const uint16_t mask = (uint16_t)(1u << pin);
    if (level) {
        s_sr_state |= mask;
    } else {
        s_sr_state &= (uint16_t)~mask;
    }

    sr_shift_out(s_sr_state);
    return 0;
}

bool sr_get_pin(sr_pin_t pin)
{
    if ((unsigned)pin > 15) {
        return false;
    }
    return ((s_sr_state >> pin) & 0x1) != 0;
}

int sr_write_all(uint16_t value)
{
    s_sr_state = value;
    sr_shift_out(s_sr_state);
    return 0;
}

void sr_output_enable(bool enable)
{
    sr_port_write(SR_LINE_OE_N, !enable);     /* active low */
}
