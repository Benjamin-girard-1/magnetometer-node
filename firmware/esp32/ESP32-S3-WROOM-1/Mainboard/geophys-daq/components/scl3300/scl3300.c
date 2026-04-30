/**
 * @file scl3300.c
 * @brief Platform-agnostic core for the SCL3300-D01 driver.
 *
 * Off-frame protocol (datasheet §5.1.2): the response to MOSI command N
 * arrives during MOSI command N+1's frame. So a single "read X" operation
 * is actually two SPI transactions, where the second transaction can be
 * any valid command — typically the NEXT read you want.
 *
 * Pattern used here:
 *   1. Send the request
 *   2. Send a "throwaway" request (we use Read STATUS) to harvest the response
 *   3. Validate CRC and RS bits, return DATA
 *
 * This pattern is correct but "wastes" one transaction per single-register
 * read. For the hot-path bulk read (scl3300_read_sample) we chain reads so
 * each transaction's data field is the previous command's answer.
 */

#include "scl3300.h"
#include "scl3300_regs.h"
#include "scl3300_port.h"

#include <string.h>

/* ------------------------------------------------------------------------- */
/*  CRC-8 (poly 0x1D, init 0xFF, XOR-out 0xFF) — datasheet §5.2 Figure 15   */
/* ------------------------------------------------------------------------- */
static uint8_t crc8_step(uint8_t bit_value, uint8_t crc)
{
    uint8_t temp = (uint8_t)(crc & 0x80);
    if (bit_value == 0x01) temp ^= 0x80;
    crc <<= 1;
    if (temp > 0) crc ^= 0x1D;
    return crc;
}

static uint8_t scl3300_crc(uint32_t data)
{
    /* CRC over the 24 MSBs, bits [31..8].                                   */
    uint8_t crc = 0xFF;
    for (int bit = 31; bit > 7; --bit) {
        uint8_t b = (uint8_t)((data >> bit) & 0x01);
        crc = crc8_step(b, crc);
    }
    return (uint8_t)~crc;
}

/* ------------------------------------------------------------------------- */
/*  Sensitivity (datasheet Table 12)                                         */
/* ------------------------------------------------------------------------- */
static float acc_sensitivity_lsb_per_g(scl3300_mode_t mode)
{
    switch (mode) {
        case SCL3300_MODE_1: return 6000.0f;
        case SCL3300_MODE_2: return 3000.0f;
        case SCL3300_MODE_3:
        case SCL3300_MODE_4: return 12000.0f;
        default: return 1.0f;
    }
}

/* Datasheet §2.4: T[°C] = -273 + (TEMP / 18.9) for raw 16-bit two's comp.  */
#define SCL3300_TEMP_LSB_PER_C    (18.9f)
#define SCL3300_TEMP_OFFSET_C     (-273.0f)

/* Datasheet §2.5: angle[°] = (raw_int / 2^14) * 90.                         */
#define SCL3300_ANG_DIV           (16384.0f)
#define SCL3300_ANG_FS_DEG        (90.0f)

/* ------------------------------------------------------------------------- */
/*  Frame helpers                                                            */
/* ------------------------------------------------------------------------- */

/**
 * Send one 32-bit frame, get one 32-bit response, validate the response's
 * CRC. The response is the answer to the *previous* command — this function
 * does NOT interpret RS bits because the RS in a startup-time response
 * legitimately reads "00" (startup in progress).
 */
static scl3300_status_t xfer_one(scl3300_t *dev, uint32_t tx, uint32_t *rx_out)
{
    uint32_t rx = 0;
    if (scl3300_port_xfer(dev->port_ctx, tx, &rx) != 0) {
        return SCL3300_ERR_BUS;
    }
    /* Validate the CRC field of the received frame.                         */
    uint8_t rx_crc = SCL3300_FRAME_CRC(rx);
    uint8_t expected = scl3300_crc(rx);
    if (rx_crc != expected) {
        return SCL3300_ERR_CRC;
    }
    if (rx_out) *rx_out = rx;
    return SCL3300_OK;
}

/**
 * "Read register" wrapper that handles off-frame timing: send the request,
 * send a follow-up command, return the DATA field of the second response.
 *
 * The follow-up command should be benign. We use Read STATUS, which is a
 * read-only side-effect-free op (well — reading STATUS clears it after the
 * response goes out, but we only care about isolated reads here). For
 * chained reads, callers use xfer_one() directly.
 */
static scl3300_status_t read_register(scl3300_t *dev, uint32_t op,
                                      uint16_t *data_out, uint8_t *rs_out)
{
    /* 1) Send the request — discard the response (it answers whatever was   */
    /*    sent before this call).                                            */
    scl3300_status_t st = xfer_one(dev, op, NULL);
    if (st != SCL3300_OK) return st;

    /* 2) Send a follow-up to harvest the answer.                            */
    uint32_t rx = 0;
    st = xfer_one(dev, SCL3300_OP_READ_STATUS, &rx);
    if (st != SCL3300_OK) return st;

    if (rs_out)   *rs_out   = (uint8_t)SCL3300_FRAME_RS(rx);
    if (data_out) *data_out = (uint16_t)SCL3300_FRAME_DATA(rx);
    return SCL3300_OK;
}

/* ------------------------------------------------------------------------- */
/*  Public API                                                               */
/* ------------------------------------------------------------------------- */

scl3300_status_t scl3300_software_reset(scl3300_t *dev)
{
    if (!dev) return SCL3300_ERR_PARAM;
    /* SW reset doesn't need a follow-up — fire and forget. The next         */
    /* command after reset returns RS=00 (startup).                          */
    return xfer_one(dev, SCL3300_OP_SW_RESET, NULL);
}

scl3300_status_t scl3300_read_status(scl3300_t *dev, uint16_t *status)
{
    return read_register(dev, SCL3300_OP_READ_STATUS, status, NULL);
}

scl3300_status_t scl3300_whoami(scl3300_t *dev, uint8_t *who)
{
    if (!who) return SCL3300_ERR_PARAM;
    uint16_t data;
    scl3300_status_t st = read_register(dev, SCL3300_OP_READ_WHOAMI, &data, NULL);
    if (st != SCL3300_OK) return st;
    *who = (uint8_t)(data & 0xFF);
    return SCL3300_OK;
}

scl3300_status_t scl3300_init(scl3300_t *dev, const scl3300_config_t *cfg)
{
    if (!dev || !cfg) return SCL3300_ERR_PARAM;

    memset(dev, 0, sizeof(*dev));
    if (scl3300_port_init(&dev->port_ctx) != 0) return SCL3300_ERR_BUS;

    /* Datasheet §4.2 step 1.2: wait 1 ms after power-up.                    */
    scl3300_port_delay_ms(2);

    /* Step 2: SW reset.                                                     */
    scl3300_status_t st = scl3300_software_reset(dev);
    if (st != SCL3300_OK) return st;

    /* Step 3: wait 1 ms.                                                    */
    scl3300_port_delay_ms(2);

    /* Step 4: select mode.                                                  */
    uint32_t mode_op;
    uint32_t settle_ms;
    switch (cfg->mode) {
        case SCL3300_MODE_1: mode_op = SCL3300_OP_CHANGE_MODE_1; settle_ms = 25;  break;
        case SCL3300_MODE_2: mode_op = SCL3300_OP_CHANGE_MODE_2; settle_ms = 15;  break;
        case SCL3300_MODE_3: mode_op = SCL3300_OP_CHANGE_MODE_3; settle_ms = 100; break;
        case SCL3300_MODE_4: mode_op = SCL3300_OP_CHANGE_MODE_4; settle_ms = 100; break;
        default: return SCL3300_ERR_PARAM;
    }
    st = xfer_one(dev, mode_op, NULL);
    if (st != SCL3300_OK) return st;

    /* Step 5: enable angle outputs (optional).                              */
    if (cfg->enable_angle_outputs) {
        st = xfer_one(dev, SCL3300_OP_ENABLE_ANGLE, NULL);
        if (st != SCL3300_OK) return st;
    }

    /* Step 6: settle wait.                                                  */
    scl3300_port_delay_ms(settle_ms);

    /* Steps 7–9: read STATUS three times to clear it and verify RS.         */
    /* First read: RS may be 11 (mode-changed flag), Data don't care.        */
    /* Second read: RS still 11 but STATUS data now visible.                 */
    /* Third read: RS should be 01 — this is our success criterion.          */
    uint8_t rs;
    uint16_t status;
    for (int i = 0; i < 3; ++i) {
        st = read_register(dev, SCL3300_OP_READ_STATUS, &status, &rs);
        if (st != SCL3300_OK) return st;
    }
    if (rs != SCL3300_RS_NORMAL) {
        /* Give it a few more reads — sometimes settle takes longer than the */
        /* nominal time on cold boot.                                        */
        for (int i = 0; i < 10 && rs != SCL3300_RS_NORMAL; ++i) {
            scl3300_port_delay_ms(5);
            st = read_register(dev, SCL3300_OP_READ_STATUS, &status, &rs);
            if (st != SCL3300_OK) return st;
        }
        if (rs != SCL3300_RS_NORMAL) return SCL3300_ERR_STARTUP;
    }

    /* WHOAMI sanity check.                                                  */
    uint8_t who;
    st = scl3300_whoami(dev, &who);
    if (st != SCL3300_OK) return st;
    dev->last_whoami = who;
    if (who != SCL3300_WHOAMI_VALUE) return SCL3300_ERR_WHOAMI;

    dev->mode           = cfg->mode;
    dev->angles_enabled = cfg->enable_angle_outputs;
    dev->initialized    = true;
    return SCL3300_OK;
}

/* ------------------------------------------------------------------------- */
/*  Single-axis reads (each is two transactions due to off-frame protocol)  */
/* ------------------------------------------------------------------------- */

scl3300_status_t scl3300_read_temp_raw(scl3300_t *dev, int16_t *raw)
{
    if (!raw) return SCL3300_ERR_PARAM;
    uint16_t d;
    scl3300_status_t st = read_register(dev, SCL3300_OP_READ_TEMP, &d, NULL);
    if (st != SCL3300_OK) return st;
    *raw = (int16_t)d;
    return SCL3300_OK;
}

/* ------------------------------------------------------------------------- */
/*  Chained reads — one SPI transaction per axis, no wasted frames.          */
/*                                                                           */
/*  Pattern: send the FIRST read; then for each subsequent read, the PREVIOUS*/
/*  command's data shows up. After the last desired read, send a STATUS to  */
/*  flush the answer.                                                        */
/* ------------------------------------------------------------------------- */

scl3300_status_t scl3300_read_acc_raw(scl3300_t *dev, scl3300_axis3_raw_t *out)
{
    if (!out) return SCL3300_ERR_PARAM;

    uint32_t rx;
    scl3300_status_t st;

    /* Send Read X — discard response (answers something prior).             */
    st = xfer_one(dev, SCL3300_OP_READ_ACC_X, NULL);
    if (st != SCL3300_OK) return st;

    /* Send Read Y — response is X.                                          */
    st = xfer_one(dev, SCL3300_OP_READ_ACC_Y, &rx);
    if (st != SCL3300_OK) return st;
    out->x = (int16_t)SCL3300_FRAME_DATA(rx);

    /* Send Read Z — response is Y.                                          */
    st = xfer_one(dev, SCL3300_OP_READ_ACC_Z, &rx);
    if (st != SCL3300_OK) return st;
    out->y = (int16_t)SCL3300_FRAME_DATA(rx);

    /* Send a flush (STATUS) — response is Z.                                */
    st = xfer_one(dev, SCL3300_OP_READ_STATUS, &rx);
    if (st != SCL3300_OK) return st;
    out->z = (int16_t)SCL3300_FRAME_DATA(rx);

    return SCL3300_OK;
}

scl3300_status_t scl3300_read_angle_raw(scl3300_t *dev, scl3300_axis3_raw_t *out)
{
    if (!out) return SCL3300_ERR_PARAM;
    if (!dev->angles_enabled) return SCL3300_ERR_PARAM;

    uint32_t rx;
    scl3300_status_t st;

    st = xfer_one(dev, SCL3300_OP_READ_ANG_X, NULL);
    if (st != SCL3300_OK) return st;

    st = xfer_one(dev, SCL3300_OP_READ_ANG_Y, &rx);
    if (st != SCL3300_OK) return st;
    out->x = (int16_t)SCL3300_FRAME_DATA(rx);

    st = xfer_one(dev, SCL3300_OP_READ_ANG_Z, &rx);
    if (st != SCL3300_OK) return st;
    out->y = (int16_t)SCL3300_FRAME_DATA(rx);

    st = xfer_one(dev, SCL3300_OP_READ_STATUS, &rx);
    if (st != SCL3300_OK) return st;
    out->z = (int16_t)SCL3300_FRAME_DATA(rx);

    return SCL3300_OK;
}

scl3300_status_t scl3300_read_sample(scl3300_t *dev, scl3300_sample_t *s)
{
    if (!s) return SCL3300_ERR_PARAM;

    /* Chain: ACC_X, ACC_Y, ACC_Z, [ANG_X, ANG_Y, ANG_Z], TEMP, STATUS.
     * Each transaction's response is the previous command's payload.        */
    const bool ang = dev->angles_enabled;
    uint32_t rx;
    scl3300_status_t st;
    int16_t ax, ay, az, gx = 0, gy = 0, gz = 0, t;

    st = xfer_one(dev, SCL3300_OP_READ_ACC_X, NULL); if (st) return st;
    st = xfer_one(dev, SCL3300_OP_READ_ACC_Y, &rx);  if (st) return st;
    ax = (int16_t)SCL3300_FRAME_DATA(rx);
    st = xfer_one(dev, SCL3300_OP_READ_ACC_Z, &rx);  if (st) return st;
    ay = (int16_t)SCL3300_FRAME_DATA(rx);

    uint32_t next = ang ? SCL3300_OP_READ_ANG_X : SCL3300_OP_READ_TEMP;
    st = xfer_one(dev, next, &rx);                   if (st) return st;
    az = (int16_t)SCL3300_FRAME_DATA(rx);

    if (ang) {
        st = xfer_one(dev, SCL3300_OP_READ_ANG_Y, &rx); if (st) return st;
        gx = (int16_t)SCL3300_FRAME_DATA(rx);
        st = xfer_one(dev, SCL3300_OP_READ_ANG_Z, &rx); if (st) return st;
        gy = (int16_t)SCL3300_FRAME_DATA(rx);
        st = xfer_one(dev, SCL3300_OP_READ_TEMP,  &rx); if (st) return st;
        gz = (int16_t)SCL3300_FRAME_DATA(rx);
    }

    /* Final flush: send STATUS to harvest the TEMP answer.                  */
    st = xfer_one(dev, SCL3300_OP_READ_STATUS, &rx);  if (st) return st;
    t = (int16_t)SCL3300_FRAME_DATA(rx);

    /* Convert.                                                              */
    const float k_acc = 1.0f / acc_sensitivity_lsb_per_g(dev->mode);
    s->acc_x_g = (float)ax * k_acc;
    s->acc_y_g = (float)ay * k_acc;
    s->acc_z_g = (float)az * k_acc;
    if (ang) {
        s->ang_x_deg = ((float)gx / SCL3300_ANG_DIV) * SCL3300_ANG_FS_DEG;
        s->ang_y_deg = ((float)gy / SCL3300_ANG_DIV) * SCL3300_ANG_FS_DEG;
        s->ang_z_deg = ((float)gz / SCL3300_ANG_DIV) * SCL3300_ANG_FS_DEG;
    } else {
        s->ang_x_deg = s->ang_y_deg = s->ang_z_deg = 0.0f;
    }
    s->temp_c = SCL3300_TEMP_OFFSET_C + ((float)t / SCL3300_TEMP_LSB_PER_C);
    return SCL3300_OK;
}
