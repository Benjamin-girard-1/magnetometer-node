/**
 * @file lsm6dsv.c
 * @brief Platform-agnostic core for the LSM6DSV driver.
 */

#include "lsm6dsv.h"
#include "lsm6dsv_regs.h"
#include "lsm6dsv_port.h"

#include <string.h>

/* ------------------------------------------------------------------------- */
/*  Sensitivity tables (datasheet §2 Table 4)                                */
/*  XL: mg/LSB at each FS                                                    */
/*  G : mdps/LSB at each FS                                                  */
/* ------------------------------------------------------------------------- */
static float xl_sensitivity_g_per_lsb(lsm6dsv_xl_fs_t fs)
{
    switch (fs) {
        case LSM6DSV_XL_FS_2G:  return 0.061e-3f;
        case LSM6DSV_XL_FS_4G:  return 0.122e-3f;
        case LSM6DSV_XL_FS_8G:  return 0.244e-3f;
        case LSM6DSV_XL_FS_16G: return 0.488e-3f;
        default:                return 0.0f;
    }
}

static float g_sensitivity_dps_per_lsb(lsm6dsv_g_fs_t fs)
{
    switch (fs) {
        case LSM6DSV_G_FS_125DPS:  return  4.375e-3f;
        case LSM6DSV_G_FS_250DPS:  return  8.75e-3f;
        case LSM6DSV_G_FS_500DPS:  return 17.50e-3f;
        case LSM6DSV_G_FS_1000DPS: return 35.00e-3f;
        case LSM6DSV_G_FS_2000DPS: return 70.00e-3f;
        case LSM6DSV_G_FS_4000DPS: return 140.0e-3f;
        default:                   return 0.0f;
    }
}

/* Temperature: 256 LSB / °C, 0 LSB at 25 °C (datasheet §2 Table 4)          */
#define LSM6DSV_TEMP_SENSITIVITY  (1.0f / 256.0f)
#define LSM6DSV_TEMP_OFFSET_C     (25.0f)

/* ------------------------------------------------------------------------- */
/*  Low-level register helpers                                               */
/* ------------------------------------------------------------------------- */

lsm6dsv_status_t lsm6dsv_read_reg(lsm6dsv_t *dev, uint8_t reg,
                                  uint8_t *buf, size_t len)
{
    if (!dev || !buf || len == 0 || len > 32) {
        return LSM6DSV_ERR_PARAM;
    }
    /* Frame: [R|addr] [dummy * len]; read response is bytes [1..len].       */
    uint8_t tx[33] = { (uint8_t)(reg | LSM6DSV_SPI_READ_BIT) };
    uint8_t rx[33] = { 0 };

    if (lsm6dsv_port_xfer(dev->port_ctx, tx, rx, len + 1) != 0) {
        return LSM6DSV_ERR_BUS;
    }
    memcpy(buf, &rx[1], len);
    return LSM6DSV_OK;
}

lsm6dsv_status_t lsm6dsv_write_reg(lsm6dsv_t *dev, uint8_t reg,
                                   const uint8_t *buf, size_t len)
{
    if (!dev || !buf || len == 0 || len > 32) {
        return LSM6DSV_ERR_PARAM;
    }
    /* Frame: [W|addr] [data * len].  W bit = 0, so no OR.                   */
    uint8_t tx[33] = { (uint8_t)(reg & 0x7F) };
    memcpy(&tx[1], buf, len);

    if (lsm6dsv_port_xfer(dev->port_ctx, tx, NULL, len + 1) != 0) {
        return LSM6DSV_ERR_BUS;
    }
    return LSM6DSV_OK;
}

/* Convenience wrappers for single-byte reg access.                          */
static lsm6dsv_status_t read_u8(lsm6dsv_t *dev, uint8_t reg, uint8_t *val)
{
    return lsm6dsv_read_reg(dev, reg, val, 1);
}
static lsm6dsv_status_t write_u8(lsm6dsv_t *dev, uint8_t reg, uint8_t val)
{
    return lsm6dsv_write_reg(dev, reg, &val, 1);
}

/* ------------------------------------------------------------------------- */
/*  Public API                                                               */
/* ------------------------------------------------------------------------- */

lsm6dsv_status_t lsm6dsv_who_am_i(lsm6dsv_t *dev, uint8_t *who)
{
    if (!who) return LSM6DSV_ERR_PARAM;
    return read_u8(dev, LSM6DSV_REG_WHO_AM_I, who);
}

lsm6dsv_status_t lsm6dsv_software_reset(lsm6dsv_t *dev)
{
    /* Read-modify-write CTRL3 to set SW_RESET without disturbing other      */
    /* bits (BOOT, IF_INC default = 1, BDU, etc.).                           */
    uint8_t ctrl3;
    lsm6dsv_status_t st = read_u8(dev, LSM6DSV_REG_CTRL3, &ctrl3);
    if (st != LSM6DSV_OK) return st;

    ctrl3 |= LSM6DSV_CTRL3_SW_RESET;
    st = write_u8(dev, LSM6DSV_REG_CTRL3, ctrl3);
    if (st != LSM6DSV_OK) return st;

    /* Datasheet: SW_RESET self-clears in ~50 µs. Poll up to 50 ms.          */
    for (int i = 0; i < 50; ++i) {
        lsm6dsv_port_delay_ms(1);
        st = read_u8(dev, LSM6DSV_REG_CTRL3, &ctrl3);
        if (st != LSM6DSV_OK) return st;
        if ((ctrl3 & LSM6DSV_CTRL3_SW_RESET) == 0) {
            return LSM6DSV_OK;
        }
    }
    return LSM6DSV_ERR_TIMEOUT;
}

lsm6dsv_status_t lsm6dsv_init(lsm6dsv_t *dev, const lsm6dsv_config_t *cfg)
{
    if (!dev || !cfg) return LSM6DSV_ERR_PARAM;

    memset(dev, 0, sizeof(*dev));

    if (lsm6dsv_port_init(&dev->port_ctx) != 0) {
        return LSM6DSV_ERR_BUS;
    }

    /* Datasheet §6.1: 10 ms boot time after power-on before any access.     */
    lsm6dsv_port_delay_ms(15);

    /* WHO_AM_I check — also confirms SPI plumbing is alive.                 */
    /* Read it 4 times so a glitched first byte (e.g. CS-not-yet-driven)     */
    /* doesn't fail us silently. Also helps spot a stuck-bus pattern (all    */
    /* 0x00 or all 0xFF).                                                    */
    uint8_t who[4] = {0};
    for (int i = 0; i < 4; ++i) {
        lsm6dsv_status_t r = lsm6dsv_who_am_i(dev, &who[i]);
        if (r != LSM6DSV_OK) return r;
    }
    /* Stash the last read in the device handle so the caller can log it.   */
    dev->last_who_am_i = who[3];
    if (who[3] != LSM6DSV_WHO_AM_I_VALUE) {
        return LSM6DSV_ERR_WHO_AM_I;
    }

    lsm6dsv_status_t st = lsm6dsv_software_reset(dev);
    if (st != LSM6DSV_OK) return st;

    /* CTRL3: BDU + IF_INC. Skip BOOT — sw reset already covers it.          */
    uint8_t ctrl3 = 0;
    if (cfg->block_data_update) ctrl3 |= LSM6DSV_CTRL3_BDU;
    if (cfg->auto_increment)    ctrl3 |= LSM6DSV_CTRL3_IF_INC;
    st = write_u8(dev, LSM6DSV_REG_CTRL3, ctrl3);
    if (st != LSM6DSV_OK) return st;

    /* CTRL8: XL full-scale (bits [1:0]).                                    */
    st = write_u8(dev, LSM6DSV_REG_CTRL8, (uint8_t)(cfg->xl_fs & 0x03));
    if (st != LSM6DSV_OK) return st;

    /* CTRL6: gyro full-scale (bits [3:0]).                                  */
    st = write_u8(dev, LSM6DSV_REG_CTRL6, (uint8_t)(cfg->g_fs & 0x0F));
    if (st != LSM6DSV_OK) return st;

    /* CTRL1: XL ODR (bits [3:0]) + OP_MODE high-performance (bits [6:4]=0). */
    st = write_u8(dev, LSM6DSV_REG_CTRL1,
                  LSM6DSV_CTRL1_OP_MODE_HP | (cfg->xl_odr & 0x0F));
    if (st != LSM6DSV_OK) return st;

    /* CTRL2: G ODR + OP_MODE.                                               */
    st = write_u8(dev, LSM6DSV_REG_CTRL2,
                  LSM6DSV_CTRL2_OP_MODE_HP | (cfg->g_odr & 0x0F));
    if (st != LSM6DSV_OK) return st;

    dev->xl_fs       = cfg->xl_fs;
    dev->g_fs        = cfg->g_fs;
    dev->initialized = true;
    return LSM6DSV_OK;
}

/* ------------------------------------------------------------------------- */
/*  Data-path helpers                                                        */
/* ------------------------------------------------------------------------- */

/* Pack 6 bytes (LE: x_l x_h y_l y_h z_l z_h) into an axis3 struct.          */
static void pack_axis3(const uint8_t *p, lsm6dsv_axis3_raw_t *out)
{
    out->x = (int16_t)((uint16_t)p[1] << 8 | p[0]);
    out->y = (int16_t)((uint16_t)p[3] << 8 | p[2]);
    out->z = (int16_t)((uint16_t)p[5] << 8 | p[4]);
}

lsm6dsv_status_t lsm6dsv_read_xl_raw(lsm6dsv_t *dev, lsm6dsv_axis3_raw_t *out)
{
    if (!out) return LSM6DSV_ERR_PARAM;
    uint8_t buf[6];
    lsm6dsv_status_t st = lsm6dsv_read_reg(dev, LSM6DSV_REG_OUTX_L_A, buf, 6);
    if (st != LSM6DSV_OK) return st;
    pack_axis3(buf, out);
    return LSM6DSV_OK;
}

lsm6dsv_status_t lsm6dsv_read_g_raw(lsm6dsv_t *dev, lsm6dsv_axis3_raw_t *out)
{
    if (!out) return LSM6DSV_ERR_PARAM;
    uint8_t buf[6];
    lsm6dsv_status_t st = lsm6dsv_read_reg(dev, LSM6DSV_REG_OUTX_L_G, buf, 6);
    if (st != LSM6DSV_OK) return st;
    pack_axis3(buf, out);
    return LSM6DSV_OK;
}

lsm6dsv_status_t lsm6dsv_read_temp_raw(lsm6dsv_t *dev, int16_t *raw)
{
    if (!raw) return LSM6DSV_ERR_PARAM;
    uint8_t buf[2];
    lsm6dsv_status_t st = lsm6dsv_read_reg(dev, LSM6DSV_REG_OUT_TEMP_L, buf, 2);
    if (st != LSM6DSV_OK) return st;
    *raw = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    return LSM6DSV_OK;
}

lsm6dsv_status_t lsm6dsv_data_ready(lsm6dsv_t *dev,
                                    bool *xl_ready,
                                    bool *g_ready,
                                    bool *temp_ready)
{
    uint8_t s;
    lsm6dsv_status_t st = read_u8(dev, LSM6DSV_REG_STATUS_REG, &s);
    if (st != LSM6DSV_OK) return st;
    if (xl_ready)   *xl_ready   = (s & LSM6DSV_STATUS_XLDA) != 0;
    if (g_ready)    *g_ready    = (s & LSM6DSV_STATUS_GDA)  != 0;
    if (temp_ready) *temp_ready = (s & LSM6DSV_STATUS_TDA)  != 0;
    return LSM6DSV_OK;
}

lsm6dsv_status_t lsm6dsv_read_sample(lsm6dsv_t *dev, lsm6dsv_sample_t *s)
{
    if (!s) return LSM6DSV_ERR_PARAM;

    /* Burst-read OUT_TEMP_L (0x20) through OUTZ_H_A (0x2D) = 14 bytes.
     * Layout: temp_l temp_h | gx_l gx_h gy_l gy_h gz_l gz_h
     *                       | ax_l ax_h ay_l ay_h az_l az_h               */
    uint8_t buf[14];
    lsm6dsv_status_t st = lsm6dsv_read_reg(dev, LSM6DSV_REG_OUT_TEMP_L,
                                           buf, sizeof(buf));
    if (st != LSM6DSV_OK) return st;

    int16_t temp_raw = (int16_t)((uint16_t)buf[1]  << 8 | buf[0]);
    lsm6dsv_axis3_raw_t g_raw, xl_raw;
    pack_axis3(&buf[2],  &g_raw);
    pack_axis3(&buf[8],  &xl_raw);

    const float k_xl = xl_sensitivity_g_per_lsb(dev->xl_fs);
    const float k_g  = g_sensitivity_dps_per_lsb(dev->g_fs);

    s->xl_x_g  = (float)xl_raw.x * k_xl;
    s->xl_y_g  = (float)xl_raw.y * k_xl;
    s->xl_z_g  = (float)xl_raw.z * k_xl;
    s->g_x_dps = (float)g_raw.x  * k_g;
    s->g_y_dps = (float)g_raw.y  * k_g;
    s->g_z_dps = (float)g_raw.z  * k_g;
    s->temp_c  = (float)temp_raw * LSM6DSV_TEMP_SENSITIVITY
                 + LSM6DSV_TEMP_OFFSET_C;
    return LSM6DSV_OK;
}