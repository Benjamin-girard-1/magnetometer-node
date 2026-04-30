/**
 * @file lsm6dsv.h
 * @brief Platform-agnostic driver for the ST LSM6DSV(ETR) 6-axis IMU.
 *
 * This driver only depends on the three functions declared in lsm6dsv_port.h.
 * To port to a new MCU, implement that file — never touch lsm6dsv.c.
 *
 * Wiring assumed (Mode 1 — primary 4-wire SPI, see datasheet §7.1):
 *   SDx (pin 2) and SCx (pin 3) tied to GND.
 *   SDO_Aux / OCS_Aux (pins 10, 11) left unconnected.
 *   INT1 / INT2 unconnected — driver polls STATUS_REG.
 */

#ifndef LSM6DSV_H
#define LSM6DSV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/*  Return codes                                                             */
/* ------------------------------------------------------------------------- */

typedef enum {
    LSM6DSV_OK            =  0,
    LSM6DSV_ERR_BUS       = -1,  /* SPI transfer failed at the port layer  */
    LSM6DSV_ERR_WHO_AM_I  = -2,  /* Got something other than 0x70          */
    LSM6DSV_ERR_PARAM     = -3,  /* Bad argument                           */
    LSM6DSV_ERR_TIMEOUT   = -4,  /* Sensor never produced data             */
} lsm6dsv_status_t;

/* ------------------------------------------------------------------------- */
/*  Configuration enums                                                      */
/* ------------------------------------------------------------------------- */

/** Accelerometer output data rate (CTRL1.ODR_XL[3:0]). */
typedef enum {
    LSM6DSV_XL_ODR_OFF      = 0x0,
    LSM6DSV_XL_ODR_1Hz875   = 0x1,  /* low-power only                      */
    LSM6DSV_XL_ODR_7Hz5     = 0x2,
    LSM6DSV_XL_ODR_15Hz     = 0x3,
    LSM6DSV_XL_ODR_30Hz     = 0x4,
    LSM6DSV_XL_ODR_60Hz     = 0x5,
    LSM6DSV_XL_ODR_120Hz    = 0x6,
    LSM6DSV_XL_ODR_240Hz    = 0x7,
    LSM6DSV_XL_ODR_480Hz    = 0x8,
    LSM6DSV_XL_ODR_960Hz    = 0x9,
    LSM6DSV_XL_ODR_1920Hz   = 0xA,
    LSM6DSV_XL_ODR_3840Hz   = 0xB,
    LSM6DSV_XL_ODR_7680Hz   = 0xC,
} lsm6dsv_xl_odr_t;

/** Gyroscope output data rate (CTRL2.ODR_G[3:0]). */
typedef enum {
    LSM6DSV_G_ODR_OFF       = 0x0,
    LSM6DSV_G_ODR_7Hz5      = 0x2,
    LSM6DSV_G_ODR_15Hz      = 0x3,
    LSM6DSV_G_ODR_30Hz      = 0x4,
    LSM6DSV_G_ODR_60Hz      = 0x5,
    LSM6DSV_G_ODR_120Hz     = 0x6,
    LSM6DSV_G_ODR_240Hz     = 0x7,
    LSM6DSV_G_ODR_480Hz     = 0x8,
    LSM6DSV_G_ODR_960Hz     = 0x9,
    LSM6DSV_G_ODR_1920Hz    = 0xA,
    LSM6DSV_G_ODR_3840Hz    = 0xB,
    LSM6DSV_G_ODR_7680Hz    = 0xC,
} lsm6dsv_g_odr_t;

/** Accelerometer full-scale (CTRL8.FS_XL[1:0]). */
typedef enum {
    LSM6DSV_XL_FS_2G  = 0x0,
    LSM6DSV_XL_FS_4G  = 0x1,
    LSM6DSV_XL_FS_8G  = 0x2,
    LSM6DSV_XL_FS_16G = 0x3,
} lsm6dsv_xl_fs_t;

/** Gyroscope full-scale (CTRL6.FS_G[3:0]). */
typedef enum {
    LSM6DSV_G_FS_125DPS  = 0x0,
    LSM6DSV_G_FS_250DPS  = 0x1,
    LSM6DSV_G_FS_500DPS  = 0x2,
    LSM6DSV_G_FS_1000DPS = 0x3,
    LSM6DSV_G_FS_2000DPS = 0x4,
    LSM6DSV_G_FS_4000DPS = 0xC,  /* available only with specific config    */
} lsm6dsv_g_fs_t;

/** Initial configuration. Pass to lsm6dsv_init(). */
typedef struct {
    lsm6dsv_xl_odr_t xl_odr;
    lsm6dsv_g_odr_t  g_odr;
    lsm6dsv_xl_fs_t  xl_fs;
    lsm6dsv_g_fs_t   g_fs;
    bool             block_data_update; /* CTRL3.BDU — recommended true   */
    bool             auto_increment;    /* CTRL3.IF_INC — true for burst  */
} lsm6dsv_config_t;

/** Default config: 120 Hz combo, ±4g, ±2000 dps, BDU + auto-inc on. */
#define LSM6DSV_CONFIG_DEFAULT() ((lsm6dsv_config_t){ \
    .xl_odr = LSM6DSV_XL_ODR_120Hz,  \
    .g_odr  = LSM6DSV_G_ODR_120Hz,   \
    .xl_fs  = LSM6DSV_XL_FS_4G,      \
    .g_fs   = LSM6DSV_G_FS_2000DPS,  \
    .block_data_update = true,       \
    .auto_increment    = true,       \
})

/* ------------------------------------------------------------------------- */
/*  Data containers                                                          */
/* ------------------------------------------------------------------------- */

/** Raw 16-bit two's complement data straight from the device. */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} lsm6dsv_axis3_raw_t;

/** Converted reading (XL in g, gyro in dps, temperature in °C). */
typedef struct {
    float xl_x_g,  xl_y_g,  xl_z_g;
    float g_x_dps, g_y_dps, g_z_dps;
    float temp_c;
} lsm6dsv_sample_t;

/* ------------------------------------------------------------------------- */
/*  Driver handle                                                            */
/* ------------------------------------------------------------------------- */

/**
 * Opaque-ish handle. The user passes one of these to every call. The port
 * layer stores its bus/CS state inside `port_ctx` (an opaque void*) so that
 * one process can drive multiple LSM6DSVs sharing the same SPI bus.
 */
typedef struct {
    void              *port_ctx;       /* set by lsm6dsv_port_init()      */
    lsm6dsv_xl_fs_t    xl_fs;          /* cached for unit conversion      */
    lsm6dsv_g_fs_t     g_fs;
    bool               initialized;
    uint8_t            last_who_am_i;  /* set during init for diagnostics */
} lsm6dsv_t;

/* ------------------------------------------------------------------------- */
/*  Public API                                                               */
/* ------------------------------------------------------------------------- */

/**
 * Initialize the SPI bus + CS pin (via the port layer), verify WHO_AM_I,
 * software-reset the device, then apply @p cfg.
 *
 * @param dev  driver handle (zero-init before first call)
 * @param cfg  configuration; pass &LSM6DSV_CONFIG_DEFAULT() for sane defaults
 */
lsm6dsv_status_t lsm6dsv_init(lsm6dsv_t *dev, const lsm6dsv_config_t *cfg);

/** Read WHO_AM_I (0Fh). Should return 0x70 in @p who. */
lsm6dsv_status_t lsm6dsv_who_am_i(lsm6dsv_t *dev, uint8_t *who);

/** Trigger a software reset (CTRL3.SW_RESET) and wait for completion. */
lsm6dsv_status_t lsm6dsv_software_reset(lsm6dsv_t *dev);

/** Read raw accelerometer data (OUTX_L_A..OUTZ_H_A, 28h..2Dh). */
lsm6dsv_status_t lsm6dsv_read_xl_raw(lsm6dsv_t *dev, lsm6dsv_axis3_raw_t *out);

/** Read raw gyroscope data (OUTX_L_G..OUTZ_H_G, 22h..27h). */
lsm6dsv_status_t lsm6dsv_read_g_raw(lsm6dsv_t *dev, lsm6dsv_axis3_raw_t *out);

/** Read raw temperature (OUT_TEMP_L/H, 20h..21h). */
lsm6dsv_status_t lsm6dsv_read_temp_raw(lsm6dsv_t *dev, int16_t *raw);

/**
 * Read all 14 output bytes in a single burst (OUT_TEMP_L through OUTZ_H_A)
 * and convert to engineering units. This is the fast path you want in a
 * sample loop. Requires auto_increment=true in the initial config.
 */
lsm6dsv_status_t lsm6dsv_read_sample(lsm6dsv_t *dev, lsm6dsv_sample_t *s);

/** STATUS_REG (1Eh) bits 0/1: gyro/XL data-ready. Polled equivalent of DRDY. */
lsm6dsv_status_t lsm6dsv_data_ready(lsm6dsv_t *dev,
                                    bool *xl_ready,
                                    bool *g_ready,
                                    bool *temp_ready);

/** Direct register access — escape hatch for features the API doesn't cover. */
lsm6dsv_status_t lsm6dsv_read_reg (lsm6dsv_t *dev, uint8_t reg,
                                   uint8_t *buf, size_t len);
lsm6dsv_status_t lsm6dsv_write_reg(lsm6dsv_t *dev, uint8_t reg,
                                   const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSV_H */