/**
 * @file scl3300.h
 * @brief Platform-agnostic driver for the Murata SCL3300-D01 inclinometer.
 *
 * Same architecture as the LSM6DSV driver in this project: the core logic
 * here calls only the three functions in scl3300_port.h. To support a new
 * MCU, implement that file — never touch scl3300.c.
 *
 * Wiring assumed (datasheet §2.8 + your board's {IMU_SPI} bus):
 *   CSB on a dedicated GPIO (the port layer drives it).
 *   MISO/MOSI/SCK shared with the LSM6DSV.
 *   No interrupt lines — driver is polled.
 */

#ifndef SCL3300_H
#define SCL3300_H

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
    SCL3300_OK            =  0,
    SCL3300_ERR_BUS       = -1,  /* SPI transfer failed at the port layer  */
    SCL3300_ERR_WHOAMI    = -2,  /* WHOAMI returned something other than 0xC1 */
    SCL3300_ERR_PARAM     = -3,
    SCL3300_ERR_TIMEOUT   = -4,
    SCL3300_ERR_CRC       = -5,  /* CRC mismatch on a MISO frame           */
    SCL3300_ERR_RS        = -6,  /* Return Status field reported error     */
    SCL3300_ERR_STARTUP   = -7,  /* RS bits never reached "normal" state   */
} scl3300_status_t;

/* ------------------------------------------------------------------------- */
/*  Operation modes (datasheet §4.3, Table 12)                               */
/* ------------------------------------------------------------------------- */
typedef enum {
    SCL3300_MODE_1 = 1,  /* ±1.2 g full-scale,  40 Hz LPF, 6000 LSB/g       */
    SCL3300_MODE_2 = 2,  /* ±2.4 g full-scale,  70 Hz LPF, 3000 LSB/g       */
    SCL3300_MODE_3 = 3,  /* Inclination, ±10°, 10 Hz LPF, 12000 LSB/g       */
    SCL3300_MODE_4 = 4,  /* Inclination, ±10°, 10 Hz LPF, low-noise         */
} scl3300_mode_t;

/** Initial configuration. */
typedef struct {
    scl3300_mode_t mode;
    bool           enable_angle_outputs;  /* recommended true               */
} scl3300_config_t;

/** Default config: mode 1 (1.2 g full-scale, 40 Hz LPF), angles enabled.   */
#define SCL3300_CONFIG_DEFAULT() ((scl3300_config_t){ \
    .mode = SCL3300_MODE_1,                           \
    .enable_angle_outputs = true,                     \
})

/* ------------------------------------------------------------------------- */
/*  Data containers                                                          */
/* ------------------------------------------------------------------------- */

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} scl3300_axis3_raw_t;

/** Converted reading (acc in g, angles in degrees, temp in °C). */
typedef struct {
    float acc_x_g, acc_y_g, acc_z_g;
    float ang_x_deg, ang_y_deg, ang_z_deg;
    float temp_c;
} scl3300_sample_t;

/* ------------------------------------------------------------------------- */
/*  Driver handle                                                            */
/* ------------------------------------------------------------------------- */

typedef struct {
    void          *port_ctx;
    scl3300_mode_t mode;            /* cached for unit conversion           */
    bool           angles_enabled;
    bool           initialized;
    uint8_t        last_whoami;     /* set during init for diagnostics      */
} scl3300_t;

/* ------------------------------------------------------------------------- */
/*  Public API                                                               */
/* ------------------------------------------------------------------------- */

/**
 * Run the full datasheet §4.2 startup sequence:
 *   1. SW reset, 2. wait 1 ms, 3. set mode, 4. enable angles (optional),
 *   5. mode-dependent settle wait, 6. read STATUS twice to clear flags,
 *   7. verify RS bits, 8. read WHOAMI = 0xC1.
 */
scl3300_status_t scl3300_init(scl3300_t *dev, const scl3300_config_t *cfg);

/** Read WHOAMI. Returns 0xC1 in @p who on success. */
scl3300_status_t scl3300_whoami(scl3300_t *dev, uint8_t *who);

/** Read raw 16-bit acceleration on all three axes. */
scl3300_status_t scl3300_read_acc_raw(scl3300_t *dev, scl3300_axis3_raw_t *out);

/** Read raw 16-bit angle on all three axes (requires angles_enabled). */
scl3300_status_t scl3300_read_angle_raw(scl3300_t *dev, scl3300_axis3_raw_t *out);

/** Read raw temperature register. */
scl3300_status_t scl3300_read_temp_raw(scl3300_t *dev, int16_t *raw);

/** Read everything (acc + angles + temp) and convert to engineering units. */
scl3300_status_t scl3300_read_sample(scl3300_t *dev, scl3300_sample_t *s);

/** Read STATUS summary (datasheet §6.3). Returns 0 if no flags set. */
scl3300_status_t scl3300_read_status(scl3300_t *dev, uint16_t *status);

/** Software reset (datasheet operation 'SW Reset'). */
scl3300_status_t scl3300_software_reset(scl3300_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* SCL3300_H */
