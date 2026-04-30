/**
 * @file lsm6dsv_regs.h
 * @brief Register addresses & bit fields actually used by this driver.
 *
 * Source: ST datasheet DS13476 Rev 5, August 2023.
 * Add more here only when you need them — keep this file lean.
 */

#ifndef LSM6DSV_REGS_H
#define LSM6DSV_REGS_H

/* ------------------------------------------------------------------------- */
/*  SPI framing                                                              */
/* ------------------------------------------------------------------------- */
/* The R/W bit is the MSB of the address byte. 1 = read, 0 = write.          */
/* Multi-byte transfers auto-increment when CTRL3.IF_INC=1 (default after    */
/* power-on).                                                                */
#define LSM6DSV_SPI_READ_BIT     (0x80u)

/* ------------------------------------------------------------------------- */
/*  Register addresses (primary interface)                                   */
/* ------------------------------------------------------------------------- */
#define LSM6DSV_REG_FUNC_CFG_ACCESS  (0x01u)
#define LSM6DSV_REG_PIN_CTRL         (0x02u)
#define LSM6DSV_REG_IF_CFG           (0x03u)
#define LSM6DSV_REG_INT1_CTRL        (0x0Du)
#define LSM6DSV_REG_INT2_CTRL        (0x0Eu)
#define LSM6DSV_REG_WHO_AM_I         (0x0Fu)
#define LSM6DSV_REG_CTRL1            (0x10u)  /* XL ODR + op mode           */
#define LSM6DSV_REG_CTRL2            (0x11u)  /* G  ODR + op mode           */
#define LSM6DSV_REG_CTRL3            (0x12u)  /* BDU, IF_INC, SW_RESET, BOOT*/
#define LSM6DSV_REG_CTRL4            (0x13u)
#define LSM6DSV_REG_CTRL5            (0x14u)
#define LSM6DSV_REG_CTRL6            (0x15u)  /* G  full-scale              */
#define LSM6DSV_REG_CTRL7            (0x16u)
#define LSM6DSV_REG_CTRL8            (0x17u)  /* XL full-scale              */
#define LSM6DSV_REG_CTRL9            (0x18u)
#define LSM6DSV_REG_CTRL10           (0x19u)
#define LSM6DSV_REG_STATUS_REG       (0x1Eu)
#define LSM6DSV_REG_OUT_TEMP_L       (0x20u)
#define LSM6DSV_REG_OUTX_L_G         (0x22u)
#define LSM6DSV_REG_OUTX_L_A         (0x28u)

/* ------------------------------------------------------------------------- */
/*  Magic values                                                             */
/* ------------------------------------------------------------------------- */
#define LSM6DSV_WHO_AM_I_VALUE       (0x70u)

/* ------------------------------------------------------------------------- */
/*  Bit positions                                                            */
/* ------------------------------------------------------------------------- */
/* CTRL3 (0x12) */
#define LSM6DSV_CTRL3_SW_RESET       (1u << 0)
#define LSM6DSV_CTRL3_IF_INC         (1u << 2)
#define LSM6DSV_CTRL3_BDU            (1u << 6)
#define LSM6DSV_CTRL3_BOOT           (1u << 7)

/* STATUS_REG (0x1E) */
#define LSM6DSV_STATUS_XLDA          (1u << 0)
#define LSM6DSV_STATUS_GDA           (1u << 1)
#define LSM6DSV_STATUS_TDA           (1u << 2)

/* CTRL1 (0x10) — accelerometer
 *   bits [3:0] = ODR_XL[3:0]
 *   bits [6:4] = OP_MODE_XL[2:0]   (000 = high-performance, default)
 */
#define LSM6DSV_CTRL1_OP_MODE_HP     (0x00u << 4)

/* CTRL2 (0x11) — gyroscope, same layout as CTRL1                            */
#define LSM6DSV_CTRL2_OP_MODE_HP     (0x00u << 4)

/* CTRL6 (0x15) — bits [3:0] = FS_G[3:0]                                     */
/* CTRL8 (0x17) — bits [1:0] = FS_XL[1:0]                                    */

#endif /* LSM6DSV_REGS_H */
