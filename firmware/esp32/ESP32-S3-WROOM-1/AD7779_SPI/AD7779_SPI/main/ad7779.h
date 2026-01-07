#ifndef AD7779_H
#define AD7779_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"

/* Type alias: use spi_device_handle_t directly */
typedef spi_device_handle_t ad7779_handle_t;

/* ============================================================
 * Register addresses (Table 44 "Register Summary")
 * ============================================================
 * Only 8-bit addresses are used on the wire; we keep them as uint16_t
 * mostly to make macros clearer.
 */

/* Channel configuration registers */
#define AD7779_REG_CH0_CONFIG           0x000
#define AD7779_REG_CH1_CONFIG           0x001
#define AD7779_REG_CH2_CONFIG           0x002
#define AD7779_REG_CH3_CONFIG           0x003
#define AD7779_REG_CH4_CONFIG           0x004
#define AD7779_REG_CH5_CONFIG           0x005
#define AD7779_REG_CH6_CONFIG           0x006
#define AD7779_REG_CH7_CONFIG           0x007

/* Disable clocks to ADC channels */
#define AD7779_REG_CH_DISABLE           0x008

/* Per-channel sync offset */
#define AD7779_REG_CH0_SYNC_OFFSET      0x009
#define AD7779_REG_CH1_SYNC_OFFSET      0x00A
#define AD7779_REG_CH2_SYNC_OFFSET      0x00B
#define AD7779_REG_CH3_SYNC_OFFSET      0x00C
#define AD7779_REG_CH4_SYNC_OFFSET      0x00D
#define AD7779_REG_CH5_SYNC_OFFSET      0x00E
#define AD7779_REG_CH6_SYNC_OFFSET      0x00F
#define AD7779_REG_CH7_SYNC_OFFSET      0x010

/* General user configuration registers */
#define AD7779_REG_GENERAL_USER_CONFIG_1  0x011
#define AD7779_REG_GENERAL_USER_CONFIG_2  0x012
#define AD7779_REG_GENERAL_USER_CONFIG_3  0x013

/* Data output format register */
#define AD7779_REG_DOUT_FORMAT          0x014

/* Mux control */
#define AD7779_REG_ADC_MUX_CONFIG       0x015
#define AD7779_REG_GLOBAL_MUX_CONFIG    0x016

/* GPIO + reference buffer configuration */
#define AD7779_REG_GPIO_CONFIG          0x017
#define AD7779_REG_GPIO_DATA            0x018
#define AD7779_REG_BUFFER_CONFIG_1      0x019
#define AD7779_REG_BUFFER_CONFIG_2      0x01A

/* Channel 0 offset/gain calibration */
#define AD7779_REG_CH0_OFFSET_UPPER     0x01C
#define AD7779_REG_CH0_OFFSET_MID       0x01D
#define AD7779_REG_CH0_OFFSET_LOWER     0x01E
#define AD7779_REG_CH0_GAIN_UPPER       0x01F
#define AD7779_REG_CH0_GAIN_MID         0x020
#define AD7779_REG_CH0_GAIN_LOWER       0x021

/* Channel 1 offset/gain calibration */
#define AD7779_REG_CH1_OFFSET_UPPER     0x022
#define AD7779_REG_CH1_OFFSET_MID       0x023
#define AD7779_REG_CH1_OFFSET_LOWER     0x024
#define AD7779_REG_CH1_GAIN_UPPER       0x025
#define AD7779_REG_CH1_GAIN_MID         0x026
#define AD7779_REG_CH1_GAIN_LOWER       0x027

/* Generic macros for offset/gain registers for channel n (0–7).
 * Each channel block is 6 bytes: offset[3] + gain[3].
 */
#define AD7779_REG_CHx_OFFSET_UPPER(ch)  (0x01C + (uint16_t)(6 * (ch)))
#define AD7779_REG_CHx_OFFSET_MID(ch)    (0x01D + (uint16_t)(6 * (ch)))
#define AD7779_REG_CHx_OFFSET_LOWER(ch)  (0x01E + (uint16_t)(6 * (ch)))
#define AD7779_REG_CHx_GAIN_UPPER(ch)    (0x01F + (uint16_t)(6 * (ch)))
#define AD7779_REG_CHx_GAIN_MID(ch)      (0x020 + (uint16_t)(6 * (ch)))
#define AD7779_REG_CHx_GAIN_LOWER(ch)    (0x021 + (uint16_t)(6 * (ch)))

/* Channel status/error / general error / SRC registers around 0x054+  */
#define AD7779_REG_CH0_CH1_ERR          0x054
#define AD7779_REG_CH2_CH3_ERR          0x055
#define AD7779_REG_CH4_CH5_ERR          0x056
#define AD7779_REG_CH6_CH7_ERR          0x057   /* CH6_7_SAT_ERR in details */

#define AD7779_REG_CHX_ERR_REG_EN       0x058   /* CHX_ERR_REG_EN */
#define AD7779_REG_GEN_ERR_REG_1        0x059
#define AD7779_REG_GEN_ERR_REG_1_EN     0x05A

/* SRC (sample rate converter) registers */
#define AD7779_REG_SRC_CONFIG           0x060
#define AD7779_REG_SRC_UPDATE           0x061
#define AD7779_REG_DEC_RATE_HI          0x062
#define AD7779_REG_DEC_RATE_MID         0x063
#define AD7779_REG_DEC_RATE_LOW         0x064


/* ============================================================
 * Bit fields – only for the registers you’re likely to touch soon
 * ============================================================
 */

/* ---------- GENERAL_USER_CONFIG_1 (0x011) ---------- */
#define AD7779_GUC1_ALL_CH_DIS_MCLK_EN   (1u << 7)
#define AD7779_GUC1_POWERMODE            (1u << 6)  /* 0 = low power, 1 = high res */
#define AD7779_GUC1_PDB_VCM              (1u << 5)  /* active low power-down */
#define AD7779_GUC1_PDB_REFOUT_BUF       (1u << 4)  /* active low */
#define AD7779_GUC1_PDB_SAR              (1u << 3)  /* active low */
#define AD7779_GUC1_PDB_RC_OSC           (1u << 2)  /* active low */
#define AD7779_GUC1_SOFT_RESET_MASK      (0x3u)     /* bits [1:0] */

/* Encodings for POWERMODE bit */
#define AD7779_POWERMODE_LOW_POWER       0u
#define AD7779_POWERMODE_HIGH_RES        1u

/* ---------- GENERAL_USER_CONFIG_2 (0x012) ---------- */
#define AD7779_GUC2_SAR_DIAG_MODE_EN     (1u << 5)
#define AD7779_GUC2_SDO_DRIVE_STR_MASK   (0x3u << 3)
#define AD7779_GUC2_SDO_DRIVE_STR_SHIFT  3
#define AD7779_GUC2_DOUT_DRIVE_STR_MASK  (0x3u << 1)
#define AD7779_GUC2_DOUT_DRIVE_STR_SHIFT 1
#define AD7779_GUC2_SPI_SYNC             (1u << 0)

/* Drive strength enumerations (both SDO and DOUT) */
#define AD7779_DRIVE_NOMINAL             0u
#define AD7779_DRIVE_STRONG              1u
#define AD7779_DRIVE_WEAK                2u
#define AD7779_DRIVE_EXTRA_STRONG        3u

/* ---------- GENERAL_USER_CONFIG_3 (0x013) ---------- */
#define AD7779_GUC3_CONVST_DEGLITCH_DIS_MASK  (0x3u << 6)
#define AD7779_GUC3_CONVST_DEGLITCH_1P5_MCLK  (0x2u << 6)  /* default */
#define AD7779_GUC3_CONVST_DEGLITCH_NONE      (0x3u << 6)
#define AD7779_GUC3_SPI_SLAVE_MODE_EN         (1u << 4)
#define AD7779_GUC3_CLK_QUAL_DIS              (1u << 0)

/* ---------- DOUT_FORMAT (0x014) ---------- */
#define AD7779_DOUTFMT_DOUT_FORMAT_MASK   (0x3u << 6)
#define AD7779_DOUTFMT_DOUT_FORMAT_SHIFT  6
/* 00: 4 DOUT lines, 01: 2 lines, 10/11: 1 line */
#define AD7779_DOUTFMT_HEADER_FORMAT      (1u << 5)  /* 0: status, 1: CRC */
#define AD7779_DOUTFMT_DCLK_CLK_DIV_MASK  (0x7u << 1)
#define AD7779_DOUTFMT_DCLK_CLK_DIV_SHIFT 1

/* ---------- GPIO_CONFIG (0x017) ---------- */
#define AD7779_GPIO_OP_EN_MASK           0x07u   /* bits [2:0]; 0=input, 1=output */

/* ---------- GPIO_DATA (0x018) ---------- */
#define AD7779_GPIO_READ_DATA_MASK       (0x7u << 3)
#define AD7779_GPIO_READ_DATA_SHIFT      3
#define AD7779_GPIO_WRITE_DATA_MASK      0x07u

/* ---------- BUFFER_CONFIG_1 (0x019) ---------- */
#define AD7779_BUF1_REF_BUF_POS_EN       (1u << 4)
#define AD7779_BUF1_REF_BUF_NEG_EN       (1u << 3)

/* ---------- BUFFER_CONFIG_2 (0x01A) ---------- */
#define AD7779_BUF2_REFBUFP_PREQ         (1u << 7)
#define AD7779_BUF2_REFBUFN_PREQ         (1u << 6)
#define AD7779_BUF2_PDB_ALDO1_OVRDRV     (1u << 2)
#define AD7779_BUF2_PDB_ALDO2_OVRDRV     (1u << 1)
#define AD7779_BUF2_PDB_DLDO_OVRDRV      (1u << 0)

/* ---------- CHx_CONFIG (0x000+ch) – gain bits only for now ---------- */
#define AD7779_CHx_CONFIG_GAIN_MASK      (0x3u << 6)
#define AD7779_CHx_CONFIG_GAIN_SHIFT     6
#define AD7779_CHx_GAIN_1                0u
#define AD7779_CHx_GAIN_2                1u
#define AD7779_CHx_GAIN_4                2u
#define AD7779_CHx_GAIN_8                3u

/* ============================================================
 * Low-level SPI access API
 * ============================================================
 */

/**
 * Read one 8-bit register.
 */
esp_err_t ad7779_reg_read8(ad7779_handle_t dev, uint16_t reg, uint8_t *value);

/**
 * Write one 8-bit register.
 */
esp_err_t ad7779_reg_write8(ad7779_handle_t dev, uint16_t reg, uint8_t value);

/**
 * Read-modify-write helper: (reg = (reg & ~mask) | (value & mask))
 */
esp_err_t ad7779_reg_update_bits(ad7779_handle_t dev,
                                 uint16_t reg,
                                 uint8_t mask,
                                 uint8_t value);

/* ============================================================
 * Small convenience helpers
 * ============================================================
 */

/* Perform a software reset via GENERAL_USER_CONFIG_1[1:0] (SOFT_RESET bits). */
esp_err_t ad7779_soft_reset(ad7779_handle_t dev);

/* Set POWERMODE (high-resolution or low-power). */
esp_err_t ad7779_set_powermode(ad7779_handle_t dev, bool high_resolution);

/* Configure DOUT format (1, 2, or 4 data lines) and DCLK divider. */
esp_err_t ad7779_set_dout_format(ad7779_handle_t dev,
                                 uint8_t dout_format,   /* 0..3 (00,01,10,11) */
                                 uint8_t dclk_div);     /* 0..7, see Table 39 */

#endif /* AD7779_H */