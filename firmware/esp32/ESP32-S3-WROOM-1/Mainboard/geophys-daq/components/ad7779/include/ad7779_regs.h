/**
 * @file ad7779_regs.h
 * @brief AD7779 register map and bit definitions.
 *
 * Source: Analog Devices AD7779 datasheet, Rev. E. Register addresses
 * are 7-bit; the 8th bit of the SPI command byte is R/W (0 = write,
 * 1 = read).
 */

#ifndef AD7779_REGS_H_
#define AD7779_REGS_H_

#include <stdint.h>

/* SPI command byte: R/W bit is the MSB of address byte. */
#define AD7779_SPI_READ       0x80U
#define AD7779_SPI_WRITE      0x00U

/* ---- Per-channel config (0x00..0x07) -------------------------------- */
#define AD7779_REG_CH_CONFIG(n)        (0x00U + (n))      /* n = 0..7 */
#  define AD7779_CH_CONFIG_GAIN_POS        6
#  define AD7779_CH_CONFIG_GAIN_MSK        (0x3U << 6)
#  define AD7779_CH_CONFIG_RX              (1U << 4)      /* ref monitor */

/* ---- Channel disable / power-down ----------------------------------- */
#define AD7779_REG_CH_DISABLE          0x08U
#  define AD7779_CH_DISABLE_BIT(n)         (1U << (n))

/* ---- Per-channel sync offset (0x09..0x10) --------------------------- */
#define AD7779_REG_CH_SYNC_OFFSET(n)   (0x09U + (n))

/* ---- General user config ------------------------------------------- */
#define AD7779_REG_GENERAL_USER_CONFIG_1   0x11U
#  define AD7779_GUC1_HR_MODE              (1U << 6)      /* 1 = HR, 0 = LP */
#  define AD7779_GUC1_PDB_VCM              (1U << 5)      /* active low */
#  define AD7779_GUC1_PDB_REFOUT_BUF       (1U << 4)
#  define AD7779_GUC1_PDB_SAR              (1U << 3)
#  define AD7779_GUC1_PDB_RC_OSC           (1U << 2)
#  define AD7779_GUC1_SOFT_RESET_MSK       0x03U          /* needs 11b then 10b */

#define AD7779_REG_GENERAL_USER_CONFIG_2   0x12U
#  define AD7779_GUC2_SAR_DIAG_MODE_EN     (1U << 5)
#  define AD7779_GUC2_SDO_DRIVE_STR_POS    3
#  define AD7779_GUC2_SDO_DRIVE_STR_MSK    (0x3U << 3)
#  define AD7779_GUC2_DOUT_DRIVE_STR_POS   1
#  define AD7779_GUC2_DOUT_DRIVE_STR_MSK   (0x3U << 1)
#  define AD7779_GUC2_SPI_SYNC             (1U << 0)

#define AD7779_REG_GENERAL_USER_CONFIG_3   0x13U
#  define AD7779_GUC3_CONVST_DEGLITCH_MSK  (0x3U << 6)
#  define AD7779_GUC3_SPI_SLAVE_MODE_EN    (1U << 4)      /* read SD data on SDO */
#  define AD7779_GUC3_CLK_QUAL_DIS         (1U << 0)

/* ---- Data output format -------------------------------------------- */
#define AD7779_REG_DOUT_FORMAT             0x14U
#  define AD7779_DOUT_FORMAT_POS           6
#  define AD7779_DOUT_FORMAT_MSK           (0x3U << 6)
#  define AD7779_DOUT_HEADER_FORMAT        (1U << 5)      /* 1 = CRC, 0 = status */
#  define AD7779_DCLK_CLK_DIV_POS          1
#  define AD7779_DCLK_CLK_DIV_MSK          (0x7U << 1)

/* ---- ADC mux / reference ------------------------------------------- */
#define AD7779_REG_ADC_MUX_CONFIG          0x15U
#  define AD7779_REF_MUX_CTRL_POS          6
#  define AD7779_REF_MUX_CTRL_MSK          (0x3U << 6)

/* ---- Buffer config -------------------------------------------------- */
#define AD7779_REG_BUFFER_CONFIG_1         0x19U
#  define AD7779_BC1_REF_BUF_POS_EN        (1U << 4)
#  define AD7779_BC1_REF_BUF_NEG_EN        (1U << 3)

#define AD7779_REG_BUFFER_CONFIG_2         0x1AU
#  define AD7779_BC2_REFBUFP_PRECHARGE     (1U << 7)
#  define AD7779_BC2_REFBUFN_PRECHARGE     (1U << 6)

/* ---- Per-channel offset / gain calibration -------------------------- */
#define AD7779_REG_CH_OFFSET_UPPER(n)      (0x1CU + 6U * (n))
#define AD7779_REG_CH_OFFSET_MID(n)        (0x1DU + 6U * (n))
#define AD7779_REG_CH_OFFSET_LOWER(n)      (0x1EU + 6U * (n))
#define AD7779_REG_CH_GAIN_UPPER(n)        (0x1FU + 6U * (n))
#define AD7779_REG_CH_GAIN_MID(n)          (0x20U + 6U * (n))
#define AD7779_REG_CH_GAIN_LOWER(n)        (0x21U + 6U * (n))

/* ---- Errors --------------------------------------------------------- */
#define AD7779_REG_GEN_ERR_REG_1           0x59U
#  define AD7779_ERR1_MEMMAP_CRC           (1U << 5)
#  define AD7779_ERR1_ROM_CRC              (1U << 4)
#  define AD7779_ERR1_SPI_CLK_COUNT        (1U << 3)
#  define AD7779_ERR1_SPI_INVALID_READ     (1U << 2)
#  define AD7779_ERR1_SPI_INVALID_WRITE    (1U << 1)
#  define AD7779_ERR1_SPI_CRC              (1U << 0)

#define AD7779_REG_GEN_ERR_REG_1_EN        0x5AU
#  define AD7779_ERR1_EN_SPI_CRC_TEST      (1U << 0)

#define AD7779_REG_GEN_ERR_REG_2           0x5BU
#define AD7779_REG_STATUS_REG_1            0x5DU
#define AD7779_REG_STATUS_REG_2            0x5EU
#define AD7779_REG_STATUS_REG_3            0x5FU
#  define AD7779_STAT3_INIT_COMPLETE       (1U << 4)

/* ---- SRC (sample rate converter) ----------------------------------- */
#define AD7779_REG_SRC_N_MSB               0x60U
#define AD7779_REG_SRC_N_LSB               0x61U
#define AD7779_REG_SRC_IF_MSB              0x62U
#define AD7779_REG_SRC_IF_LSB              0x63U
#define AD7779_REG_SRC_UPDATE              0x64U
#  define AD7779_SRC_LOAD_SOURCE           (1U << 7)
#  define AD7779_SRC_LOAD_UPDATE           (1U << 0)

/* ---- GPIO ----------------------------------------------------------- */
#define AD7779_REG_GPIO_CONFIG             0x16U
#define AD7779_REG_GPIO_DATA               0x17U

/* ---- Useful constants ---------------------------------------------- */
#define AD7779_NUM_CHANNELS         8U
#define AD7779_BITS_PER_SAMPLE      24U
#define AD7779_HEADER_BITS          8U
#define AD7779_FRAME_BYTES_PER_CH   4U                        /* header(1) + data(3) */
#define AD7779_FRAME_BYTES_TOTAL    (AD7779_NUM_CHANNELS * AD7779_FRAME_BYTES_PER_CH) /* 32 */

/* "No-op" SPI write to avoid unwanted register writes while reading
 * SD data on SDO. See datasheet "SPI Software Reset" / Σ-Δ readback. */
#define AD7779_NOP_CMD_HI            0x80U
#define AD7779_NOP_CMD_LO            0x00U

#endif /* AD7779_REGS_H_ */
