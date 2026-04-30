/**
 * @file scl3300_regs.h
 * @brief SCL3300-D01 SPI frame opcodes (datasheet §5.1.4 Table 15).
 *
 * The SCL3300 protocol does NOT expose a register-address abstraction the
 * way the LSM6DSV does. Instead, the datasheet specifies a fixed set of
 * 32-bit "operation" frames, each with its own pre-computed CRC. We mirror
 * those constants here. To send any of them, use scl3300_xfer_frame().
 *
 * If you ever want to construct a frame from scratch, use the
 * SCL3300_BUILD_FRAME macro in scl3300.c — it computes the CRC for you.
 */

#ifndef SCL3300_REGS_H
#define SCL3300_REGS_H

#include <stdint.h>

/* ------------------------------------------------------------------------- */
/*  Pre-computed 32-bit operation frames (Table 15)                          */
/* ------------------------------------------------------------------------- */
#define SCL3300_OP_READ_ACC_X        (0x040000F7u)
#define SCL3300_OP_READ_ACC_Y        (0x080000FDu)
#define SCL3300_OP_READ_ACC_Z        (0x0C0000FBu)
#define SCL3300_OP_READ_STO          (0x100000E9u)
#define SCL3300_OP_READ_TEMP         (0x140000EFu)
#define SCL3300_OP_READ_STATUS       (0x180000E5u)
#define SCL3300_OP_READ_ERR_FLAG1    (0x1C0000E3u)
#define SCL3300_OP_READ_ERR_FLAG2    (0x200000C1u)
#define SCL3300_OP_READ_CMD          (0x340000DFu)
#define SCL3300_OP_READ_ANG_X        (0x240000C7u)
#define SCL3300_OP_READ_ANG_Y        (0x280000CDu)
#define SCL3300_OP_READ_ANG_Z        (0x2C0000CBu)
#define SCL3300_OP_READ_WHOAMI       (0x40000091u)

#define SCL3300_OP_ENABLE_ANGLE      (0xB0001F6Fu)

#define SCL3300_OP_CHANGE_MODE_1     (0xB400001Fu)
#define SCL3300_OP_CHANGE_MODE_2     (0xB4000102u)
#define SCL3300_OP_CHANGE_MODE_3     (0xB4000225u)
#define SCL3300_OP_CHANGE_MODE_4     (0xB4000338u)
#define SCL3300_OP_POWER_DOWN        (0xB400046Bu)
#define SCL3300_OP_WAKE_UP           (0xB400001Fu)
#define SCL3300_OP_SW_RESET          (0xB4002098u)

#define SCL3300_OP_SWITCH_BANK_0     (0xFC000073u)
#define SCL3300_OP_SWITCH_BANK_1     (0xFC00016Eu)

/* ------------------------------------------------------------------------- */
/*  Frame field extraction helpers                                           */
/* ------------------------------------------------------------------------- */
/* MISO frame layout (datasheet §5.1.3 Figure 14):
 *   bits [31:26]  OP (RW + ADDR), echoed
 *   bits [25:24]  RS (return status)
 *   bits [23:8]   DATA (16 bits)
 *   bits [7:0]    CRC
 */
#define SCL3300_FRAME_RS(f)    (((f) >> 24) & 0x03u)
#define SCL3300_FRAME_DATA(f)  (((f) >> 8)  & 0xFFFFu)
#define SCL3300_FRAME_CRC(f)   ((f) & 0xFFu)

#define SCL3300_RS_STARTUP     (0x0u)  /* 00b: startup in progress           */
#define SCL3300_RS_NORMAL      (0x1u)  /* 01b: normal, no flags              */
#define SCL3300_RS_RESERVED    (0x2u)  /* 10b: reserved                      */
#define SCL3300_RS_ERROR       (0x3u)  /* 11b: error                         */

#define SCL3300_WHOAMI_VALUE   (0xC1u)

#endif /* SCL3300_REGS_H */
