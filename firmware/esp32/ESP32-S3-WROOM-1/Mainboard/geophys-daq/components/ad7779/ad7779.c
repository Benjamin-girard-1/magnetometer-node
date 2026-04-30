/**
 * @file ad7779.c
 * @brief AD7779 driver core — platform-independent.
 */

#include "ad7779.h"
#include "ad7779_crc.h"

#include <string.h>

/* -------------------------------------------------------------------- */
/* Internal helpers                                                      */
/* -------------------------------------------------------------------- */

#define CHECK(expr) do { ad7779_status_t _s = (expr);                  \
                         if (_s != AD7779_OK) return _s; } while (0)

#define HAL_CHECK(expr) do {                                           \
        ad7779_hal_status_t _hs = (expr);                              \
        if (_hs != AD7779_HAL_OK) return AD7779_ERR_BUS;               \
    } while (0)

static ad7779_status_t map_hal(ad7779_hal_status_t s)
{
    switch (s) {
    case AD7779_HAL_OK:           return AD7779_OK;
    case AD7779_HAL_ERR_PARAM:    return AD7779_ERR_PARAM;
    case AD7779_HAL_ERR_BUS:      return AD7779_ERR_BUS;
    case AD7779_HAL_ERR_TIMEOUT:  return AD7779_ERR_TIMEOUT;
    default:                      return AD7779_ERR_BUS;
    }
}

/* -------------------------------------------------------------------- */
/* Register access                                                       */
/*                                                                       */
/* We ALWAYS send 24-bit frames (3 bytes per register access). When CRC  */
/* is disabled at the chip, the third byte is ignored on writes and is   */
/* don't-care on reads. When CRC is enabled, the third byte is the CRC.  */
/* This way the same code path works regardless of CRC state, which is   */
/* essential during bring-up because the chip's reset state for CRC      */
/* enable can vary.                                                      */
/*                                                                       */
/* Frame layout (always 24 bits):                                        */
/*   Read:  SDI: [R/W=1|addr(7)] [0x00]   [CRC of bytes 0,1]             */
/*          SDO: [0x20 echo]     [data]   [CRC of bytes 0,1] (if en)     */
/*   Write: SDI: [R/W=0|addr(7)] [data]   [CRC of bytes 0,1]             */
/*          SDO: [0x20 echo]     [0x00]   [CRC]            (if en)       */
/* -------------------------------------------------------------------- */

ad7779_status_t ad7779_reg_read(ad7779_t *dev, uint8_t addr, uint8_t *val)
{
    if (!dev || !val || addr & 0x80U) return AD7779_ERR_PARAM;

    uint8_t tx[3];
    uint8_t rx[3] = { 0 };

    tx[0] = (uint8_t)(AD7779_SPI_READ | (addr & 0x7FU));
    tx[1] = 0x00U;
    tx[2] = ad7779_crc8(tx, 2);     /* harmless if chip ignores it */

    HAL_CHECK(ad7779_hal_spi_xfer(dev->hal, tx, rx, sizeof(tx)));

    if (dev->crc_enabled) {
        /* Chip computes its CRC over [cmd_byte_sent, data_byte_returned].
         * The 0x20 header echo on rx[0] is NOT part of the CRC input. */
        uint8_t crc_input[2] = { tx[0], rx[1] };
        uint8_t expected = ad7779_crc8(crc_input, 2);
        if (expected != rx[2]) return AD7779_ERR_VERIFY;
    }
    *val = rx[1];
    return AD7779_OK;
}

ad7779_status_t ad7779_reg_write(ad7779_t *dev, uint8_t addr, uint8_t val)
{
    if (!dev || addr & 0x80U) return AD7779_ERR_PARAM;

    uint8_t tx[3];
    tx[0] = (uint8_t)(AD7779_SPI_WRITE | (addr & 0x7FU));
    tx[1] = val;
    tx[2] = ad7779_crc8(tx, 2);

    HAL_CHECK(ad7779_hal_spi_xfer(dev->hal, tx, NULL, sizeof(tx)));

    if (dev->cfg.verify_writes) {
        /* Some registers don't read back exactly what was written
         * (read-only bits, self-clearing bits). For now, verify all
         * writes — caller can disable verify_writes for those regs
         * by toggling the cfg field. */
        uint8_t readback = 0;
        CHECK(ad7779_reg_read(dev, addr, &readback));
        if (readback != val) return AD7779_ERR_VERIFY;
    }
    return AD7779_OK;
}

ad7779_status_t ad7779_reg_update(ad7779_t *dev, uint8_t addr,
                                  uint8_t mask, uint8_t val)
{
    uint8_t cur = 0;
    CHECK(ad7779_reg_read(dev, addr, &cur));
    cur = (uint8_t)((cur & ~mask) | (val & mask));
    return ad7779_reg_write(dev, addr, cur);
}

/* -------------------------------------------------------------------- */
/* Soft reset                                                            */
/* -------------------------------------------------------------------- */

ad7779_status_t ad7779_soft_reset(ad7779_t *dev)
{
    if (!dev) return AD7779_ERR_PARAM;

    /* Per datasheet: SDI held high for 64 SCLKs triggers a soft reset. */
    uint8_t tx[8];
    memset(tx, 0xFF, sizeof(tx));
    HAL_CHECK(ad7779_hal_spi_xfer(dev->hal, tx, NULL, sizeof(tx)));

    /* After soft reset the chip returns to its hardware-default CRC state.
     * We don't know what that is, but we can be conservative and assume
     * CRC is disabled for the next few accesses (we still SEND a valid
     * CRC byte; we just don't validate the response). */
    dev->crc_enabled = false;

    /* Datasheet specifies tINIT_RESET ~225 µs at 16 kSPS HR mode.
     * Use a healthy margin. */
    ad7779_hal_delay_ms(dev->hal, 5);
    return AD7779_OK;
}

/* -------------------------------------------------------------------- */
/* SRC programming for desired ODR                                       */
/*                                                                       */
/*   Decimation_total = fmod / ODR                                       */
/*     where fmod = MCLK/4 (HR) or MCLK/8 (LP)                           */
/*   N  = floor(Decimation_total)                                        */
/*   IF = round((Decimation_total - N) * 2^16)                           */
/* -------------------------------------------------------------------- */

static ad7779_status_t program_src(ad7779_t *dev)
{
    uint32_t fmod = (dev->cfg.power_mode == AD7779_PWR_HIGH_RES)
                    ? (dev->cfg.mclk_hz >> 2)   /* /4 */
                    : (dev->cfg.mclk_hz >> 3);  /* /8 */

    if (dev->cfg.odr_hz == 0) return AD7779_ERR_PARAM;

    /* Decimation in fixed-point Q16 to extract integer & fractional parts. */
    uint64_t dec_q16   = ((uint64_t)fmod << 16) / dev->cfg.odr_hz;
    uint32_t dec_int   = (uint32_t)(dec_q16 >> 16);
    uint32_t dec_frac  = (uint32_t)(dec_q16 & 0xFFFFU);

    /* SRC_N is 12 bits. */
    if (dec_int == 0 || dec_int > 0x0FFFU) return AD7779_ERR_PARAM;

    CHECK(ad7779_reg_write(dev, AD7779_REG_SRC_N_MSB,
                           (uint8_t)((dec_int >> 8) & 0x0FU)));
    CHECK(ad7779_reg_write(dev, AD7779_REG_SRC_N_LSB,
                           (uint8_t)(dec_int & 0xFFU)));
    CHECK(ad7779_reg_write(dev, AD7779_REG_SRC_IF_MSB,
                           (uint8_t)((dec_frac >> 8) & 0xFFU)));
    CHECK(ad7779_reg_write(dev, AD7779_REG_SRC_IF_LSB,
                           (uint8_t)(dec_frac & 0xFFU)));

    /* Pulse SRC_LOAD_UPDATE: hold high for ≥2 MCLK, then clear. */
    CHECK(ad7779_reg_write(dev, AD7779_REG_SRC_UPDATE,
                           AD7779_SRC_LOAD_UPDATE));
    ad7779_hal_delay_us(dev->hal, 10);
    CHECK(ad7779_reg_write(dev, AD7779_REG_SRC_UPDATE, 0x00U));

    return AD7779_OK;
}

/* -------------------------------------------------------------------- */
/* Forward declarations for streaming                                    */
/* -------------------------------------------------------------------- */

static void on_drdy_isr(void *user_ctx);
static void on_xfer_done(void *user_ctx, ad7779_hal_status_t status);

/* -------------------------------------------------------------------- */
/* Init / deinit                                                         */
/* -------------------------------------------------------------------- */

ad7779_status_t ad7779_init(ad7779_t *dev,
                            ad7779_hal_t *hal,
                            const ad7779_config_t *cfg)
{
    if (!dev || !hal || !cfg) return AD7779_ERR_PARAM;

    memset(dev, 0, sizeof(*dev));
    dev->hal = hal;
    dev->cfg = *cfg;

    /* HAL bring-up. Caller is expected to have already enabled the
     * external signals controlled by the shift register
     * (LDO_3V3, MCLK_EN, START, CONVST_SAR). */
    if (ad7779_hal_init(hal) != AD7779_HAL_OK) return AD7779_ERR_BUS;

    /* Wait for the clock to settle and INIT_COMPLETE. */
    ad7779_hal_delay_ms(hal, 10);

    /* Soft reset to known state. */
    CHECK(ad7779_soft_reset(dev));

    /* Poll INIT_COMPLETE with 100 ms timeout. */
    uint32_t t0 = ad7779_hal_now_ms(hal);
    uint8_t status3 = 0;
    do {
        if (ad7779_reg_read(dev, AD7779_REG_STATUS_REG_3, &status3) !=
            AD7779_OK) {
            /* The very first reads after reset may return stale data;
             * keep polling instead of bailing. */
        }
        if (status3 & AD7779_STAT3_INIT_COMPLETE) break;
        ad7779_hal_delay_ms(hal, 1);
    } while ((ad7779_hal_now_ms(hal) - t0) < 100U);

    if (!(status3 & AD7779_STAT3_INIT_COMPLETE)) return AD7779_ERR_TIMEOUT;

    /* ----------------------------------------------------------------
     * CRC handling. We always send 24-bit frames; what changes is
     * whether we VALIDATE the third byte. The chip's reset state for
     * SPI_CRC_TEST_EN may vary, so we explicitly set it here.
     *
     * Important sequencing: we have NOT yet validated any CRC, so
     * dev->crc_enabled is still false at this point — meaning reads
     * up to here ignored byte 2. Now we write 0x5A to set/clear the
     * CRC enable bit. The write is sent as 24 bits with a valid CRC
     * regardless of current chip state, so it always lands.
     * ---------------------------------------------------------------- */
    if (dev->cfg.use_crc) {
        CHECK(ad7779_reg_write(dev, AD7779_REG_GEN_ERR_REG_1_EN,
                               AD7779_ERR1_EN_SPI_CRC_TEST));
        dev->crc_enabled = true;   /* now validate CRC on all reads */

        /* Sanity check: after enabling, a read should return a valid
         * CRC. If it doesn't, something is very wrong (wiring noise,
         * silicon variant, etc.) — bail with a clear error. */
        uint8_t check = 0;
        ad7779_status_t cr = ad7779_reg_read(dev, AD7779_REG_GEN_ERR_REG_1_EN,
                                             &check);
        if (cr != AD7779_OK) return cr;
        if (!(check & AD7779_ERR1_EN_SPI_CRC_TEST)) return AD7779_ERR_VERIFY;
    } else {
        CHECK(ad7779_reg_write(dev, AD7779_REG_GEN_ERR_REG_1_EN, 0));
        dev->crc_enabled = false;
    }

    /* Power mode (HR vs LP), keep VCM and oscillator on. */
    {
        uint8_t guc1 = AD7779_GUC1_PDB_VCM | AD7779_GUC1_PDB_RC_OSC;
        if (dev->cfg.power_mode == AD7779_PWR_HIGH_RES)
            guc1 |= AD7779_GUC1_HR_MODE;
        CHECK(ad7779_reg_write(dev, AD7779_REG_GENERAL_USER_CONFIG_1, guc1));
    }

    /* Reference selection. */
    CHECK(ad7779_set_reference(dev, dev->cfg.reference));

    /* Per-channel gain & enable. */
    for (uint8_t ch = 0; ch < AD7779_NUM_CHANNELS; ++ch) {
        CHECK(ad7779_set_channel_gain(dev, ch, dev->cfg.gain[ch]));
    }
    CHECK(ad7779_reg_write(dev, AD7779_REG_CH_DISABLE,
                           (uint8_t)(~dev->cfg.channels_enabled)));

    /* SRC for target ODR. */
    CHECK(program_src(dev));

    /* Set DOUT_FORMAT to status-header mode (not CRC) — must be done
     * BEFORE enabling SPI_SLAVE_MODE_EN, because once that bit is set
     * we can't write registers anymore through SPI. */
    CHECK(ad7779_reg_update(dev, AD7779_REG_DOUT_FORMAT,
                            AD7779_DOUT_HEADER_FORMAT,
                            0));   /* status header, no CRC */

    /* Make sure SAR diag mode is OFF (we want Σ-Δ on SDO, not SAR). */
    CHECK(ad7779_reg_update(dev, AD7779_REG_GENERAL_USER_CONFIG_2,
                            AD7779_GUC2_SAR_DIAG_MODE_EN, 0));

    /* Trigger SYNC via SPI to lock in config (toggle SPI_SYNC bit).
     * Done BEFORE entering slave mode, because afterwards we can't
     * touch registers. */
    CHECK(ad7779_reg_update(dev, AD7779_REG_GENERAL_USER_CONFIG_2,
                            AD7779_GUC2_SPI_SYNC, 0));
    ad7779_hal_delay_us(hal, 10);
    CHECK(ad7779_reg_update(dev, AD7779_REG_GENERAL_USER_CONFIG_2,
                            AD7779_GUC2_SPI_SYNC, AD7779_GUC2_SPI_SYNC));

    /* Allow first conversions to settle. */
    ad7779_hal_delay_ms(hal, 5);

    /* ============================================================
     * FINAL STEP: enable SPI slave readback mode. After this write,
     * the SPI interface is no longer usable for register access —
     * it now carries Σ-Δ streaming data only. We do this LAST so
     * all configuration is in place first.
     *
     * The write itself uses the standard 24-bit + CRC frame, and
     * the chip processes it normally. We do NOT verify by readback
     * (verify_writes is bypassed for this single write) because the
     * subsequent read would return streaming bytes, not register data.
     * ============================================================ */
    {
        /* Read current GUC3, set the bit, write it back without verify. */
        uint8_t guc3_cur = 0;
        CHECK(ad7779_reg_read(dev, AD7779_REG_GENERAL_USER_CONFIG_3, &guc3_cur));
        uint8_t guc3_new = guc3_cur | AD7779_GUC3_SPI_SLAVE_MODE_EN;

        /* Manually write WITHOUT the verify_writes step (we'd lose CRC sync). */
        uint8_t tx[3];
        tx[0] = (uint8_t)(AD7779_SPI_WRITE | (AD7779_REG_GENERAL_USER_CONFIG_3 & 0x7FU));
        tx[1] = guc3_new;
        tx[2] = ad7779_crc8(tx, 2);
        HAL_CHECK(ad7779_hal_spi_xfer(dev->hal, tx, NULL, sizeof(tx)));
    }

    /* Hook DRDY but keep it disabled until start_streaming. */
    if (ad7779_hal_attach_drdy_isr(hal, on_drdy_isr, dev) != AD7779_HAL_OK)
        return AD7779_ERR_BUS;

    return AD7779_OK;
}

ad7779_status_t ad7779_deinit(ad7779_t *dev)
{
    if (!dev) return AD7779_ERR_PARAM;
    (void)ad7779_stop_streaming(dev);
    return map_hal(ad7779_hal_deinit(dev->hal));
}

/* -------------------------------------------------------------------- */
/* Configuration helpers                                                 */
/* -------------------------------------------------------------------- */

ad7779_status_t ad7779_set_channel_gain(ad7779_t *dev, uint8_t ch,
                                        ad7779_gain_t gain)
{
    if (!dev || ch >= AD7779_NUM_CHANNELS) return AD7779_ERR_PARAM;
    return ad7779_reg_update(dev, AD7779_REG_CH_CONFIG(ch),
                             AD7779_CH_CONFIG_GAIN_MSK,
                             (uint8_t)((gain & 0x3U) << AD7779_CH_CONFIG_GAIN_POS));
}

ad7779_status_t ad7779_set_channel_enable(ad7779_t *dev, uint8_t ch,
                                          bool enable)
{
    if (!dev || ch >= AD7779_NUM_CHANNELS) return AD7779_ERR_PARAM;
    /* CH_DISABLE: bit n = 1 *disables* channel n. */
    uint8_t bit = AD7779_CH_DISABLE_BIT(ch);
    return ad7779_reg_update(dev, AD7779_REG_CH_DISABLE,
                             bit, enable ? 0 : bit);
}

ad7779_status_t ad7779_set_odr(ad7779_t *dev, uint32_t odr_hz)
{
    if (!dev || odr_hz == 0) return AD7779_ERR_PARAM;
    dev->cfg.odr_hz = odr_hz;
    return program_src(dev);
}

ad7779_status_t ad7779_set_reference(ad7779_t *dev, ad7779_reference_t ref)
{
    if (!dev) return AD7779_ERR_PARAM;
    uint8_t mux = (uint8_t)((ref & 0x3U) << AD7779_REF_MUX_CTRL_POS);
    CHECK(ad7779_reg_update(dev, AD7779_REG_ADC_MUX_CONFIG,
                            AD7779_REF_MUX_CTRL_MSK, mux));

    /* Enable both reference buffers in precharge mode (default-safe). */
    CHECK(ad7779_reg_update(dev, AD7779_REG_BUFFER_CONFIG_1,
                            AD7779_BC1_REF_BUF_POS_EN |
                            AD7779_BC1_REF_BUF_NEG_EN,
                            AD7779_BC1_REF_BUF_POS_EN |
                            AD7779_BC1_REF_BUF_NEG_EN));
    CHECK(ad7779_reg_update(dev, AD7779_REG_BUFFER_CONFIG_2,
                            AD7779_BC2_REFBUFP_PRECHARGE |
                            AD7779_BC2_REFBUFN_PRECHARGE,
                            AD7779_BC2_REFBUFP_PRECHARGE |
                            AD7779_BC2_REFBUFN_PRECHARGE));
    dev->cfg.reference = ref;
    return AD7779_OK;
}

/* -------------------------------------------------------------------- */
/* Diagnostics                                                           */
/* -------------------------------------------------------------------- */

ad7779_status_t ad7779_read_errors(ad7779_t *dev, uint8_t *err1, uint8_t *err2)
{
    if (!dev) return AD7779_ERR_PARAM;
    uint8_t e1 = 0, e2 = 0;
    CHECK(ad7779_reg_read(dev, AD7779_REG_GEN_ERR_REG_1, &e1));
    CHECK(ad7779_reg_read(dev, AD7779_REG_GEN_ERR_REG_2, &e2));
    if (err1) *err1 = e1;
    if (err2) *err2 = e2;
    return AD7779_OK;
}

ad7779_status_t ad7779_clear_errors(ad7779_t *dev)
{
    /* SPI errors clear on read of GEN_ERR_REG_1. */
    uint8_t dummy;
    return ad7779_read_errors(dev, &dummy, &dummy);
}

/* -------------------------------------------------------------------- */
/* Streaming                                                             */
/* -------------------------------------------------------------------- */

void ad7779_set_sample_callback(ad7779_t *dev,
                                ad7779_sample_cb_t cb,
                                void *ctx)
{
    if (!dev) return;
    dev->cb = cb;
    dev->cb_ctx = ctx;
}

ad7779_status_t ad7779_start_streaming(ad7779_t *dev)
{
    if (!dev) return AD7779_ERR_PARAM;
    if (dev->streaming) return AD7779_OK;

    dev->frame_idx       = 0;
    dev->frames_dropped  = 0;
    dev->active_buf      = 0;
    dev->xfer_in_flight  = false;
    dev->streaming       = true;

    return map_hal(ad7779_hal_drdy_enable(dev->hal, true));
}

ad7779_status_t ad7779_stop_streaming(ad7779_t *dev)
{
    if (!dev) return AD7779_ERR_PARAM;
    dev->streaming = false;
    return map_hal(ad7779_hal_drdy_enable(dev->hal, false));
}

uint32_t ad7779_frame_count(const ad7779_t *dev)
{
    return dev ? dev->frame_idx : 0U;
}

uint32_t ad7779_frames_dropped(const ad7779_t *dev)
{
    return dev ? dev->frames_dropped : 0U;
}

void ad7779_poll(ad7779_t *dev) { (void)dev; }

/* -------------------------------------------------------------------- */
/* DRDY ISR — triggers async SPI read of one full frame.                 */
/*                                                                       */
/* We keep CS low only for the duration of the 32-byte transfer; the     */
/* HAL is responsible for asserting CS at the start of the transfer and  */
/* releasing it at the end.                                              */
/* -------------------------------------------------------------------- */

static void on_drdy_isr(void *user_ctx)
{
    ad7779_t *dev = (ad7779_t *)user_ctx;
    if (!dev->streaming) return;

    if (dev->xfer_in_flight) {
        dev->frames_dropped++;
        return;
    }

    dev->xfer_in_flight = true;
    uint8_t *buf = dev->rx_buf[dev->active_buf];

    /* Note: we don't pre-fill TX with the NOP pattern here because
     * (a) the HAL-level read_frame_async is a "read" that internally
     * drives MOSI with the NOP bytes 0x80, 0x00, 0x80, 0x00...
     * to avoid spurious register writes. See HAL implementation. */
    (void)ad7779_hal_spi_read_frame_async(dev->hal, buf,
                                          AD7779_FRAME_BYTES_TOTAL,
                                          on_xfer_done, dev);
}

/* -------------------------------------------------------------------- */
/* Frame decode                                                          */
/*                                                                       */
/* Each channel arrives as 4 bytes:                                      */
/*   byte 0 = status header (alert | CH_ID[2:0] | RESET | MOD_SAT |      */
/*            FILT_SAT | AIN_OV_UV)                                      */
/*   bytes 1..3 = 24-bit two's-complement sample (MSB first)             */
/* Channels arrive in order 0..7 starting from the DRDY edge.            */
/* -------------------------------------------------------------------- */

void ad7779_decode_frame(const uint8_t *raw32,
                         int32_t *out_samples_8,
                         uint8_t *header_or)
{
    uint8_t hdr_or = 0;
    for (uint8_t ch = 0; ch < AD7779_NUM_CHANNELS; ++ch) {
        const uint8_t *p = &raw32[ch * AD7779_FRAME_BYTES_PER_CH];
        hdr_or |= p[0];
        uint32_t raw24 = ((uint32_t)p[1] << 16) |
                         ((uint32_t)p[2] << 8)  |
                          (uint32_t)p[3];
        out_samples_8[ch] = ad7779_s24_to_s32(raw24);
    }
    if (header_or) *header_or = hdr_or;
}

static void on_xfer_done(void *user_ctx, ad7779_hal_status_t status)
{
    ad7779_t *dev = (ad7779_t *)user_ctx;
    uint8_t completed = dev->active_buf;
    dev->active_buf = (uint8_t)(completed ^ 1U);
    dev->xfer_in_flight = false;

    if (status != AD7779_HAL_OK) return;

    /* TEMP DEBUG: print raw frame bytes for first few frames */
    if (dev->frame_idx < 3) {
        uint8_t *r = dev->rx_buf[completed];
        extern int printf(const char *, ...);
        printf("frame %lu raw: ", (unsigned long)dev->frame_idx);
        for (int i = 0; i < 32; ++i) printf("%02X ", r[i]);
        printf("\n");
    }

    if (dev->cb) {
        int32_t samples[AD7779_NUM_CHANNELS];
        uint8_t hdr;
        ad7779_decode_frame(dev->rx_buf[completed], samples, &hdr);
        dev->cb(dev->cb_ctx, samples, hdr, dev->frame_idx);
    }
    dev->frame_idx++;
}