/**
 * @file ad7779.h
 * @brief AD7779 8-channel Σ-Δ ADC driver — public API.
 *
 * Usage outline:
 *
 *     ad7779_t adc;
 *     ad7779_config_t cfg = AD7779_DEFAULT_CONFIG;  // 1 kSPS, gain x1, internal ref
 *     ad7779_init(&adc, &hal, &cfg);
 *
 *     ad7779_set_sample_callback(&adc, on_samples, ctx);
 *     ad7779_start_streaming(&adc);
 *
 *     // ...your callback receives 8 channels of int32_t per DRDY
 *
 *     ad7779_stop_streaming(&adc);
 */

#ifndef AD7779_H_
#define AD7779_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ad7779_hal.h"
#include "ad7779_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AD7779_OK             = 0,
    AD7779_ERR_PARAM      = -1,
    AD7779_ERR_BUS        = -2,
    AD7779_ERR_TIMEOUT    = -3,
    AD7779_ERR_VERIFY     = -4,    /* register read-back mismatch */
    AD7779_ERR_NOT_READY  = -5,
    AD7779_ERR_STATE      = -6,    /* called in wrong state */
    AD7779_ERR_DEVICE     = -7,    /* AD7779 reported an error */
} ad7779_status_t;

typedef enum {
    AD7779_GAIN_1 = 0,
    AD7779_GAIN_2 = 1,
    AD7779_GAIN_4 = 2,
    AD7779_GAIN_8 = 3,
} ad7779_gain_t;

typedef enum {
    AD7779_PWR_HIGH_RES = 0,   /* fmod = MCLK/4, ODR up to 16 kSPS */
    AD7779_PWR_LOW_POWER = 1,  /* fmod = MCLK/8, ODR up to 8 kSPS */
} ad7779_power_mode_t;

typedef enum {
    AD7779_REF_EXTERNAL = 0,   /* REFx+/REFx- (e.g. external 2.5 V) */
    AD7779_REF_INTERNAL = 1,   /* on-chip 2.5 V */
    AD7779_REF_AVDD     = 2,
} ad7779_reference_t;

/** Driver configuration. */
typedef struct {
    /** External MCLK frequency in Hz (your board: 8 192 000). */
    uint32_t        mclk_hz;

    /** Target output data rate per channel, in Hz (e.g. 1000). */
    uint32_t        odr_hz;

    /** Power / resolution mode. */
    ad7779_power_mode_t power_mode;

    /** Reference source. */
    ad7779_reference_t  reference;

    /** Per-channel PGA gain. */
    ad7779_gain_t   gain[AD7779_NUM_CHANNELS];

    /** Bitmask: bit n = 1 enables channel n. 0xFF = all enabled. */
    uint8_t         channels_enabled;

    /** If true, verify every register write by read-back. */
    bool            verify_writes;

    /** If true, enable SPI CRC validation on register accesses. */
    bool            use_crc;
} ad7779_config_t;

#define AD7779_DEFAULT_CONFIG                            \
    ((ad7779_config_t){                                  \
        .mclk_hz          = 8192000U,                    \
        .odr_hz           = 1000U,                       \
        .power_mode       = AD7779_PWR_HIGH_RES,         \
        .reference        = AD7779_REF_INTERNAL,         \
        .gain             = { AD7779_GAIN_1, AD7779_GAIN_1,    \
                              AD7779_GAIN_1, AD7779_GAIN_1,    \
                              AD7779_GAIN_1, AD7779_GAIN_1,    \
                              AD7779_GAIN_1, AD7779_GAIN_1 },  \
        .channels_enabled = 0xFFU,                       \
        .verify_writes    = true,                        \
        .use_crc          = true,                        \
    })

/**
 * Per-frame callback. Called once per DRDY (i.e. at ODR rate),
 * normally from a low-priority task — NOT from ISR.
 *
 * @param ctx       User context passed to ad7779_set_sample_callback.
 * @param samples   Array of AD7779_NUM_CHANNELS sign-extended 24-bit
 *                  samples (held in int32_t).
 * @param status    Aggregated header bits (alert / saturation flags),
 *                  ORed across all channels in the frame. 0 = clean.
 * @param frame_idx Monotonic frame counter since streaming started.
 */
typedef void (*ad7779_sample_cb_t)(void *ctx,
                                   const int32_t *samples,
                                   uint8_t status,
                                   uint32_t frame_idx);

typedef struct ad7779_s {
    ad7779_hal_t        *hal;
    ad7779_config_t      cfg;
    ad7779_sample_cb_t   cb;
    void                *cb_ctx;

    /** True if SPI CRC is currently enabled at the chip side.
     *  When true, register accesses use and validate 24-bit CRC frames.
     *  When false, register accesses use 16-bit non-CRC frames. */
    bool                 crc_enabled;

    /* Double buffer for DMA-friendly streaming. */
    uint8_t              rx_buf[2][AD7779_FRAME_BYTES_TOTAL];
    volatile uint8_t     active_buf;       /* index currently in flight */
    volatile uint32_t    frame_idx;
    volatile uint32_t    frames_dropped;   /* DRDY while busy */
    volatile bool        streaming;
    volatile bool        xfer_in_flight;
} ad7779_t;

/* ===========================================================
 * Lifecycle
 * =========================================================== */

ad7779_status_t ad7779_init(ad7779_t *dev,
                            ad7779_hal_t *hal,
                            const ad7779_config_t *cfg);

ad7779_status_t ad7779_deinit(ad7779_t *dev);

/** Soft reset via SPI (pulls SDI high for 64 SCLKs). */
ad7779_status_t ad7779_soft_reset(ad7779_t *dev);

/* ===========================================================
 * Register access (low level — exported for advanced use)
 * =========================================================== */

ad7779_status_t ad7779_reg_read(ad7779_t *dev, uint8_t addr, uint8_t *val);
ad7779_status_t ad7779_reg_write(ad7779_t *dev, uint8_t addr, uint8_t val);
ad7779_status_t ad7779_reg_update(ad7779_t *dev, uint8_t addr,
                                  uint8_t mask, uint8_t val);

/* ===========================================================
 * Configuration helpers
 * =========================================================== */

ad7779_status_t ad7779_set_channel_gain(ad7779_t *dev, uint8_t ch,
                                        ad7779_gain_t gain);
ad7779_status_t ad7779_set_channel_gain_writeonly(ad7779_t *dev, uint8_t ch,
                                                  ad7779_gain_t gain);
ad7779_status_t ad7779_set_channel_enable(ad7779_t *dev, uint8_t ch,
                                          bool enable);
ad7779_status_t ad7779_set_odr(ad7779_t *dev, uint32_t odr_hz);
ad7779_status_t ad7779_set_odr_writeonly(ad7779_t *dev, uint32_t odr_hz);
ad7779_status_t ad7779_set_reference(ad7779_t *dev, ad7779_reference_t ref);

/* ===========================================================
 * Diagnostics
 * =========================================================== */

ad7779_status_t ad7779_read_errors(ad7779_t *dev, uint8_t *err_reg_1,
                                   uint8_t *err_reg_2);
ad7779_status_t ad7779_clear_errors(ad7779_t *dev);

/* ===========================================================
 * Streaming
 * =========================================================== */

void ad7779_set_sample_callback(ad7779_t *dev,
                                ad7779_sample_cb_t cb,
                                void *ctx);

ad7779_status_t ad7779_start_streaming(ad7779_t *dev);
ad7779_status_t ad7779_stop_streaming(ad7779_t *dev);

/** Process pending frames — call from a task in main loop / RTOS task.
 *  No-op if streaming is event-driven through the ISR path; provided
 *  for polling-only platforms. */
void ad7779_poll(ad7779_t *dev);

/** Frame counter (monotonic since start_streaming). */
uint32_t ad7779_frame_count(const ad7779_t *dev);

/** Frames missed because a previous SPI read was still in flight. */
uint32_t ad7779_frames_dropped(const ad7779_t *dev);

/* ===========================================================
 * Decode helpers
 * =========================================================== */

/** Sign-extend a 24-bit two's-complement word into int32_t. */
static inline int32_t ad7779_s24_to_s32(uint32_t raw24)
{
    return (int32_t)(raw24 & 0x00FFFFFFU) -
           (int32_t)((raw24 & 0x00800000U) << 1);
}

/**
 * Decode one streamed frame (32 raw bytes) into 8 sign-extended samples.
 * @param header_or  If non-NULL, receives the OR of all 8 status headers.
 */
void ad7779_decode_frame(const uint8_t *raw32,
                         int32_t *out_samples_8,
                         uint8_t *header_or);

#ifdef __cplusplus
}
#endif

#endif /* AD7779_H_ */
