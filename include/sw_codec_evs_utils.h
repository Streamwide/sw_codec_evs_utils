/**
 *
 * @category   Streamwide
 * @copyright  Copyright (c) 2021 StreamWIDE SA
 * @author     Pierre Bodilis <pbodilis@streamwide.com>
 * @version    2.6
 *
 * MIT License
 *
 * Copyright (c) 2022 STREAMWIDE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef SW_CODEC_EVS_UTILS_H
#define SW_CODEC_EVS_UTILS_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SW_BITS_TO_BYTES(a) (((a) + 7) / 8)

/* See RFC 4867 section 5.3 */
struct sw_codec_amr_toc {
    uint8_t padding : 2; /* padding */
    uint8_t Q : 1;       /* Frame quality indicator */
    uint8_t FT : 4;      /* Frame type index */
    uint8_t F : 1;       /* If set to 1, indicates that this frame is followed by
                            another speech frame in this payload; if set to 0, indicates
                            that this frame is the last frame in this payload */
} __attribute__((packed, aligned(1)));

/* 3GPP TS 26.445 A.2.2.1.1 */
struct sw_codec_evs_cmr {
    uint8_t frame_type : 4;  // D the requested frametype
    uint8_t type : 3;        // T type of request
    uint8_t header_type : 1; // H
} __attribute__((packed));

/* 3GPP TS 26.445 A.2.2.1.2 */
struct sw_codec_evs_toc {
    uint8_t frame_type : 4;  /* EVS frametype (~FT in amr toc) */
    uint8_t quality : 1;     /* AMRWB Q bit */
    uint8_t is_amrwb : 1;    /* EVS mode bit */
    uint8_t followed : 1;    /* F */
    uint8_t header_type : 1; /* T */
} __attribute__((packed));

struct sw_codec_evs_mode {
    int     mode;
    ssize_t rate;
    ssize_t bytes;
    ssize_t bits;
    int     codec_mode; // MODE1 or MODE2
};

extern struct sw_codec_evs_mode sw_codec_amrwb_modes[];
extern struct sw_codec_evs_mode sw_codec_evs_modes[];

enum {
    SW_AMR_TOC_CMR_OA_BLEN = 8, /**< Octet Align length in bits */
    SW_AMR_TOC_CMR_BE_BLEN = 4, /**< Bandwidth Efficiency length in bits */

    SW_AMR_TOC_ENTRY_OA_BLEN = 8, /**< Octet Align length in bits */
    SW_AMR_TOC_ENTRY_BE_BLEN = 6, /**< Bandwidth Efficiency length in bits */

    SW_CODEC_AMRWB_FRAME_MAX_LEN = 60,

    SW_CODEC_AMRWB_NUMBER_OF_MODES = 16,
    SW_CODEC_EVS_NUMBER_OF_MODES   = 16,

    SW_CODEC_AMR_NO_DATA       = 0xf,
    SW_CODEC_EVS_NO_DATA       = 0xf,
    SW_CODEC_EVS_CMR_NO_REQ    = 0xf,
    SW_CODEC_EVS_FRAME_MAX_LEN = (2560 + 7) / 8,
};

struct sw_codec_evs_mode* sw_codec_evs_get_mode_from_toc(const struct sw_codec_evs_toc* toc);

/**
 * find closest mode to the provided rate if it doesn't match exactly a rate explicitely supported by the provided modes
 *
 * @param modes the modes to look into
 * @param modes_len modes length
 * @param rate the rate in bps
 * @return int the mode value, -1 if nothing could be found
 */
int sw_codec_evs_find_mode_by_rate(const struct sw_codec_evs_mode modes[], size_t modes_len, unsigned rate);

/**
 * find the mode the provided framesize in bits
 *
 * @param modes the modes to look into
 * @param modes_len modes length
 * @param bits the frame size in bits
 * @return int the mode value, -1 if nothing could be found
 */
int sw_codec_evs_find_mode_by_framesize_in_bits(const struct sw_codec_evs_mode modes[], size_t modes_len, unsigned bits);

const struct sw_codec_evs_mode* sw_codec_evs_get_mode(const struct sw_codec_evs_mode* modes, size_t modes_len, int mode);

// encoder
struct sw_codec_evs_encoder_params {
    bool     is_amrwb_rfc4867;
    unsigned sample_rate;
    unsigned bitrate;
    bool     is_dtx_enabled;
};

typedef struct sw_codec_evs_encoder sw_codec_evs_encoder_t;

sw_codec_evs_encoder_t* sw_codec_evs_encoder_create(const struct sw_codec_evs_encoder_params* params);
void                    sw_codec_evs_encoder_destroy(sw_codec_evs_encoder_t* encoder);
unsigned                sw_codec_evs_encoder_get_sample_rate(const sw_codec_evs_encoder_t* encoder);
size_t                  sw_codec_evs_encoder_get_samples_per_frame(const sw_codec_evs_encoder_t* encoder);
uint8_t                 sw_codec_evs_encoder_get_mode(const sw_codec_evs_encoder_t* encoder);
size_t                  sw_codec_evs_encoder_get_bitrate(const sw_codec_evs_encoder_t* encoder);
bool                    sw_codec_evs_encoder_set_bitrate(sw_codec_evs_encoder_t* encoder, unsigned bitrate);

enum {
    SW_CODEC_EVS_ENCODE_ERROR_INVALID_MODE    = -1,
    SW_CODEC_EVS_ENCODE_ERROR_NOT_ENOUGH_DATA = -2,
};
ssize_t        sw_codec_evs_encoder_encode(sw_codec_evs_encoder_t* encoder, uint16_t* pcm, size_t pcm_len);
const uint8_t* sw_codec_evs_encoder_get_encoded_data(const sw_codec_evs_encoder_t* encoder);
size_t         sw_codec_evs_encoder_get_encoded_len(const sw_codec_evs_encoder_t* encoder);

// decoder
struct sw_codec_evs_decoder_params {
    bool     is_amrwb_rfc4867;
    unsigned sample_rate;
    bool     is_dtx_enabled;
};

typedef struct sw_codec_evs_decoder sw_codec_evs_decoder_t;
sw_codec_evs_decoder_t*             sw_codec_evs_decoder_create(const struct sw_codec_evs_decoder_params* params);
void                                sw_codec_evs_decoder_destroy(sw_codec_evs_decoder_t* decoder);
unsigned                            sw_codec_evs_decoder_get_sample_rate(const sw_codec_evs_decoder_t* decoder);
size_t                              sw_codec_evs_decoder_get_samples_per_frame(const sw_codec_evs_decoder_t* decoder);

enum {
    SW_CODEC_EVS_DECODE_ERROR_INVALID_DATA = -1,
    SW_CODEC_EVS_DECODE_ERROR_UNKNOWN      = -99,
};
ssize_t         sw_codec_evs_decoder_decode(sw_codec_evs_decoder_t* decoder, uint8_t* data, size_t len);
const uint16_t* sw_codec_evs_decoder_get_decoded_data(const sw_codec_evs_decoder_t* decoder);
size_t          sw_codec_evs_decoder_get_decoded_len(const sw_codec_evs_decoder_t* decoder);

#ifdef __cplusplus
};
#endif

#endif // SW_CODEC_EVS_UTILS_H
