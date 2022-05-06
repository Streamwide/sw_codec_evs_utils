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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <sw_codec_evs_utils.h>

#ifdef WITH_SWLOG
#include <sw_log.h>
#else
#define LOGD(...)
#define LOGI(...)
#endif // WITH_SWLOG

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif


//#define EVS_FX 1
#ifdef EVS_FX
#define EVS(a) a##_fx
#include <cnst_fx.h> /* for MAX_BITS_PER_FRAME, etc */
#include <prot_fx.h> /* for amr_wb_enc, etc */
#else
#define EVS(a) a
#include <cnst.h> /* for MAX_BITS_PER_FRAME, etc */
#include <prot.h> /* for amr_wb_enc, etc */
#endif
#include <mime.h>     /* for AMRWB_IOmode2rate, etc */
#include <stat_com.h> /* for FRAMEMODE_NORMAL */
#include <typedef.h>  /* for UWord8, UWord16, Word16 */

// clang-format off

/* For codec_mode, see lib_enc/io_enc.c:io_ini_enc */
// clang-format off
struct sw_codec_evs_mode sw_codec_amrwb_modes[] = {
    {AMRWB_IO_6600,    ACELP_6k60,  SW_BITS_TO_BYTES(ACELP_6k60 / 50),  ACELP_6k60 / 50, MODE1},
    {AMRWB_IO_8850,    ACELP_8k85,  SW_BITS_TO_BYTES(ACELP_8k85 / 50),  ACELP_8k85 / 50, MODE1},
    {AMRWB_IO_1265,   ACELP_12k65, SW_BITS_TO_BYTES(ACELP_12k65 / 50), ACELP_12k65 / 50, MODE1},
    {AMRWB_IO_1425,   ACELP_14k25, SW_BITS_TO_BYTES(ACELP_14k25 / 50), ACELP_14k25 / 50, MODE1},
    {AMRWB_IO_1585,   ACELP_15k85, SW_BITS_TO_BYTES(ACELP_15k85 / 50), ACELP_15k85 / 50, MODE1},
    {AMRWB_IO_1825,   ACELP_18k25, SW_BITS_TO_BYTES(ACELP_18k25 / 50), ACELP_18k25 / 50, MODE1},
    {AMRWB_IO_1985,   ACELP_19k85, SW_BITS_TO_BYTES(ACELP_19k85 / 50), ACELP_19k85 / 50, MODE1},
    {AMRWB_IO_2305,   ACELP_23k05, SW_BITS_TO_BYTES(ACELP_23k05 / 50), ACELP_23k05 / 50, MODE1},
    {AMRWB_IO_2385,   ACELP_23k85, SW_BITS_TO_BYTES(ACELP_23k85 / 50), ACELP_23k85 / 50, MODE1},
    { AMRWB_IO_SID,      SID_1k75,    SW_BITS_TO_BYTES(SID_1k75 / 50),    SID_1k75 / 50, MODE1},
    {   /* 10*/ -1,            -1,                                 -1,               -1,    -1},
    {   /* 11*/ -1,            -1,                                 -1,               -1,    -1},
    {   /* 12*/ -1,            -1,                                 -1,               -1,    -1},
    {   /* 13*/ -1,            -1,                                 -1,               -1,    -1},
    {  SPEECH_LOST,            -1,                                 -1,               -1,    -1},
    {      NO_DATA, FRAME_NO_DATA,                      FRAME_NO_DATA,    FRAME_NO_DATA, MODE1},
};

struct sw_codec_evs_mode sw_codec_evs_modes[] = {
    {  PRIMARY_2800, PPP_NELP_2k80, SW_BITS_TO_BYTES(PPP_NELP_2k80 / 50), PPP_NELP_2k80 / 50, MODE1},
    {  PRIMARY_7200,    ACELP_7k20,    SW_BITS_TO_BYTES(ACELP_7k20 / 50),    ACELP_7k20 / 50, MODE1},
    {  PRIMARY_8000,    ACELP_8k00,    SW_BITS_TO_BYTES(ACELP_8k00 / 50),    ACELP_8k00 / 50, MODE1},
    {  PRIMARY_9600,    ACELP_9k60,    SW_BITS_TO_BYTES(ACELP_9k60 / 50),    ACELP_9k60 / 50, MODE2},
    { PRIMARY_13200,   ACELP_13k20,   SW_BITS_TO_BYTES(ACELP_13k20 / 50),   ACELP_13k20 / 50, MODE1},
    { PRIMARY_16400,   ACELP_16k40,   SW_BITS_TO_BYTES(ACELP_16k40 / 50),   ACELP_16k40 / 50, MODE2},
    { PRIMARY_24400,   ACELP_24k40,   SW_BITS_TO_BYTES(ACELP_24k40 / 50),   ACELP_24k40 / 50, MODE2},
    { PRIMARY_32000,     ACELP_32k,     SW_BITS_TO_BYTES(ACELP_32k / 50),     ACELP_32k / 50, MODE1},
    { PRIMARY_48000,     ACELP_48k,     SW_BITS_TO_BYTES(ACELP_48k / 50),     ACELP_48k / 50, MODE2},
    { PRIMARY_64000,     ACELP_64k,     SW_BITS_TO_BYTES(ACELP_64k / 50),     ACELP_64k / 50, MODE1},
    { PRIMARY_96000,        HQ_96k,        SW_BITS_TO_BYTES(HQ_96k / 50),        HQ_96k / 50, MODE2},
    {PRIMARY_128000,       HQ_128k,       SW_BITS_TO_BYTES(HQ_128k / 50),       HQ_128k / 50, MODE2},
    {   PRIMARY_SID,      SID_2k40,      SW_BITS_TO_BYTES(SID_2k40 / 50),      SID_2k40 / 50, MODE1},
    {    /* 13*/ -1,            -1,                                   -1,                 -1,    -1},
    {   SPEECH_LOST,            -1,                                   -1,                 -1,    -1},
    {       NO_DATA, FRAME_NO_DATA,                        FRAME_NO_DATA,      FRAME_NO_DATA, MODE1},
};

// clang-format on

struct sw_codec_evs_mode* sw_codec_evs_get_mode_from_toc(const struct sw_codec_evs_toc* toc) {
    struct sw_codec_evs_mode* evs_mode = NULL;
    if (toc->is_amrwb) {
        if (sw_codec_amrwb_modes[toc->frame_type].mode > -1) {
            evs_mode = &sw_codec_amrwb_modes[toc->frame_type];
        }
    } else {
        if (sw_codec_evs_modes[toc->frame_type].mode > -1) {
            evs_mode = &sw_codec_evs_modes[toc->frame_type];
        }
    }
    return evs_mode;
}

int sw_codec_evs_find_mode_by_rate(const struct sw_codec_evs_mode modes[], size_t modes_len, unsigned rate) {
    int closest     = -1;
    int closestdiff = 0;
    for (unsigned i = 0; i < modes_len; ++i) {
        if (modes[i].rate < 0) {
            continue;
        }
        if (modes[i].rate == (ssize_t)rate) {
            return modes[i].mode;
        }
        if (closest < 0 || closestdiff > abs((int)modes[i].rate - (int)rate)) {
            closest     = i;
            closestdiff = abs((int)modes[i].rate - (int)rate);
        }
    }
    return closest < 0 ? -1 : modes[closest].mode;
}

int sw_codec_evs_find_mode_by_framesize_in_bits(const struct sw_codec_evs_mode modes[], size_t modes_len, unsigned bits) {
    for (unsigned i = 0; i < modes_len; i++) {
        if (modes[i].rate < 0) {
            continue;
        }
        if (modes[i].bits == (ssize_t)bits) {
            return modes[i].mode;
        }
    }
    return -1;
}

const struct sw_codec_evs_mode* sw_codec_evs_get_mode(const struct sw_codec_evs_mode* modes, size_t modes_len, int mode) {
    if (mode < 0 || (size_t)mode >= modes_len) {
        return NULL;
    }
    if (modes[mode].mode == mode) { // if modes is indexed by the mode value
        return &modes[mode];
    } else { // else, look for the right mode in the given modes
        for (unsigned i = 0; i < modes_len; ++i) {
            if (modes[i].rate < 0) {
                continue;
            }
            if (modes[i].mode == mode) {
                return &modes[i];
            }
        }
        return NULL;
    }
}

/*************************** encoder ***************************/
struct sw_codec_evs_encoder {
    EVS(Encoder_State) st;
    EVS(Indice) ind_list[MAX_NUM_INDICES];

    uint8_t encoded_data[SW_BITS_TO_BYTES(MAX_BITS_PER_FRAME) + sizeof(struct sw_codec_evs_toc)]; // + 1 byte for the TOC
    size_t  encoded_len;

    struct sw_codec_evs_cmr cmr;
    bool                    cmr_set_from_codec_mode_request;

    struct sw_codec_evs_mode* modes;     /* mode set in FMTP */
    size_t                    modes_len; /* encoder_modes length */

    bool is_amrwb_rfc4867;
};

static void                  sw_codec_evs_encoder_configure(struct sw_codec_evs_encoder* encoder, const struct sw_codec_evs_cmr* cmr);
struct sw_codec_evs_encoder* sw_codec_evs_encoder_create(const struct sw_codec_evs_encoder_params* params) {
    struct sw_codec_evs_encoder* encoder = malloc(sizeof(struct sw_codec_evs_encoder));
    if (!encoder) {
        return NULL;
    }
    memset(encoder, 0, sizeof(struct sw_codec_evs_encoder));

    if (params->is_amrwb_rfc4867) {
        encoder->modes     = sw_codec_amrwb_modes;
        encoder->modes_len = SW_CODEC_AMRWB_NUMBER_OF_MODES;
    } else {
        encoder->modes     = sw_codec_evs_modes;
        encoder->modes_len = SW_CODEC_EVS_NUMBER_OF_MODES;
    }

    encoder->st.EVS(ind_list) = encoder->ind_list;
    encoder->st.EVS(input_Fs) = params->sample_rate;
    if (encoder->st.EVS(input_Fs) <= 8000) {
        encoder->st.EVS(max_bwidth) = NB;
        encoder->cmr.type           = 0b000;
    } else if (encoder->st.EVS(input_Fs) <= 16000 || encoder->st.EVS(Opt_SC_VBR)) {
        encoder->st.EVS(max_bwidth) = WB;
        encoder->cmr.type           = 0b010;
    } else if (encoder->st.EVS(input_Fs) <= 32000) {
        encoder->st.EVS(max_bwidth) = SWB;
        encoder->cmr.type           = 0b011;
    } else {
        encoder->st.EVS(max_bwidth) = FB;
        encoder->cmr.type           = 0b100;
    }

#ifdef EVS_FX
    encoder->st.EVS(input_frame) = extract_l(Mult_32_16(encoder->st.EVS(input_Fs), 0x0290));
#endif
    encoder->is_amrwb_rfc4867 = params->is_amrwb_rfc4867;
    if (params->is_amrwb_rfc4867) {
        encoder->cmr.type = 0b001;
    }

    /* Value range:  0..2, see res/res_format_attr_evs.c */
    encoder->st.EVS(Opt_DTX_ON)        = params->is_dtx_enabled;
    encoder->st.EVS(var_SID_rate_flag) = 1; /* Automatic interval */
    encoder->st.EVS(Opt_AMR_WB)        = params->is_amrwb_rfc4867;
    /* Value range: -2..7, see res/res_format_attr_evs.c */
    encoder->st.Opt_RF_ON = 0;

    encoder->st.rf_fec_offset    = 0;
    encoder->st.rf_fec_indicator = 1;

    // Set default frame_type
    sw_codec_evs_encoder_set_bitrate(encoder, params->bitrate);

    encoder->st.last_codec_mode = encoder->st.codec_mode;

    EVS(init_encoder)(&encoder->st);

    sw_codec_evs_encoder_configure(encoder, &encoder->cmr);

    return encoder;
}

void sw_codec_evs_encoder_destroy(struct sw_codec_evs_encoder* encoder) {
    EVS(destroy_encoder)(&encoder->st);
    free(encoder);
}

/* see 3GPP TS 26.445 section A.2.2.1.1 CMR byte */
void sw_codec_evs_encoder_configure(struct sw_codec_evs_encoder* encoder, const struct sw_codec_evs_cmr* cmr) {
    unsigned max_bandwidth = (encoder->st.EVS(input_Fs) / 8000) >> 1;

    encoder->st.EVS(Opt_AMR_WB)  = 0;
    encoder->st.EVS(total_brate) = PRIMARYmode2rate[cmr->frame_type];
    encoder->st.EVS(Opt_SC_VBR)  = 0;
    encoder->st.Opt_RF_ON        = 0;

    switch (cmr->type) {
        case 0b000:
            encoder->st.EVS(max_bwidth) = NB;
            if (!cmr->frame_type) {
                encoder->st.EVS(Opt_SC_VBR)  = 1;
                encoder->st.EVS(total_brate) = PRIMARYmode2rate[PRIMARY_7200];
            }
            break;
        case 0b001:
            encoder->st.EVS(Opt_AMR_WB)  = 1;
            encoder->st.EVS(total_brate) = AMRWB_IOmode2rate[cmr->frame_type];
            break;
        case 0b010:
            encoder->st.EVS(max_bwidth) = MIN(max_bandwidth, WB);
            if (!cmr->frame_type) {
                encoder->st.EVS(Opt_SC_VBR)  = 1;
                encoder->st.EVS(total_brate) = PRIMARYmode2rate[PRIMARY_7200];
            }
            break;
        case 0b011:
            encoder->st.EVS(max_bwidth)  = MIN(max_bandwidth, SWB);
            encoder->st.EVS(total_brate) = PRIMARYmode2rate[cmr->frame_type];
            break;
        case 0b100:
            encoder->st.EVS(max_bwidth)  = MIN(max_bandwidth, FB);
            encoder->st.EVS(total_brate) = PRIMARYmode2rate[cmr->frame_type];
            break;
        case 0b101:
            encoder->st.Opt_RF_ON        = 1;
            encoder->st.EVS(max_bwidth)  = MIN(max_bandwidth, WB);
            encoder->st.EVS(total_brate) = PRIMARYmode2rate[PRIMARY_13200];
            break;
        case 0b110:
            encoder->st.Opt_RF_ON        = 1;
            encoder->st.EVS(max_bwidth)  = MIN(max_bandwidth, SWB);
            encoder->st.EVS(total_brate) = PRIMARYmode2rate[PRIMARY_13200];
            break;
        case 0b111:
            if (SW_CODEC_EVS_CMR_NO_REQ == cmr->frame_type) {
                break;
            }
            break;

        default:
            break;
    }
    if (encoder->st.EVS(total_brate) == 13200 && encoder->st.Opt_RF_ON == 1) {
        encoder->st.codec_mode = MODE2;
    } else {
        if (encoder->st.EVS(Opt_AMR_WB)) {
            encoder->st.codec_mode = MODE1;
        } else {
            const struct sw_codec_evs_mode* evs_mode = sw_codec_evs_get_mode(encoder->modes, encoder->modes_len, encoder->cmr.frame_type);
            if (evs_mode) {
                encoder->st.codec_mode = evs_mode->codec_mode;
            }
        }
    }
    LOGD("Configuring Encoder: %s - max_bwidth: %d - total brate: %ld", encoder->st.EVS(Opt_AMR_WB) ? "AMRWB" : "EVS",
         encoder->st.EVS(max_bwidth), encoder->st.EVS(total_brate));
}

unsigned sw_codec_evs_encoder_get_sample_rate(const sw_codec_evs_encoder_t* encoder) {
    return encoder->st.EVS(input_Fs);
}
size_t sw_codec_evs_encoder_get_bitrate(const sw_codec_evs_encoder_t* encoder) {
    return encoder->st.EVS(total_brate) + 50 * sizeof(struct sw_codec_evs_toc);
}
bool sw_codec_evs_encoder_set_bitrate(struct sw_codec_evs_encoder* encoder, unsigned bitrate) {
    /* if encoder_mode is set by the codec mode request, don't modify it */
    if (encoder->cmr_set_from_codec_mode_request) {
        return false;
    }

    int new_mode = sw_codec_evs_find_mode_by_rate(encoder->modes, encoder->modes_len, bitrate);
    if (encoder->cmr.frame_type == new_mode) { // already set, do nothing!
        return true;
    }
    LOGI("Encoder new mode: %u (was: %u)", new_mode, encoder->cmr.frame_type);
    encoder->cmr.frame_type = new_mode;

    sw_codec_evs_encoder_configure(encoder, (struct sw_codec_evs_cmr*)&encoder->cmr);
    return true;
}

size_t sw_codec_evs_encoder_get_samples_per_frame(const sw_codec_evs_encoder_t* encoder) {
    return (size_t)encoder->st.EVS(input_Fs) / 50;
}

uint8_t sw_codec_evs_encoder_get_mode(const sw_codec_evs_encoder_t* encoder) {
    return sw_codec_evs_find_mode_by_framesize_in_bits(encoder->modes, encoder->modes_len, encoder->st.EVS(nb_bits_tot));
}

ssize_t sw_codec_evs_encoder_encode(sw_codec_evs_encoder_t* encoder, uint16_t* pcm, size_t pcm_len) {
    if (encoder->st.EVS(Opt_AMR_WB)) {
        EVS(amr_wb_enc)(&encoder->st, (Word16*)pcm, (Word16)pcm_len);
    } else {
        EVS(evs_enc)(&encoder->st, (Word16*)pcm, (Word16)pcm_len);
    }
    size_t offset = 0;
    if (encoder->is_amrwb_rfc4867) {
        struct sw_codec_amr_toc toc = {0};
        toc.FT                      = sw_codec_evs_encoder_get_mode(encoder);
        toc.Q                       = 1;

        encoder->encoded_data[0] = *(uint8_t*)(&toc);
        offset += sizeof(struct sw_codec_amr_toc);
    } else {
        struct sw_codec_evs_toc toc = {0};
        toc.is_amrwb                = !!encoder->st.EVS(Opt_AMR_WB);
        toc.quality                 = !!encoder->st.EVS(Opt_AMR_WB);
        toc.frame_type              = sw_codec_evs_encoder_get_mode(encoder);

        encoder->encoded_data[0] = *(uint8_t*)(&toc);
        offset += sizeof(struct sw_codec_evs_toc);
    }

    Word16 outlen = 0;
    indices_to_serial(&encoder->st, encoder->encoded_data + offset, &outlen);
    encoder->encoded_len = sizeof(struct sw_codec_evs_toc) + SW_BITS_TO_BYTES(outlen);

    EVS(reset_indices_enc)(&encoder->st);
    return (ssize_t)encoder->encoded_len;
}

const uint8_t* sw_codec_evs_encoder_get_encoded_data(const sw_codec_evs_encoder_t* encoder) {
    return encoder->encoded_data;
}
size_t sw_codec_evs_encoder_get_encoded_len(const sw_codec_evs_encoder_t* encoder) {
    return encoder->encoded_len;
}

/*************************** decoder ***************************/

struct sw_codec_evs_decoder {
    EVS(Decoder_State) st;
#ifdef EVS_FX
    UWord16 bit_stream[MAX_BITS_PER_FRAME + 16];
#endif

    float  output[48000 / 50];       // max per frame
    Word16 decoded_data[48000 / 50]; // max per frame
    size_t decoded_len;
};

struct sw_codec_evs_decoder* sw_codec_evs_decoder_create(const struct sw_codec_evs_decoder_params* params);
void                         sw_codec_evs_decoder_destroy(struct sw_codec_evs_decoder* decoder);

struct sw_codec_evs_decoder* sw_codec_evs_decoder_create(const struct sw_codec_evs_decoder_params* params) {
    struct sw_codec_evs_decoder* decoder = malloc(sizeof(struct sw_codec_evs_decoder));
    if (!decoder) {
        return NULL;
    }
    memset(decoder, 0, sizeof(struct sw_codec_evs_decoder));

    decoder->st.EVS(output_Fs) = params->sample_rate;
#ifdef EVS_FX
    decoder->st.EVS(bit_stream)   = decoder->bit_stream;
    decoder->st.EVS(output_frame) = extract_l(Mult_32_16(decoder->st.EVS(output_Fs), 0x0290));
#endif

    decoder->st.amrwb_rfc4867_flag = params->is_amrwb_rfc4867;
    decoder->st.bitstreamformat    = VOIP_RTPDUMP;

    EVS(reset_indices_dec)(&decoder->st);
    EVS(init_decoder)(&decoder->st);

    return decoder;
}

void sw_codec_evs_decoder_destroy(struct sw_codec_evs_decoder* decoder) {

    destroy_decoder(&decoder->st);
    free(decoder);
}

unsigned sw_codec_evs_decoder_get_sample_rate(const sw_codec_evs_decoder_t* decoder) {
    return decoder->st.EVS(output_Fs);
}
size_t sw_codec_evs_decoder_get_samples_per_frame(const sw_codec_evs_decoder_t* decoder) {
    return (size_t)decoder->st.EVS(output_Fs) / 50;
}

// this mostly copy/pasted from "read_indices_mime", but the "exit" calls are unacceptable in real life
// so this one checks the header, then calls read_indices_from_djb that does more or less the same
static bool sw_codec_evs_decoder_check(EVS(Decoder_State) * st, uint8_t* data, size_t len) {
    Word16 core_mode   = -1;
    Word32 total_brate = -1;
    bool   is_amrwb    = false;
    bool   qbit        = false;
    size_t offset      = 0;

    if (!len) {
        return false;
    }

    if (st->amrwb_rfc4867_flag) {
        struct sw_codec_amr_toc* toc = (struct sw_codec_amr_toc*)data;
        is_amrwb                     = true;
        core_mode                    = toc->FT;
        qbit                         = toc->Q;
        offset += sizeof(struct sw_codec_amr_toc);
    } else {
        struct sw_codec_evs_toc* toc = (struct sw_codec_evs_toc*)data;
        is_amrwb                     = toc->is_amrwb;
        qbit                         = !toc->is_amrwb || toc->quality;
        core_mode                    = toc->frame_type;
        offset += sizeof(struct sw_codec_evs_toc);

        // the read_indices_mime checks that the header is correctly formed:
        // the followed and header type must be set to 0
        // if this is NOT an amrwb header, then the quality header must be set to O
        if (toc->followed || toc->header_type || (!toc->is_amrwb && toc->quality)) {
            LOGD("Invalid ToC header: 0x%2x", data[0]);
            return false;
        }
    }

    if (is_amrwb) {
        total_brate = AMRWB_IOmode2rate[core_mode];
    } else {
        total_brate = PRIMARYmode2rate[core_mode];
    }
    if (total_brate < 0) {
        LOGD("Invalid bit rate mode in ToC: mode 0x%x (%d)", core_mode, core_mode);
        return false;
    }

    size_t frame_len = SW_BITS_TO_BYTES(total_brate / 50);

    if (len < frame_len + offset) {
        LOGD("Invalid packet %zu < %zu + %zu", len, frame_len, offset)
        return false;
    }
    read_indices_from_djb(st, data + offset, total_brate / 50, is_amrwb, core_mode, qbit, 0, 0);
    return true;
}

ssize_t sw_codec_evs_decoder_decode(sw_codec_evs_decoder_t* decoder, uint8_t* data, size_t len) {
    decoder->decoded_len = sw_codec_evs_decoder_get_samples_per_frame(decoder);

    memset(decoder->output, 0, sizeof(decoder->output));

    if (data) {
        if (!sw_codec_evs_decoder_check(&decoder->st, data, len)) {
            return SW_CODEC_EVS_DECODE_ERROR_INVALID_DATA;
        }
    }

#ifdef EVS_FX
    if (decoder->st.EVS(Opt_AMR_WB)) {
        EVS(amr_wb_dec)((Word16*)decoder->decoded_data, &decoder->st);
    } else {
        EVS(evs_dec)(&decoder->st, (Word16*)decoder->decoded_data, data ? FRAMEMODE_NORMAL : FRAMEMODE_MISSING);
    }
#else
    if (decoder->st.EVS(Opt_AMR_WB)) {
        EVS(amr_wb_dec)(&decoder->st, decoder->output);
    } else {
        EVS(evs_dec)(&decoder->st, decoder->output, data ? FRAMEMODE_NORMAL : FRAMEMODE_MISSING);
    }

    syn_output(decoder->output, decoder->decoded_len, decoder->decoded_data);

#endif
    return decoder->decoded_len;
}

const uint16_t* sw_codec_evs_decoder_get_decoded_data(const sw_codec_evs_decoder_t* decoder) {
    return (const uint16_t*)decoder->decoded_data;
}
size_t sw_codec_evs_decoder_get_decoded_len(const sw_codec_evs_decoder_t* decoder) {
    return decoder->decoded_len;
}
