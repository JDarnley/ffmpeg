/*
 * Copyright (C) 2016 Open Broadcast Systems Ltd.
 * Author        2016 Rostislav Pehlivanov <atomnuker@gmail.com>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"
#include "dirac.h"
#include "put_bits.h"
#include "internal.h"
#include "version.h"

#include "vc2enc_dwt.h"
#include "diractab.h"

/* Total range is -COEF_LUT_TAB to +COEFF_LUT_TAB, but total tab size is half
 * (COEF_LUT_TAB*DIRAC_MAX_QUANT_INDEX), as the sign is appended during encoding */
#define COEF_LUT_TAB 2048

/* The limited size resolution of each slice forces us to do this */
#define SSIZE_ROUND(b) (FFALIGN((b), s->size_scaler) + 4 + s->prefix_bytes)

/* Decides the cutoff point in # of slices to distribute the leftover bytes */
#define SLICE_REDIST_TOTAL 150

typedef struct VC2BaseVideoFormat {
    enum AVPixelFormat pix_fmt;
    AVRational time_base;
    int width, height, interlaced, level;
    const char *name;
} VC2BaseVideoFormat;

static const VC2BaseVideoFormat base_video_fmts[] = {
    { 0 }, /* Custom format, here just to make indexing equal to base_vf */
    { AV_PIX_FMT_YUV420P,   { 1001, 15000 },  176,  120, 0, 1,     "QSIF525" },
    { AV_PIX_FMT_YUV420P,   {    2,    25 },  176,  144, 0, 1,     "QCIF"    },
    { AV_PIX_FMT_YUV420P,   { 1001, 15000 },  352,  240, 0, 1,     "SIF525"  },
    { AV_PIX_FMT_YUV420P,   {    2,    25 },  352,  288, 0, 1,     "CIF"     },
    { AV_PIX_FMT_YUV420P,   { 1001, 15000 },  704,  480, 0, 1,     "4SIF525" },
    { AV_PIX_FMT_YUV420P,   {    2,    25 },  704,  576, 0, 1,     "4CIF"    },

    { AV_PIX_FMT_YUV422P10, { 1001, 30000 },  720,  480, 1, 2,   "SD480I-60" },
    { AV_PIX_FMT_YUV422P10, {    1,    25 },  720,  576, 1, 2,   "SD576I-50" },

    { AV_PIX_FMT_YUV422P10, { 1001, 60000 }, 1280,  720, 0, 3,  "HD720P-60"  },
    { AV_PIX_FMT_YUV422P10, {    1,    50 }, 1280,  720, 0, 3,  "HD720P-50"  },
    { AV_PIX_FMT_YUV422P10, { 1001, 30000 }, 1920, 1080, 1, 3,  "HD1080I-60" },
    { AV_PIX_FMT_YUV422P10, {    1,    25 }, 1920, 1080, 1, 3,  "HD1080I-50" },
    { AV_PIX_FMT_YUV422P10, { 1001, 60000 }, 1920, 1080, 0, 3,  "HD1080P-60" },
    { AV_PIX_FMT_YUV422P10, {    1,    50 }, 1920, 1080, 0, 3,  "HD1080P-50" },

    { AV_PIX_FMT_YUV444P12, {    1,    24 }, 2048, 1080, 0, 4,        "DC2K" },
    { AV_PIX_FMT_YUV444P12, {    1,    24 }, 4096, 2160, 0, 5,        "DC4K" },

    { AV_PIX_FMT_YUV422P10, { 1001, 60000 }, 3840, 2160, 0, 6, "UHDTV 4K-60" },
    { AV_PIX_FMT_YUV422P10, {    1,    50 }, 3840, 2160, 0, 6, "UHDTV 4K-50" },

    { AV_PIX_FMT_YUV422P10, { 1001, 60000 }, 7680, 4320, 0, 7, "UHDTV 8K-60" },
    { AV_PIX_FMT_YUV422P10, {    1,    50 }, 7680, 4320, 0, 7, "UHDTV 8K-50" },

    { AV_PIX_FMT_YUV422P10, { 1001, 24000 }, 1920, 1080, 0, 3,  "HD1080P-24" },
    { AV_PIX_FMT_YUV422P10, { 1001, 30000 },  720,  486, 1, 2,  "SD Pro486"  },
};
static const int base_video_fmts_len = FF_ARRAY_ELEMS(base_video_fmts);

enum VC2_QM {
    VC2_QM_DEF = 0,
    VC2_QM_COL,
    VC2_QM_FLAT,

    VC2_QM_NB
};

typedef struct SubBand {
    dwtcoef *buf;
    ptrdiff_t stride;
    int width;
    int height;
    int hstride;
} SubBand;

typedef struct Plane {
    SubBand band[MAX_DWT_LEVELS][4];
    dwtcoef *coef_buf;
    int width;
    int height;
    int dwt_width;
    int dwt_height;
    ptrdiff_t coef_stride;
} Plane;

typedef struct SliceArgs {
    PutBitContext pb;
    int cache[DIRAC_MAX_QUANT_INDEX];
    void *ctx;
    int x;
    int y;
    int quant_idx;
    int bits_ceil;
    int bits_floor;
    int bytes;
} SliceArgs;

typedef struct TransformArgs {
    void *ctx;
    Plane *plane;
    VC2TransformContext t;
    const AVFrame *frame;
} TransformArgs;

typedef struct VC2EncContext {
    AVClass *av_class;
    PutBitContext pb;
    Plane plane[3];
    AVCodecContext *avctx;
    DiracVersionInfo ver;

    SliceArgs *slice_args;
    TransformArgs transform_args[3];

    /* For conversion from unsigned pixel values to signed */
    int diff_offset;
    int bpp;
    int bpp_idx;

    /* Picture number */
    uint32_t picture_number;

    /* Base video format */
    int base_vf;
    int level;
    int profile;

    /* Quantization matrix */
    uint8_t quant[MAX_DWT_LEVELS][4];
    int custom_quant_matrix;

    /* Coefficient LUT */
    uint32_t *coef_lut_val;
    uint8_t  *coef_lut_len;

    int num_x; /* #slices horizontally */
    int num_y; /* #slices vertically */
    int num_y_partial;
    int prefix_bytes;
    int size_scaler;
    int chroma_x_shift;
    int chroma_y_shift;

    /* Rate control stuff */
    int frame_max_bytes;
    int slice_max_bytes;
    int slice_min_bytes;
    int q_ceil;
    int q_avg;
    int64_t slice_count;

    /* Options */
    double tolerance;
    int wavelet_idx;
    int wavelet_depth;
    int strict_compliance;
    int slice_height;
    int slice_width;
    int interlaced;
    enum VC2_QM quant_matrix;

    /* Parse code state */
    int prev_parse_info_position;
    uint32_t prev_offset;

    int const_quant;

    int fragment_size;

    int expected_pos_y;

    int number_of_rows_sent;
    int number_of_lines_sent;
} VC2EncContext;

/* Puts an unsigned golomb code to the bitstream */
static av_always_inline void put_vc2_ue_uint(PutBitContext *pb, uint32_t val)
{
    int i;
    int pbits = 0, bits = 0, topbit = 1, maxval = 1;

    if (!val++) {
        put_bits(pb, 1, 1);
        return;
    }

    while (val > maxval) {
        topbit <<= 1;
        maxval <<= 1;
        maxval |=  1;
    }

    bits = ff_log2(topbit);

    for (i = 0; i < bits; i++) {
        topbit >>= 1;
        pbits <<= 2;
        if (val & topbit)
            pbits |= 0x1;
    }

    put_bits(pb, bits*2 + 1, (pbits << 1) | 1);
}

/* Returns the number of bits needed to encode a value */
static av_always_inline int count_vc2_ue_uint(uint32_t val)
{
    int topbit = 1, maxval = 1;

    if (!val++)
        return 1;

    while (val > maxval) {
        topbit <<= 1;
        maxval <<= 1;
        maxval |=  1;
    }

    return ff_log2(topbit)*2 + 1;
}

/* Returns the encoded value and the number of bits needed to encode it */
static av_always_inline void get_vc2_ue_uint(int val, uint8_t *nbits,
                                             uint32_t *eval)
{
    int i;
    int pbits = 0, bits = 0, topbit = 1, maxval = 1;

    if (!val++) {
        *nbits = 1;
        *eval = 1;
        return;
    }

    while (val > maxval) {
        topbit <<= 1;
        maxval <<= 1;
        maxval |=  1;
    }

    bits = ff_log2(topbit);

    for (i = 0; i < bits; i++) {
        topbit >>= 1;
        pbits <<= 2;
        if (val & topbit)
            pbits |= 0x1;
    }

    *nbits = bits*2 + 1;
    *eval = (pbits << 1) | 1;
}

/* VC-2 10.4 - parse_info() - keeps track of how many bytes it's been since the
 * last header the specs mention that you can just put 0 as the distance but the
 * libavcodec parser still needs them because the format sucks */
static void encode_parse_info(VC2EncContext *s, enum DiracParseCodes pcode,
        uint32_t next, uint32_t prev)
{
    avpriv_align_put_bits(&s->pb);
    s->prev_parse_info_position = put_bits_count(&s->pb) >> 3;
    s->prev_offset = next;

    /* Magic string */
    avpriv_put_string(&s->pb, "BBCD", 0);
    /* Parse code */
    put_bits(&s->pb, 8, pcode);
    /* Next parse offset */
    put_bits32(&s->pb, next);
    /* Last parse offset */
    put_bits32(&s->pb, prev);
}

static void write_prev_parse_info_next_offset(VC2EncContext *s, uint32_t value)
{
    uint8_t *ptr = s->pb.buf + s->prev_parse_info_position + 5;
    s->prev_offset = value;
    AV_WB32(ptr, value);
}

static uint32_t get_distance_from_prev_parse_info(VC2EncContext *s)
{
    return (put_bits_count(&s->pb) + 7 >> 3) - s->prev_parse_info_position;
}

/* VC-2 11.1 - parse_parameters()
 * The level dictates what the decoder should expect in terms of resolution
 * and allows it to quickly reject whatever it can't support. Remember,
 * this codec kinda targets cheapo FPGAs without much memory. Unfortunately
 * it also limits us greatly in our choice of formats, hence the flag to disable
 * strict_compliance */
static void encode_parse_params(VC2EncContext *s)
{
    put_vc2_ue_uint(&s->pb, s->ver.major); /* VC-2 demands this to be 2 */
    put_vc2_ue_uint(&s->pb, s->ver.minor); /* ^^ and this to be 0       */
    put_vc2_ue_uint(&s->pb, s->profile);   /* 3 to signal HQ profile    */
    put_vc2_ue_uint(&s->pb, s->level);     /* 3 - 1080/720, 6 - 4K      */
}

/* VC-2 11.3 - frame_size() */
static void encode_frame_size(VC2EncContext *s)
{
    put_bits(&s->pb, 1, !s->strict_compliance);
    if (!s->strict_compliance) {
        AVCodecContext *avctx = s->avctx;
        put_vc2_ue_uint(&s->pb, avctx->width);
        if (s->interlaced)
            put_vc2_ue_uint(&s->pb, 2*avctx->height);
        else
            put_vc2_ue_uint(&s->pb, avctx->height);
    }
}

/* VC-2 11.3.3 - color_diff_sampling_format() */
static void encode_sample_fmt(VC2EncContext *s)
{
    put_bits(&s->pb, 1, !s->strict_compliance);
    if (!s->strict_compliance) {
        int idx;
        if (s->chroma_x_shift == 1 && s->chroma_y_shift == 0)
            idx = 1; /* 422 */
        else if (s->chroma_x_shift == 1 && s->chroma_y_shift == 1)
            idx = 2; /* 420 */
        else
            idx = 0; /* 444 */
        put_vc2_ue_uint(&s->pb, idx);
    }
}

/* VC-2 11.3.4 - scan_format() */
static void encode_scan_format(VC2EncContext *s)
{
    put_bits(&s->pb, 1, !s->strict_compliance);
    if (!s->strict_compliance)
        put_vc2_ue_uint(&s->pb, s->interlaced);
}

/* VC-2 11.3.5 - frame_rate() */
static void encode_frame_rate(VC2EncContext *s)
{
    put_bits(&s->pb, 1, !s->strict_compliance);
    if (!s->strict_compliance) {
        AVCodecContext *avctx = s->avctx;
        put_vc2_ue_uint(&s->pb, 0);
        if (s->interlaced)
            put_vc2_ue_uint(&s->pb, avctx->time_base.den/2);
        else
            put_vc2_ue_uint(&s->pb, avctx->time_base.den);
        put_vc2_ue_uint(&s->pb, avctx->time_base.num);
    }
}

/* VC-2 11.3.6 - aspect_ratio() */
static void encode_aspect_ratio(VC2EncContext *s)
{
    put_bits(&s->pb, 1, !s->strict_compliance);
    if (!s->strict_compliance) {
        AVCodecContext *avctx = s->avctx;
        put_vc2_ue_uint(&s->pb, 0);
        put_vc2_ue_uint(&s->pb, avctx->sample_aspect_ratio.num);
        put_vc2_ue_uint(&s->pb, avctx->sample_aspect_ratio.den);
    }
}

/* VC-2 11.3.7 - clean_area() */
static void encode_clean_area(VC2EncContext *s)
{
    put_bits(&s->pb, 1, 0);
}

/* VC-2 11.3.8 - signal_range() */
static void encode_signal_range(VC2EncContext *s)
{
    put_bits(&s->pb, 1, !s->strict_compliance);
    if (!s->strict_compliance)
        put_vc2_ue_uint(&s->pb, s->bpp_idx);
}

/* VC-2 11.3.9 - color_spec() */
static void encode_color_spec(VC2EncContext *s)
{
    AVCodecContext *avctx = s->avctx;
    put_bits(&s->pb, 1, !s->strict_compliance);
    if (!s->strict_compliance) {
        int val;
        put_vc2_ue_uint(&s->pb, 0);

        /* primaries */
        put_bits(&s->pb, 1, 1);
        if (avctx->color_primaries == AVCOL_PRI_BT470BG)
            val = 2;
        else if (avctx->color_primaries == AVCOL_PRI_SMPTE170M)
            val = 1;
        else if (avctx->color_primaries == AVCOL_PRI_SMPTE240M)
            val = 1;
        else
            val = 0;
        put_vc2_ue_uint(&s->pb, val);

        /* color matrix */
        put_bits(&s->pb, 1, 1);
        if (avctx->colorspace == AVCOL_SPC_RGB)
            val = 3;
        else if (avctx->colorspace == AVCOL_SPC_YCOCG)
            val = 2;
        else if (avctx->colorspace == AVCOL_SPC_BT470BG)
            val = 1;
        else
            val = 0;
        put_vc2_ue_uint(&s->pb, val);

        /* transfer function */
        put_bits(&s->pb, 1, 1);
        if (avctx->color_trc == AVCOL_TRC_LINEAR)
            val = 2;
        else if (avctx->color_trc == AVCOL_TRC_BT1361_ECG)
            val = 1;
        else
            val = 0;
        put_vc2_ue_uint(&s->pb, val);
    }
}

/* VC-2 11.3 - source_parameters() */
static void encode_source_params(VC2EncContext *s)
{
    encode_frame_size(s);
    encode_sample_fmt(s);
    encode_scan_format(s);
    encode_frame_rate(s);
    encode_aspect_ratio(s);
    encode_clean_area(s);
    encode_signal_range(s);
    encode_color_spec(s);
}

/* VC-2 11 - sequence_header() */
static void encode_seq_header(VC2EncContext *s)
{
    avpriv_align_put_bits(&s->pb);
    encode_parse_params(s);
    put_vc2_ue_uint(&s->pb, s->base_vf);
    encode_source_params(s);
    put_vc2_ue_uint(&s->pb, s->interlaced); /* Frames or fields coding */
}

/* VC-2 12.3.4.1 - slice_parameters() */
static void encode_slice_params(VC2EncContext *s)
{
    put_vc2_ue_uint(&s->pb, s->num_x);
    put_vc2_ue_uint(&s->pb, s->num_y);
    put_vc2_ue_uint(&s->pb, s->prefix_bytes);
    put_vc2_ue_uint(&s->pb, s->size_scaler);
}

/* 1st idx = LL, second - vertical, third - horizontal, fourth - total */
const uint8_t vc2_qm_col_tab[][4] = {
    {20,  9, 15,  4}, /* Level 1 - roughest details */
    { 0,  6,  6,  4}, /* Level 2 */
    { 0,  3,  3,  5}, /* Level 3 */
    { 0,  3,  5,  1}, /* Level 4 */
    { 0, 11, 10, 11}  /* Level 5 - finest details */
};

/* Flat qm table, optimized for PSNR */
const uint8_t vc2_qm_flat_tab[][4] = {
    { 0,  0,  0,  0},
    { 0,  0,  0,  0},
    { 0,  0,  0,  0},
    { 0,  0,  0,  0},
    { 0,  0,  0,  0}
};

static void init_quant_matrix(VC2EncContext *s)
{
    int level, orientation;

    if (s->wavelet_depth <= 4 && s->quant_matrix == VC2_QM_DEF) {
        s->custom_quant_matrix = 0;
        for (level = 0; level < s->wavelet_depth; level++) {
            s->quant[level][0] = ff_dirac_default_qmat[s->wavelet_idx][level][0];
            s->quant[level][1] = ff_dirac_default_qmat[s->wavelet_idx][level][1];
            s->quant[level][2] = ff_dirac_default_qmat[s->wavelet_idx][level][2];
            s->quant[level][3] = ff_dirac_default_qmat[s->wavelet_idx][level][3];
        }
        return;
    }

    /* Levels over 3 are required to have a custom quantization matrix */
    s->custom_quant_matrix = 1;

    if (s->quant_matrix == VC2_QM_DEF) {
        for (level = 0; level < s->wavelet_depth; level++) {
            for (orientation = 0; orientation < 4; orientation++) {
                /* The default quantization matrix doesn't have values for
                 * levels over 3 so use the values from the Color QM tab we have */
                if (level <= 3)
                    s->quant[level][orientation] = ff_dirac_default_qmat[s->wavelet_idx][level][orientation];
                else
                    s->quant[level][orientation] = vc2_qm_col_tab[level][orientation];
            }
        }
    } else if (s->quant_matrix == VC2_QM_COL) {
        for (level = 0; level < s->wavelet_depth; level++) {
            for (orientation = 0; orientation < 4; orientation++) {
                s->quant[level][orientation] = vc2_qm_col_tab[level][orientation];
            }
        }
    } else {
        for (level = 0; level < s->wavelet_depth; level++) {
            for (orientation = 0; orientation < 4; orientation++) {
                s->quant[level][orientation] = vc2_qm_flat_tab[level][orientation];
            }
        }
    }
}

/* VC-2 12.3.4.2 - quant_matrix() */
static void encode_quant_matrix(VC2EncContext *s)
{
    int level;
    put_bits(&s->pb, 1, s->custom_quant_matrix);
    if (s->custom_quant_matrix) {
        put_vc2_ue_uint(&s->pb, s->quant[0][0]);
        for (level = 0; level < s->wavelet_depth; level++) {
            put_vc2_ue_uint(&s->pb, s->quant[level][1]);
            put_vc2_ue_uint(&s->pb, s->quant[level][2]);
            put_vc2_ue_uint(&s->pb, s->quant[level][3]);
        }
    }
}

/* VC-2 12.3 - transform_parameters() */
static void encode_transform_params(VC2EncContext *s)
{
    put_vc2_ue_uint(&s->pb, s->wavelet_idx);
    put_vc2_ue_uint(&s->pb, s->wavelet_depth);

    if (s->ver.major >= 3) {
        /* extended_transform_parameters */
        put_bits(&s->pb, 1, 0); /* asym_transform_index_flag */
        put_bits(&s->pb, 1, 0); /* asym_transform_flag */
    }

    encode_slice_params(s);
    encode_quant_matrix(s);
}

static void encode_fragment_header(VC2EncContext *s,
        int data_length, int slice_count, int x_offset, int y_offset)
{
    avpriv_align_put_bits(&s->pb);
    put_bits32(&s->pb, s->picture_number);
    put_bits(&s->pb, 16, data_length);
    put_bits(&s->pb, 16, slice_count);
    if (slice_count) {
        put_bits(&s->pb, 16, x_offset);
        put_bits(&s->pb, 16, y_offset);
    }
}

/* Quantize a single unsigned coefficient - left shift on signed is undefined,
 * can use a multiply but it's easier to handle the sign separately */
#define QUANT(c, qf) (((c) << 2)/(qf))

/* VC-2 13.5.5.2 - slice_band() */

static void encode_subband2(VC2EncContext *s, PutBitContext *pb, Plane *p,
                            int sx, int sy,
                            SubBand *b, int quant)
{
    int x, y;

    const int left   = b->width  * (sx+0) / s->num_x;
    const int right  = b->width  * (sx+1) / s->num_x;
    const int top    = b->height * (sy+0) / s->num_y;
    const int bottom = b->height * (sy+1) / s->num_y;

    const int qfactor = ff_dirac_qscale_tab[quant];
    const uint8_t  *len_lut = &s->coef_lut_len[quant*COEF_LUT_TAB];
    const uint32_t *val_lut = &s->coef_lut_val[quant*COEF_LUT_TAB];

    dwtcoef *coeff = b->buf + top * b->stride;

    const ptrdiff_t hstride = b->hstride;

    for (y = top; y < bottom; y++) {
        for (x = left; x < right; x++) {
            const int neg = coeff[x * hstride] < 0;
            uint32_t c_abs = FFABS(coeff[x * hstride]);
            if (c_abs < COEF_LUT_TAB) {
                /* Append the sign to the end of the LUT value */
                put_bits(pb, len_lut[c_abs], val_lut[c_abs] | neg);
            } else {
                c_abs = QUANT(c_abs, qfactor);
                put_vc2_ue_uint(pb, c_abs);
                if (c_abs)
                    put_bits(pb, 1, neg);
            }
        }
        coeff += b->stride;
    }
}

static int count_hq_slice(SliceArgs *slice, int quant_idx)
{
    int x, y;
    uint8_t quants[MAX_DWT_LEVELS][4];
    int bits = 0, i, level, orientation;
    VC2EncContext *s = slice->ctx;

    /* Check the cache */
    if (slice->cache[quant_idx])
        return slice->cache[quant_idx];

    bits += 8*s->prefix_bytes;
    bits += 8; /* quant_idx */

    for (level = 0; level < s->wavelet_depth; level++)
        for (orientation = !!level; orientation < 4; orientation++)
            quants[level][orientation] = FFMAX(quant_idx - s->quant[level][orientation], 0);

    for (i = 0; i < 3; i++) {
        Plane *p = &s->plane[i];
        int bytes_start, bytes_len, pad_s, pad_c;
        bytes_start = bits >> 3;
        bits += 8;
        for (level = 0; level < s->wavelet_depth; level++) {
            for (orientation = !!level; orientation < 4; orientation++) {
                SubBand *b = &p->band[level][orientation];

                const int q_idx = quants[level][orientation];
                const uint8_t *len_lut = &s->coef_lut_len[q_idx*COEF_LUT_TAB];
                const int qfactor = ff_dirac_qscale_tab[q_idx];

                const int left   = b->width  * slice->x    / s->num_x;
                const int right  = b->width  *(slice->x+1) / s->num_x;
                const int top    = b->height * slice->y    / s->num_y;
                const int bottom = b->height *(slice->y+1) / s->num_y;

                dwtcoef *buf = b->buf + top * b->stride;

                const ptrdiff_t hstride = b->hstride;

                for (y = top; y < bottom; y++) {
                    for (x = left; x < right; x++) {
                        uint32_t c_abs = FFABS(buf[x * hstride]);
                        if (c_abs < COEF_LUT_TAB) {
                            bits += len_lut[c_abs];
                        } else {
                            c_abs = QUANT(c_abs, qfactor);
                            bits += count_vc2_ue_uint(c_abs);
                            bits += !!c_abs;
                        }
                    }
                    buf += b->stride;
                }
            }
        }
        bits += FFALIGN(bits, 8) - bits;
        bytes_len = (bits >> 3) - bytes_start - 1;
        pad_s = FFALIGN(bytes_len, s->size_scaler)/s->size_scaler;
        pad_c = (pad_s*s->size_scaler) - bytes_len;
        bits += pad_c*8;
    }

    slice->cache[quant_idx] = bits;

    return bits;
}

/* Approaches the best possible quantizer asymptotically, its kinda exaustive
 * but we have a LUT to get the coefficient size in bits. Guaranteed to never
 * overshoot, which is apparently very important when streaming */
static int rate_control(AVCodecContext *avctx, void *arg)
{
    SliceArgs *slice_dat = arg;
    VC2EncContext *s = slice_dat->ctx;

    /* Sets a top target which is the maximum slice size */
    const int top = slice_dat->bits_ceil;

    /* Sets a bottom target which is the maximum slice size/tolerance value */
    const int bottom = slice_dat->bits_floor;

    /* Keeps the previous 2 quantization index values if the second last is
     * identical to the current it means it's entered an infinite loop -
     * E.g. 12, 13, 12, 13, 12 - neither satisfy the target range, so break out
     * and pick the lowest value - waste goes to the second pass */
    int quant_buf[2] = {-1, -1};

    /* Starts from the previous quantization index (if it exists) */
    int quant = slice_dat->quant_idx;

    /* Step size depending on how far away it is from satisfying the range */
    int step = 1;

    int bits_last, bits = count_hq_slice(slice_dat, quant);

    while ((bits > top) || (bits < bottom)) {
        const int signed_step = bits > top ? +step : -step;
        quant  = av_clip(quant + signed_step, 0, s->q_ceil-1);
        bits   = count_hq_slice(slice_dat, quant);
        if (quant_buf[1] == quant) {
            quant = FFMAX(quant_buf[0], quant);
            bits  = quant == quant_buf[0] ? bits_last : bits;
            break;
        }
        step         = av_clip(step/2, 1, (s->q_ceil - 1)/2);
        quant_buf[1] = quant_buf[0];
        quant_buf[0] = quant;
        bits_last    = bits;
    }
    slice_dat->quant_idx = av_clip(quant, 0, s->q_ceil-1);
    slice_dat->bytes = SSIZE_ROUND(bits >> 3);
    return 0;
}

static int calc_slice_sizes(VC2EncContext *s)
{
    int i, j, slice_x, slice_y, bytes_left = 0;
    int bytes_top[SLICE_REDIST_TOTAL] = {0};
    int64_t total_bytes_needed = 0;
    int slice_redist_range = FFMIN(SLICE_REDIST_TOTAL, s->num_y_partial*s->num_x);
    SliceArgs *enc_args = s->slice_args;
    SliceArgs *top_loc[SLICE_REDIST_TOTAL] = {NULL};

    /* Set up the quantization matrices for each level & orientation */
    init_quant_matrix(s);

    /* Initialize the slice contexts - doesn't reset the quantization index */
    for (slice_y = 0; slice_y < s->num_y_partial; slice_y++) {
        for (slice_x = 0; slice_x < s->num_x; slice_x++) {
            SliceArgs *args = &enc_args[s->num_x*slice_y + slice_x];
            args->bits_ceil  = s->slice_max_bytes << 3;
            args->bits_floor = s->slice_min_bytes << 3;
            memset(args->cache, 0, s->q_ceil*sizeof(*args->cache));
        }
    }

    /* 1st pass - strictly fits the slices under the maximum slice size,
     * aligning the slice size to below the maximum slice size wastes bits
     * which get used by the second pass */
    s->avctx->execute(s->avctx, rate_control, enc_args, NULL, s->num_y_partial*s->num_x,
                      sizeof(SliceArgs));

    /* Fill up the pointers to the most costly slice_redist_range number of slices */
    for (i = 0; i < s->num_y_partial*s->num_x; i++) {
        SliceArgs *args = &enc_args[i];
        bytes_left += args->bytes;
        for (j = 0; j < slice_redist_range; j++) {
            if (args->bytes > bytes_top[j]) {
                bytes_top[j] = args->bytes;
                top_loc[j]   = args;
                break;
            }
        }
    }

    bytes_left = (s->num_y_partial * s->frame_max_bytes / s->num_y) - bytes_left;

    /* 2nd pass - uses the leftover bits and distributes them to the highest
     * costing slices to boost the quality. Not required, you can comment it
     * out to gain a little more speed, but impact is very minimal */
    while (bytes_left > 0) {
        int distributed = 0;
        for (i = 0; i < slice_redist_range; i++) {
            SliceArgs *args;
            int bits, bytes, diff, prev_bytes, new_idx;
            if (bytes_left <= 0)
                break;
            if (!top_loc[i] || !top_loc[i]->quant_idx)
                break;
            args = top_loc[i];
            prev_bytes = args->bytes;
            new_idx = FFMAX(args->quant_idx - 1, 0);
            bits  = count_hq_slice(args, new_idx);
            bytes = SSIZE_ROUND(bits >> 3);
            diff  = bytes - prev_bytes;
            if ((bytes_left - diff) > 0) {
                args->quant_idx = new_idx;
                args->bytes = bytes;
                bytes_left -= diff;
                distributed++;
            }
        }
        if (!distributed)
            break;
    }

    /* Counts total bytes for each slices to know how big of a packet to allocate,
     * keeps track of the average quantizer used, generally anything above 50
     * will give a gray frame */
    for (i = 0; i < s->num_y_partial*s->num_x; i++) {
        SliceArgs *args = &enc_args[i];
        total_bytes_needed += args->bytes;
        s->q_avg += args->quant_idx;
        s->slice_count += 1;
    }

    return total_bytes_needed;
}

/* VC-2 13.5.3 - hq_slice */
static int encode_hq_slice(AVCodecContext *avctx, void *arg)
{
    SliceArgs *slice_dat = arg;
    VC2EncContext *s = slice_dat->ctx;
    PutBitContext *pb = &slice_dat->pb;
    const int slice_x = slice_dat->x;
    const int slice_y = slice_dat->y;
    const int quant_idx = slice_dat->quant_idx;
    uint8_t quants[MAX_DWT_LEVELS][4];
    int i, level, orientation;

    /* The reference decoder ignores it, and its typical length is 0 */
    memset(put_bits_ptr(pb), 0, s->prefix_bytes);
    skip_put_bytes(pb, s->prefix_bytes);

    put_bits(pb, 8, quant_idx);

    /* Slice quantization (slice_quantizers() in the specs) */
    for (level = 0; level < s->wavelet_depth; level++)
        for (orientation = !!level; orientation < 4; orientation++)
            quants[level][orientation] = FFMAX(quant_idx - s->quant[level][orientation], 0);

    /* Luma + 2 Chroma planes */
    for (i = 0; i < 3; i++) {
        int bytes_start, bytes_len, pad_s, pad_c;
        Plane *p = &s->plane[i];
        bytes_start = put_bits_count(pb) >> 3;
        put_bits(pb, 8, 0);
        for (level = 0; level < s->wavelet_depth; level++) {
            for (orientation = !!level; orientation < 4; orientation++) {
                encode_subband2(s, pb, p, slice_x, slice_y,
                               &p->band[level][orientation],
                               quants[level][orientation]);
            }
        }
        avpriv_align_put_bits(pb);
        bytes_len = (put_bits_count(pb) >> 3) - bytes_start - 1;
        if (i == 2) {
            int len_diff = slice_dat->bytes - (put_bits_count(pb) >> 3);
            pad_s = FFALIGN((bytes_len + len_diff), s->size_scaler)/s->size_scaler;
            pad_c = (pad_s*s->size_scaler) - bytes_len;
        } else {
            pad_s = FFALIGN(bytes_len, s->size_scaler)/s->size_scaler;
            pad_c = (pad_s*s->size_scaler) - bytes_len;
        }
        pb->buf[bytes_start] = pad_s;
        flush_put_bits(pb);
        /* vc2-reference uses that padding that decodes to '0' coeffs -
         * that way if there's corruption and the decoder overreads it'll
         * decode the coefficients to 0 and end quickly */
        memset(put_bits_ptr(pb), 0xFF, pad_c);
        skip_put_bytes(pb, pad_c);
    }

    return 0;
}

/*
 * Transform basics for a 3 level transform
 * |---------------------------------------------------------------------|
 * |  LL-0  | HL-0  |                 |                                  |
 * |--------|-------|      HL-1       |                                  |
 * |  LH-0  | HH-0  |                 |                                  |
 * |----------------|-----------------|              HL-2                |
 * |                |                 |                                  |
 * |     LH-1       |      HH-1       |                                  |
 * |                |                 |                                  |
 * |----------------------------------|----------------------------------|
 * |                                  |                                  |
 * |                                  |                                  |
 * |                                  |                                  |
 * |              LH-2                |              HH-2                |
 * |                                  |                                  |
 * |                                  |                                  |
 * |                                  |                                  |
 * |---------------------------------------------------------------------|
 *
 * DWT transforms are generally applied by splitting the image in two vertically
 * and applying a low pass transform on the left part and a corresponding high
 * pass transform on the right hand side. This is known as the horizontal filter
 * stage.
 * After that, the same operation is performed except the image is divided
 * horizontally, with the high pass on the lower and the low pass on the higher
 * side.
 * Therefore, you're left with 4 subdivisions - known as  low-low, low-high,
 * high-low and high-high. They're referred to as orientations in the decoder
 * and encoder.
 *
 * The LL (low-low) area contains the original image downsampled by the amount
 * of levels. The rest of the areas can be thought as the details needed
 * to restore the image perfectly to its original size.
 */

static void load_pixel_data(const void *pixels, dwtcoef *coeffs,
        ptrdiff_t pixel_stride, ptrdiff_t coeff_stride,
        int width, int height, int bytes_per_pixel, dwtcoef diff_offset)
{
    int x, y;

    if (bytes_per_pixel == 1) {
        const uint8_t *pix = (const uint8_t *)pixels;

        for (y = 0; y < height; y++) {
            for (x = 0; x < width; x++)
                coeffs[x] = pix[x] - diff_offset;
            coeffs += coeff_stride;
            pix += pixel_stride;
        }
    } else {
        const uint16_t *pix = (const uint16_t *)pixels;
        pixel_stride /= 2;

        for (y = 0; y < height; y++) {
            for (x = 0; x < width; x++)
                coeffs[x] = pix[x] - diff_offset;
            coeffs += coeff_stride;
            pix += pixel_stride;
        }
    }
}

static int import_transform_plane(AVCodecContext *avctx, void *arg, int jobnr, int threadnr)
{
    VC2EncContext *s       = avctx->priv_data;
    TransformArgs *ta      = &s->transform_args[jobnr];
    Plane *p               = &s->plane[jobnr];
    VC2TransformContext *t = &ta->t;
    const AVFrame *frame   = ta->frame;

    int chroma_y_shift = jobnr ? s->chroma_y_shift : 0;
    int height = frame->height >> chroma_y_shift;
    int pos_y = frame->pos_y >> chroma_y_shift;

    if (pos_y + height > p->height)
        height = p->height - pos_y;

    load_pixel_data(frame->data[jobnr], p->coef_buf + pos_y*p->coef_stride,
            frame->linesize[jobnr], p->coef_stride,
            p->width, height, s->bpp, s->diff_offset);

    if (pos_y + height == p->height) {
        memset(p->coef_buf + p->height*p->coef_stride, 0, p->coef_stride * (p->dwt_height - p->height) * sizeof(dwtcoef));
        height = p->dwt_height;
    } else {
        height = pos_y + height;
    }

    ff_vc2enc_transform(t, p->coef_buf,
            p->coef_stride, p->dwt_width, p->dwt_height,
            height, s->wavelet_depth, s->wavelet_idx);

    return 0;
}

static int slice_rows_available(const VC2EncContext *s)
{
    int y, p, depth;

    for (y = s->number_of_rows_sent; y < s->num_y; y++) {
        for (p = 0; p < 2; p++) { /* Only check 1 chroma plane */
            for (depth = s->wavelet_depth; depth > 0; depth--) {
                int height = s->plane[p].dwt_height >> depth;
                if (height*(y+1)/s->num_y * 2 > s->transform_args[p].t.progress[depth-1].vfilter_stage1)
                    return y;
            }
        }
    }
    return y;
}

static int lines_in_packet(const VC2EncContext *s)
{
    int y, ret = 0;
    const int height = s->plane[0].dwt_height >> 1;
    for (y = s->number_of_rows_sent; y < s->number_of_rows_sent + s->num_y_partial; y++)
        ret += 2 * (height*(y+1)/s->num_y - height*(y)/s->num_y);
    return ret;
}

/**
 * Dirac Specification ->
 * 9.6 Parse Info Header Syntax. parse_info()
 * 4 byte start code + byte parse code + 4 byte size + 4 byte previous size
 */
#define DATA_UNIT_HEADER_SIZE 13

/* Field - 0 progressive, 1 - top field, 2 - bottom field */
static int encode_frame(VC2EncContext *s, AVPacket *avpkt, const AVFrame *frame,
                        const char *aux_data, const int header_size,
                        int *got_packet, int field)
{
    int i, x, y, ret;
    int64_t max_frame_bytes;

    /* Import image data and transform */
    for (i = 0; i < 3; i++) {
        Plane *p = &s->plane[i];

        if (frame->pos_y == 0)
            ff_vc2enc_reset_transforms(&s->transform_args[i].t);

        s->transform_args[i].ctx   = s;
        s->transform_args[i].plane = p;
        s->transform_args[i].frame = frame;
    }
    s->avctx->execute2(s->avctx, import_transform_plane, NULL, NULL, 3);

    /* Check how many rows are available. */
    if (slice_rows_available(s) <= s->number_of_rows_sent) {
        /* If no rows rows are available return no packet. */
        *got_packet = 0;
        return 0;
    }

    /* Create and write packet. */
    s->num_y_partial = slice_rows_available(s) - s->number_of_rows_sent;
    for (y = 0; y < s->num_y_partial; y++)
        for (x = 0; x < s->num_x; x++)
            s->slice_args[y*s->num_x + x].y = y + s->number_of_rows_sent;

    /* Calculate per-slice quantizers and sizes */
    max_frame_bytes = header_size + calc_slice_sizes(s)
        + s->num_y_partial*(s->num_x / s->fragment_size) * (13+12); /* "BBCD" header and fragment header */;

    ret = ff_alloc_packet2(s->avctx, avpkt,
            max_frame_bytes,
            max_frame_bytes);
    if (ret) {
        av_log(s->avctx, AV_LOG_ERROR, "Error getting output packet.\n");
        return ret;
    }
    init_put_bits(&s->pb, avpkt->data, avpkt->size);

    if (s->number_of_rows_sent == 0) {
        int before, after;

        /* Sequence header */
        encode_parse_info(s, DIRAC_PCODE_SEQ_HEADER, 0, s->prev_offset);
        encode_seq_header(s);

        /* Encoder version */
        if (aux_data) {
            write_prev_parse_info_next_offset(s, get_distance_from_prev_parse_info(s));
            encode_parse_info(s, DIRAC_PCODE_AUX, DATA_UNIT_HEADER_SIZE + strlen(aux_data) + 1, s->prev_offset);
            avpriv_put_string(&s->pb, aux_data, 1);
        }

        /* (14. Fragment Syntax) */
        write_prev_parse_info_next_offset(s, get_distance_from_prev_parse_info(s));
        encode_parse_info(s, DIRAC_PCODE_PICTURE_FRAGMENT_HQ, 0, s->prev_offset);

        before = put_bits_count(&s->pb) >> 3;
        encode_fragment_header(s, 0, 0, 0, 0);
        encode_transform_params(s);
        after = put_bits_count(&s->pb) >> 3;

        AV_WB16(s->pb.buf + before + 4, after - before - 8);
    }

    /* Encode slices */
    for (y = 0; y < s->num_y_partial; y++) {
    for (int x = 0; x < s->num_x; x++) {
        SliceArgs *arg = &s->slice_args[y*s->num_x + x];
        uint8_t *buf;

        if (!(x % s->fragment_size)) {
            if (s->prev_parse_info_position >= 0)
                write_prev_parse_info_next_offset(s, get_distance_from_prev_parse_info(s));
            encode_parse_info(s, DIRAC_PCODE_PICTURE_FRAGMENT_HQ, 0, s->prev_offset);
            if (s->fragment_size == 1)
                encode_fragment_header(s, arg->bytes, s->fragment_size, x, arg->y);
            else
                encode_fragment_header(s, 0, s->fragment_size, x, arg->y);
            avpriv_align_put_bits(&s->pb);
        }

        avpriv_align_put_bits(&s->pb);
        flush_put_bits(&s->pb);
        buf = put_bits_ptr(&s->pb);

        assert(arg->bytes > 0);

        init_put_bits(&arg->pb, buf, arg->bytes+s->prefix_bytes);

        /* Skips the main put_bits's position by the total slice bytes */
        skip_put_bytes(&s->pb, arg->bytes+s->prefix_bytes);
    }
    }

    s->avctx->execute(s->avctx, encode_hq_slice, s->slice_args, NULL,
            s->num_y_partial * s->num_x, sizeof(SliceArgs));

    write_prev_parse_info_next_offset(s, get_distance_from_prev_parse_info(s));

    /* End sequence */
    /* 2012 spec. 10.3.2, 2017 spec. 10.4.3:
     * Where pictures are fields, a sequence shall comprise a whole number of
     * frames (i.e., an even number of fields) and shall begin and end with a
     * whole frame/field-pair. */
    if (field != 1 && s->num_y == s->number_of_rows_sent + s->num_y_partial) {
        encode_parse_info(s, DIRAC_PCODE_END_SEQ, 13, s->prev_offset);
    }

    s->number_of_rows_sent += s->num_y_partial;
    s->number_of_lines_sent += lines_in_packet(s);
    *got_packet = 1;
    return 0;
}

static av_cold int vc2_encode_frame(AVCodecContext *avctx, AVPacket *avpkt,
                                      const AVFrame *frame, int *got_packet)
{
    int ret = 0;
    int slice_ceil, sig_size = 256;
    VC2EncContext *s = avctx->priv_data;
    const int bitexact = avctx->flags & AV_CODEC_FLAG_BITEXACT;
    const char *aux_data = bitexact ? "Lavc" : LIBAVCODEC_IDENT;
    const int aux_data_size = bitexact ? sizeof("Lavc") : sizeof(LIBAVCODEC_IDENT);
    const int header_size = 100 + aux_data_size;

    if (frame->pos_x || frame->pos_y != s->expected_pos_y) {
        av_log(avctx, AV_LOG_ERROR, "given picture at position (%d,%d) not at expected position (%d,%d)\n",
                frame->pos_x, frame->pos_y, 0, s->expected_pos_y);
        return AVERROR(EINVAL);
    }

    if (frame->pos_y + frame->height > avctx->height) {
        av_log(avctx, AV_LOG_WARNING, "given %d lines, at (0,%d)\n", frame->height, frame->pos_y);
    }

    s->avctx = avctx;
    s->prev_parse_info_position = -1;

    if (!frame->pos_y) {
        s->size_scaler = 2;
        s->prefix_bytes = 0;
        s->number_of_rows_sent = s->number_of_lines_sent = 0;

        /* Rate control */
        /* bitrate(per second)/framerate */
        s->frame_max_bytes = (av_rescale(avctx->bit_rate, s->avctx->time_base.num,
                                        s->avctx->time_base.den) >> 3) - header_size;
        /* preliminary max slice size (frame_max_bytes/number_of_slices) */
        s->slice_max_bytes = slice_ceil = FFMIN(
                av_rescale(s->frame_max_bytes, 1, s->num_x * s->num_y),
                1440 - 25 /* "BBCD and fragment headers */ - 16 /* RTP and ext headers */
                );

        /* Find an appropriate size scaler */
        while (sig_size > 255) {
            int r_size = SSIZE_ROUND(s->slice_max_bytes);
            if (r_size > slice_ceil) {
                s->slice_max_bytes -= r_size - slice_ceil;
                r_size = SSIZE_ROUND(s->slice_max_bytes);
            }
            sig_size = r_size/s->size_scaler; /* Signalled slize size */
            s->size_scaler <<= 1;
        }

        s->slice_min_bytes = s->slice_max_bytes - s->slice_max_bytes*(s->tolerance/100.0f);
    }

    /* Calls encode_frame() once for each field (or once for progressive) */

    ret = encode_frame(s, avpkt, frame, aux_data, header_size, got_packet,
            (s->interlaced) ? 1 + (s->picture_number & 1) : 0);
    if (ret)
        return ret;

    if (*got_packet) {
        flush_put_bits(&s->pb);
        avpkt->size = put_bits_count(&s->pb) >> 3;
    }

    s->expected_pos_y += frame->height;
    if (s->expected_pos_y >= avctx->height) {
        int avg_quant = s->q_avg/s->slice_count;
        s->expected_pos_y = 0;
        s->picture_number++;

        if (avg_quant >= 50)
            av_log(avctx, AV_LOG_WARNING, "average quantizer very large: %i\n", avg_quant);
        s->q_avg = s->slice_count = 0;
    }

    return 0;
}

static av_cold int vc2_encode_end(AVCodecContext *avctx)
{
    int i;
    VC2EncContext *s = avctx->priv_data;

    av_log(avctx, AV_LOG_INFO, "Qavg: %i\n", s->q_avg);

    for (i = 0; i < 3; i++) {
        av_freep(&s->plane[i].coef_buf);
    }

    av_freep(&s->slice_args);
    av_freep(&s->coef_lut_len);
    av_freep(&s->coef_lut_val);

    return 0;
}

static av_cold int vc2_encode_init(AVCodecContext *avctx)
{
    int i, j, level, o, shift, ret;
    int const_quant = 0;
    const AVPixFmtDescriptor *fmt = av_pix_fmt_desc_get(avctx->pix_fmt);
    const int depth = fmt->comp[0].depth;
    VC2EncContext *s = avctx->priv_data;

    s->picture_number = 0;

    /* Total allowed quantization range, minimum quality, reduce DIRAC_MAX_QUANT_INDEX
     * to boost minimal quality the rate control system will set (clips the maximum quantizer) */
    s->q_ceil    = DIRAC_MAX_QUANT_INDEX;

    s->ver.major = (s->fragment_size) ? 3 : 2;
    s->ver.minor = 0;
    s->profile   = 3;
    s->level     = 3;

    s->base_vf   = -1;
    s->strict_compliance = 1;

    s->q_avg = 0;
    s->slice_max_bytes = 0;
    s->slice_min_bytes = 0;

    /* Mark unknown as progressive */
    s->interlaced = !((avctx->field_order == AV_FIELD_UNKNOWN) ||
                      (avctx->field_order == AV_FIELD_PROGRESSIVE));

    {
    int height = avctx->height;
    AVRational tb = avctx->time_base;
    if (s->interlaced) {
        height *= 2;
        tb.den /= 2;
    }

    for (i = 0; i < base_video_fmts_len; i++) {
        const VC2BaseVideoFormat *fmt = &base_video_fmts[i];
        if (avctx->pix_fmt != fmt->pix_fmt)
            continue;
        /* TODO: what to do about field separated pictures and their 2x rate */
        if (tb.num != fmt->time_base.num)
            continue;
        if (tb.den != fmt->time_base.den)
            continue;
        if (avctx->width != fmt->width)
            continue;
        if (height != fmt->height)
            continue;
        if (s->interlaced != fmt->interlaced)
            continue;
        s->base_vf = i;
        s->level   = base_video_fmts[i].level;
        break;
    }
    }

    if (s->interlaced)
        av_log(avctx, AV_LOG_WARNING, "Interlacing enabled!\n");

    if (avctx->flags & AV_CODEC_FLAG_QSCALE) {
        const int quant_max = FF_ARRAY_ELEMS(ff_dirac_qscale_tab);
        const_quant = avctx->global_quality / FF_QP2LAMBDA;

        if (const_quant < 0 || const_quant >= quant_max) {
            av_log(avctx, AV_LOG_ERROR, "constant quantiser (%d) outside valid range [%d..%d]\n",
                    const_quant, 0, quant_max);
            return AVERROR(EINVAL);
        }
        av_log(avctx, AV_LOG_WARNING, "encoding with constant quantiser (%d)\n",
                const_quant);
        s->const_quant = const_quant;
    }

    if ((s->slice_width > avctx->width) ||
        (s->slice_height > avctx->height)) {
        av_log(avctx, AV_LOG_ERROR, "Slice size is bigger than the image!\n");
        return AVERROR(EINVAL);
    }

    if (s->base_vf <= 0) {
        if (avctx->strict_std_compliance < FF_COMPLIANCE_STRICT) {
            s->strict_compliance = s->base_vf = 0;
            av_log(avctx, AV_LOG_WARNING, "Format does not strictly comply with VC2 specs\n");
        } else {
            av_log(avctx, AV_LOG_WARNING, "Given format does not strictly comply with "
                   "the specifications, decrease strictness to use it.\n");
            return AVERROR(EINVAL);
        }
    } else {
        av_log(avctx, AV_LOG_INFO, "Selected base video format = %i (%s)\n",
               s->base_vf, base_video_fmts[s->base_vf].name);
    }

    /* Chroma subsampling */
    ret = av_pix_fmt_get_chroma_sub_sample(avctx->pix_fmt, &s->chroma_x_shift, &s->chroma_y_shift);
    if (ret)
        return ret;

    /* Bit depth and color range index */
    if (depth == 8 && avctx->color_range == AVCOL_RANGE_JPEG) {
        s->bpp = 1;
        s->bpp_idx = 1;
        s->diff_offset = 128;
    } else if (depth == 8 && (avctx->color_range == AVCOL_RANGE_MPEG ||
               avctx->color_range == AVCOL_RANGE_UNSPECIFIED)) {
        s->bpp = 1;
        s->bpp_idx = 2;
        s->diff_offset = 128;
    } else if (depth == 10) {
        s->bpp = 2;
        s->bpp_idx = 3;
        s->diff_offset = 512;
    } else {
        s->bpp = 2;
        s->bpp_idx = 4;
        s->diff_offset = 2048;
    }

    /* Planes initialization */
    for (i = 0; i < 3; i++) {
        int chroma_x_shift = i ? s->chroma_x_shift : 0;
        int chroma_y_shift = i ? s->chroma_y_shift : 0;
        int w              = avctx->width    >> chroma_x_shift;
        int h              = avctx->height   >> chroma_y_shift;
        int alignment      = 1 << s->wavelet_depth;
        Plane *p           = &s->plane[i];
        int hstride        = 1;

        p->width      = w;
        p->height     = h;
        w             = FFALIGN(w, alignment);
        h             = FFALIGN(h, alignment);
        p->dwt_width  = w; /* TODO: do I need to store these? */
        p->dwt_height = h;

        p->coef_stride = w = FFALIGN(w, 32); /* TODO: is this stride needed? */
        p->coef_buf = av_mallocz(w*h*sizeof(dwtcoef));
        if (!p->coef_buf)
            goto alloc_fail;

        w = p->dwt_width;
        h = p->dwt_height;
        for (level = s->wavelet_depth-1; level >= 0; level--) {
            w = w >> 1;
            h = h >> 1;
            hstride <<= 1;
            for (o = 0; o < 4; o++) {
                SubBand *b = &p->band[level][o];
                b->width  = w;
                b->height = h;
                b->hstride = hstride;
                b->stride = p->coef_stride * hstride;
                shift = (o > 1)*(b->stride >> 1) + (o & 1)*(hstride >> 1);
                b->buf = p->coef_buf + shift;
            }
        }
    }

    /* Slices */
    s->num_x = s->plane[0].dwt_width/s->slice_width;
    s->num_y = s->plane[0].dwt_height/s->slice_height;

    if (s->fragment_size) {
        if (s->num_x % s->fragment_size) {
            av_log(avctx, AV_LOG_ERROR, "Fragment size (%d) is not a divisor of the number of slices across the frame (%d)\n",
                    s->fragment_size, s->num_x);
            return AVERROR(EINVAL);
        }
    }

    s->slice_args = av_calloc(s->num_x*s->num_y, sizeof(SliceArgs));
    if (!s->slice_args)
        goto alloc_fail;
    else {
        int x, y;
        for (y = 0; y < s->num_y; y++) {
            for (x = 0; x < s->num_x; x++) {
                SliceArgs *args = &s->slice_args[s->num_x * y + x];
                args->ctx = s;
                args->x   = x;
                args->y   = y;
                args->quant_idx = const_quant;
            }
        }
    }

    /* Lookup tables */
    s->coef_lut_len = av_malloc(COEF_LUT_TAB*(s->q_ceil+1)*sizeof(*s->coef_lut_len));
    if (!s->coef_lut_len)
        goto alloc_fail;

    s->coef_lut_val = av_malloc(COEF_LUT_TAB*(s->q_ceil+1)*sizeof(*s->coef_lut_val));
    if (!s->coef_lut_val)
        goto alloc_fail;

    for (i = 0; i < s->q_ceil; i++) {
        /* Value (what to put in) - without the sign bit (gets OR'd during encoding) */
        uint8_t  *len_lut = &s->coef_lut_len[i*COEF_LUT_TAB];
        /* Length (how long it is) - counts the sign bit in! */
        uint32_t *val_lut = &s->coef_lut_val[i*COEF_LUT_TAB];
        for (j = 0; j < COEF_LUT_TAB; j++) {
            get_vc2_ue_uint(QUANT(j, ff_dirac_qscale_tab[i]),
                            &len_lut[j], &val_lut[j]);
            if (len_lut[j] != 1) {
                len_lut[j] += 1;
                val_lut[j] <<= 1;
            } else {
                val_lut[j] = 1;
            }
        }
    }

    /* Quantization LUT layout: {
     *     [Coeff 0 for Quant 1, Coeff 1 for Quant 1 ...],
     *     [Coeff 0 for Quant 2, Coeff 1 for Quant 2 ...], }
     */
    init_quant_matrix(s);

    return 0;

alloc_fail:
    vc2_encode_end(avctx);
    av_log(avctx, AV_LOG_ERROR, "Unable to allocate memory!\n");
    return AVERROR(ENOMEM);
}

#define VC2ENC_FLAGS (AV_OPT_FLAG_ENCODING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption vc2enc_options[] = {
    {"tolerance",     "Max undershoot in percent", offsetof(VC2EncContext, tolerance), AV_OPT_TYPE_DOUBLE, {.dbl = 5.0f}, 0.0f, 45.0f, VC2ENC_FLAGS, "tolerance"},
    {"slice_width",   "Slice width",  offsetof(VC2EncContext, slice_width), AV_OPT_TYPE_INT, {.i64 = 32}, 32, 1024, VC2ENC_FLAGS, "slice_width"},
    {"slice_height",  "Slice height", offsetof(VC2EncContext, slice_height), AV_OPT_TYPE_INT, {.i64 = 16}, 4, 1024, VC2ENC_FLAGS, "slice_height"},
    {"wavelet_depth", "Transform depth", offsetof(VC2EncContext, wavelet_depth), AV_OPT_TYPE_INT, {.i64 = 4}, 1, 5, VC2ENC_FLAGS, "wavelet_depth"},
    {"wavelet_type",  "Transform type",  offsetof(VC2EncContext, wavelet_idx), AV_OPT_TYPE_INT, {.i64 = VC2_TRANSFORM_HAAR_S}, VC2_TRANSFORM_HAAR, VC2_TRANSFORM_HAAR_S, VC2ENC_FLAGS, "wavelet_idx"},
        {"haar",         "Haar (with shift)",       0, AV_OPT_TYPE_CONST, {.i64 = VC2_TRANSFORM_HAAR_S}, INT_MIN, INT_MAX, VC2ENC_FLAGS, "wavelet_idx"},
        {"haar_noshift", "Haar (without shift)",    0, AV_OPT_TYPE_CONST, {.i64 = VC2_TRANSFORM_HAAR},   INT_MIN, INT_MAX, VC2ENC_FLAGS, "wavelet_idx"},
    {"qm", "Custom quantization matrix", offsetof(VC2EncContext, quant_matrix), AV_OPT_TYPE_INT, {.i64 = VC2_QM_DEF}, 0, VC2_QM_NB, VC2ENC_FLAGS, "quant_matrix"},
        {"default",   "Default from the specifications", 0, AV_OPT_TYPE_CONST, {.i64 = VC2_QM_DEF}, INT_MIN, INT_MAX, VC2ENC_FLAGS, "quant_matrix"},
        {"color",     "Prevents low bitrate discoloration", 0, AV_OPT_TYPE_CONST, {.i64 = VC2_QM_COL}, INT_MIN, INT_MAX, VC2ENC_FLAGS, "quant_matrix"},
        {"flat",      "Optimize for PSNR", 0, AV_OPT_TYPE_CONST, {.i64 = VC2_QM_FLAT}, INT_MIN, INT_MAX, VC2ENC_FLAGS, "quant_matrix"},
    {"fragment_size", "Number of slices to put in each fragment", offsetof(VC2EncContext, fragment_size), AV_OPT_TYPE_INT, {.i64 = 1}, 1, INT_MAX, VC2ENC_FLAGS, "fragment_size"},
    {NULL}
};

static const AVClass vc2enc_class = {
    .class_name = "SMPTE VC-2 encoder",
    .category = AV_CLASS_CATEGORY_ENCODER,
    .option = vc2enc_options,
    .item_name = av_default_item_name,
    .version = LIBAVUTIL_VERSION_INT
};

static const AVCodecDefault vc2enc_defaults[] = {
    { "b",              "600000000"   },
    { NULL },
};

static const enum AVPixelFormat allowed_pix_fmts[] = {
    AV_PIX_FMT_YUV420P,   AV_PIX_FMT_YUV422P,   AV_PIX_FMT_YUV444P,
    AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV422P10, AV_PIX_FMT_YUV444P10,
    AV_PIX_FMT_YUV420P12, AV_PIX_FMT_YUV422P12, AV_PIX_FMT_YUV444P12,
    AV_PIX_FMT_NONE
};

AVCodec ff_vc2_encoder = {
    .name           = "vc2",
    .long_name      = NULL_IF_CONFIG_SMALL("SMPTE VC-2"),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_DIRAC,
    .priv_data_size = sizeof(VC2EncContext),
    .init           = vc2_encode_init,
    .close          = vc2_encode_end,
    .capabilities   = AV_CODEC_CAP_SLICE_THREADS,
    .caps_internal  = FF_CODEC_CAP_INIT_THREADSAFE,
    .encode2        = vc2_encode_frame,
    .priv_class     = &vc2enc_class,
    .defaults       = vc2enc_defaults,
    .pix_fmts       = allowed_pix_fmts
};
