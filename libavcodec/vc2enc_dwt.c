/*
 * Copyright (C) 2007 Marco Gerards <marco@gnu.org>
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

#include "libavutil/attributes.h"
#include "libavutil/mem.h"
#include "dirac.h"
#include "vc2enc_dwt.h"

/* Deslauriers-Dubuc (9,7), VC2_TRANSFORM_9_7  */

#define LIFT1(val0, val1, val2)\
    ((val1) + ((val0) + (val2) + 2 >> 2))

#define LIFT2(val0, val1, val2, val3, val4) \
    ((val2) - (-(val0) + 9*(val1) + 9*(val3) - (val4) + 8 >> 4))

static void vc2_subband_dwt_97(VC2TransformContext *t, dwtcoef *data,
                               ptrdiff_t stride, int width, int height,
                               const ptrdiff_t hstride)
{
    int x, y;
    dwtcoef *data_original = data;
    const ptrdiff_t synth_width  = width  << 1;
    const ptrdiff_t synth_height = height << 1;

    /* Horizontal synthesis. */
    for (y = 0; y < synth_height; y++) {
        /* Lifting stage 2. */
        data[hstride] = LIFT2(data[0] << 1,
                              data[0] << 1,
                              data[1*hstride] << 1,
                              data[2*hstride] << 1,
                              data[4*hstride] << 1);
        for (x = 1; x < width - 2; x++)
            data[(2*x + 1)*hstride] = LIFT2(data[(2*x - 2)*hstride] << 1,
                                            data[(2*x    )*hstride] << 1,
                                            data[(2*x + 1)*hstride] << 1,
                                            data[(2*x + 2)*hstride] << 1,
                                            data[(2*x + 4)*hstride] << 1);
        data[(2*x + 1)*hstride] = LIFT2(data[(2*x - 2)*hstride] << 1,
                                        data[(2*x    )*hstride] << 1,
                                        data[(2*x + 1)*hstride] << 1,
                                        data[(2*x + 2)*hstride] << 1,
                                        data[(2*x + 2)*hstride] << 1);
        data[(2*x + 3)*hstride] = LIFT2(data[(2*x    )*hstride] << 1,
                                        data[(2*x + 2)*hstride] << 1,
                                        data[(2*x + 3)*hstride] << 1,
                                        data[(2*x + 2)*hstride] << 1,
                                        data[(2*x + 2)*hstride] << 1);

        /* Lifting stage 1. */
        data[0] = LIFT1(data[hstride], data[0] << 1, data[hstride]);
        for (x = 1; x < width; x++)
            data[2*x*hstride] = LIFT1(data[(2*x-1)*hstride],
                                      data[(2*x  )*hstride] << 1,
                                      data[(2*x+1)*hstride]);

        data += stride;
    }

    /* Vertical synthesis: Lifting stage 2. */
    data = data_original;
    for (x = 0; x < synth_width; x++)
        data[x*hstride + stride] = LIFT2(data[x*hstride],
                                         data[x*hstride],
                                         data[x*hstride + stride],
                                         data[x*hstride + 2*stride],
                                         data[x*hstride + 4*stride]);
    for (y = 1; y < height - 2; y++) {
        for (x = 0; x < synth_width; x++)
            data[x*hstride + 3*stride] = LIFT2(data[x*hstride],
                                               data[x*hstride + 2*stride],
                                               data[x*hstride + 3*stride],
                                               data[x*hstride + 4*stride],
                                               data[x*hstride + 6*stride]);
        data += stride*2;
    }
    for (x = 0; x < synth_width; x++) {
        data[x*hstride + 3*stride] = LIFT2(data[x*hstride],
                                           data[x*hstride + 2*stride],
                                           data[x*hstride + 3*stride],
                                           data[x*hstride + 4*stride],
                                           data[x*hstride + 4*stride]);
        data[x*hstride + 5*stride] = LIFT2(data[x*hstride + 2*stride],
                                           data[x*hstride + 4*stride],
                                           data[x*hstride + 5*stride],
                                           data[x*hstride + 4*stride],
                                           data[x*hstride + 4*stride]);
    }

    /* Vertical synthesis: Lifting stage 1. */
    data = data_original;
    for (x = 0; x < synth_width; x++)
        data[x*hstride] = LIFT1(data[x*hstride + stride],
                                data[x*hstride],
                                data[x*hstride + stride]);
    data = data_original + stride;
    for (y = 1; y < height; y++) {
        for (x = 0; x < synth_width; x++)
            data[x*hstride + stride] = LIFT1(data[x*hstride],
                                             data[x*hstride + stride],
                                             data[x*hstride + stride*2]);
        data += stride*2;
    }
}

static void deslauriers_dubuc_9_7_transform(dwtcoef *data,
        ptrdiff_t stride, int width, int height, int hstride,
        const int y, struct progress *progress)
{
    int x, line, line_max;
    dwtcoef *data_original = data;
    const ptrdiff_t synth_width  = width  << 1;
    const ptrdiff_t synth_height = height << 1;

    /* Horizontal synthesis. */
    data = data_original + stride*progress->hfilter;
    for (line = progress->hfilter; line < y; line++) {
        /* Lifting stage 2. */
        data[hstride] = LIFT2(data[0] << 1,
                              data[0] << 1,
                              data[1*hstride] << 1,
                              data[2*hstride] << 1,
                              data[4*hstride] << 1);
        for (x = 1; x < width - 2; x++)
            data[(2*x + 1)*hstride] = LIFT2(data[(2*x - 2)*hstride] << 1,
                                            data[(2*x    )*hstride] << 1,
                                            data[(2*x + 1)*hstride] << 1,
                                            data[(2*x + 2)*hstride] << 1,
                                            data[(2*x + 4)*hstride] << 1);
        data[(2*x + 1)*hstride] = LIFT2(data[(2*x - 2)*hstride] << 1,
                                        data[(2*x    )*hstride] << 1,
                                        data[(2*x + 1)*hstride] << 1,
                                        data[(2*x + 2)*hstride] << 1,
                                        data[(2*x + 2)*hstride] << 1);
        data[(2*x + 3)*hstride] = LIFT2(data[(2*x    )*hstride] << 1,
                                        data[(2*x + 2)*hstride] << 1,
                                        data[(2*x + 3)*hstride] << 1,
                                        data[(2*x + 2)*hstride] << 1,
                                        data[(2*x + 2)*hstride] << 1);

        /* Lifting stage 1. */
        data[0] = LIFT1(data[hstride], data[0] << 1, data[hstride]);
        for (x = 1; x < width; x++)
            data[2*x*hstride] = LIFT1(data[(2*x-1)*hstride],
                                      data[(2*x  )*hstride] << 1,
                                      data[(2*x+1)*hstride]);

        data += stride;
    }
    progress->hfilter = line;

    /* Vertical synthesis: Lifting stage 2. */
    data = data_original;
    line_max = y - 4;
    line = progress->vfilter_stage2;
    if (line == 0 && line_max > 0) {
        for (x = 0; x < synth_width; x++)
            data[x*hstride + stride] = LIFT2(data[x*hstride],
                    data[x*hstride],
                    data[x*hstride + stride],
                    data[x*hstride + 2*stride],
                    data[x*hstride + 4*stride]);
        line += 2;
    }
    data += stride*(line - 2);
    for (/* do nothing */; line < line_max; line += 2) {
        for (x = 0; x < synth_width; x++)
            data[x*hstride + 3*stride] = LIFT2(data[x*hstride],
                                               data[x*hstride + 2*stride],
                                               data[x*hstride + 3*stride],
                                               data[x*hstride + 4*stride],
                                               data[x*hstride + 6*stride]);
        data += stride*2;
    }
    if (line == synth_height - 4) {
        for (x = 0; x < synth_width; x++) {
            data[x*hstride + 3*stride] = LIFT2(data[x*hstride],
                                               data[x*hstride + 2*stride],
                                               data[x*hstride + 3*stride],
                                               data[x*hstride + 4*stride],
                                               data[x*hstride + 4*stride]);
            data[x*hstride + 5*stride] = LIFT2(data[x*hstride + 2*stride],
                                               data[x*hstride + 4*stride],
                                               data[x*hstride + 5*stride],
                                               data[x*hstride + 4*stride],
                                               data[x*hstride + 4*stride]);
        }
        line += 4;
    }
    progress->vfilter_stage2 = line;

    /* Vertical synthesis: Lifting stage 1. */
    line_max = line;
    if (y != synth_height)
        line_max -= 2;
    line = progress->vfilter_stage1;
    data = data_original;
    if (line == 0 && line_max > 0) {
        for (x = 0; x < synth_width; x++)
            data[x*hstride] = LIFT1(data[x*hstride + stride],
                                    data[x*hstride],
                                    data[x*hstride + stride]);
        line += 2;
    }

    data += line*stride - stride;
    for (; line < line_max; line += 2) {
        for (x = 0; x < synth_width; x++)
            data[x*hstride + stride] = LIFT1(data[x*hstride],
                                             data[x*hstride + stride],
                                             data[x*hstride + stride*2]);
        data += stride*2;
    }
    progress->vfilter_stage1 = line;
}

#undef LIFT1
#undef LIFT2

/* LeGall (5,3), VC2_TRANSFORM_5_3 */

#define LIFT1(val0, val1, val2)\
    ((val1) + ((val0) + (val2) + 2 >> 2))

#define LIFT2(val0, val1, val2)\
    ((val1) - ((val0) + (val2) + 1 >> 1))

static void vc2_subband_dwt_53(VC2TransformContext *t, dwtcoef *data,
                               ptrdiff_t stride, int width, int height,
                               const ptrdiff_t hstride)
{
    int x, y;
    dwtcoef *data_original = data;
    const ptrdiff_t synth_width  = width  << 1;
    const ptrdiff_t synth_height = height << 1;

    /* Horizontal synthesis. */
    data = data_original;
    for (y = 0; y < synth_height; y++) {
        /* Lifting stage 2. */
        for (x = 0; x < width - 1; x++)
            data[(2*x+1)*hstride] = LIFT2(data[(2*x  )*hstride] << 1,
                                          data[(2*x+1)*hstride] << 1,
                                          data[(2*x+2)*hstride] << 1);
        data[(2*x+1)*hstride] = LIFT2(data[(2*x  )*hstride] << 1,
                                      data[(2*x+1)*hstride] << 1,
                                      data[(2*x  )*hstride] << 1);

        /* Lifting stage 1. */
        data[0] = LIFT1(data[hstride], data[0] << 1, data[hstride]);
        for (x = 1; x < width; x++)
            data[2*x*hstride] = LIFT1(data[(2*x-1)*hstride],
                                      data[(2*x  )*hstride] << 1,
                                      data[(2*x+1)*hstride]);

        data += stride;
    }

    /* Vertical synthesis: Lifting stage 2. */
    data = data_original;
    for (y = 0; y < height - 1; y++) {
        for (x = 0; x < synth_width; x++)
            data[x*hstride+stride] = LIFT2(data[x*hstride],
                                           data[x*hstride + stride],
                                           data[x*hstride + stride*2]);
        data += stride*2;
    }
    for (x = 0; x < synth_width; x++)
        data[x*hstride + stride] = LIFT2(data[x*hstride],
                                         data[x*hstride + stride],
                                         data[x*hstride]);

    /* Vertical synthesis: Lifting stage 1. */
    data = data_original;
    for (x = 0; x < synth_width; x++)
        data[x*hstride] = LIFT1(data[x*hstride + stride],
                                data[x*hstride],
                                data[x*hstride + stride]);
    data = data_original + stride;
    for (y = 1; y < height; y++) {
        for (x = 0; x < synth_width; x++)
            data[x*hstride + stride] = LIFT1(data[x*hstride],
                                             data[x*hstride + stride],
                                             data[x*hstride + stride*2]);
        data += stride*2;
    }
}

static void legall_5_3_transform(dwtcoef *data,
        ptrdiff_t stride, int width, int height, int hstride,
        int y, struct progress *progress
        )
{
    int x, line, line_max;
    dwtcoef *data_original = data;
    const ptrdiff_t synth_width  = width  << 1;
    const ptrdiff_t synth_height = height << 1;

    /* Horizontal synthesis. */
    data = data_original + stride*progress->hfilter;
    for (line = progress->hfilter; line < y; line++) {
        /* Lifting stage 2. */
        for (x = 0; x < width - 1; x++)
            data[(2*x+1)*hstride] = LIFT2(data[(2*x  )*hstride] << 1,
                                          data[(2*x+1)*hstride] << 1,
                                          data[(2*x+2)*hstride] << 1);
        data[(2*x+1)*hstride] = LIFT2(data[(2*x  )*hstride] << 1,
                                      data[(2*x+1)*hstride] << 1,
                                      data[(2*x  )*hstride] << 1);

        /* Lifting stage 1. */
        data[0] = LIFT1(data[hstride], data[0] << 1, data[hstride]);
        for (x = 1; x < width; x++)
            data[2*x*hstride] = LIFT1(data[(2*x-1)*hstride],
                                      data[(2*x  )*hstride] << 1,
                                      data[(2*x+1)*hstride]);

        data += stride;
    }
    progress->hfilter = line;

    /* Vertical synthesis: Lifting stage 2. */
    data = data_original + stride*progress->vfilter_stage2;
    line_max = y - 2;
    for (line = progress->vfilter_stage2; line < line_max; line += 2) {
        for (x = 0; x < synth_width; x++)
            data[x*hstride+stride] = LIFT2(data[x*hstride],
                                           data[x*hstride + stride],
                                           data[x*hstride + stride*2]);
        data += stride*2;
    }
    // line1 = line1 - line0 + line2, line=0
    // line3 = line3 - line2 + line4, line=1
    // line5 = line5 - line4 + line6, line=2
    // line7 = line7 - line6 + line8, line=3
    // line9 = line9 - line8 + line10, line=4
    // line11 = line11 - line10 + line12, line=5
    // line13 = line13 - line12 + line14, line=6

    // line15 = line15 - line14 + line16, line=7
    if (line == synth_height - 2) {
        for (x = 0; x < synth_width; x++)
            data[x*hstride + stride] = LIFT2(data[x*hstride],
                                             data[x*hstride + stride],
                                             data[x*hstride]);
        line += 2;
    }
    progress->vfilter_stage2 = line;

    /* Vertical synthesis: Lifting stage 1. */
    line_max = line;
    line = progress->vfilter_stage1;
    data = data_original;
    if (line == 0 && line_max > 0) {
        for (x = 0; x < synth_width; x++)
            data[x*hstride] = LIFT1(data[x*hstride + stride],
                                    data[x*hstride],
                                    data[x*hstride + stride]);
        line += 2;
    }

    data += line*stride - stride;
    for (; line < line_max; line += 2) {
        for (x = 0; x < synth_width; x++)
            data[x*hstride + stride] = LIFT1(data[x*hstride],
                                             data[x*hstride + stride],
                                             data[x*hstride + stride*2]);
        data += stride*2;
    }
    // line0 = line0 + line1 + line1, line=0
    // line2 = line2 + line3 + line1, line=1
    // line4 = line4 + line5 + line3, line=2
    // line6 = line6 + line7 + line5, line=3
    // line8 = line8 + line9 + line7, line=4
    // line10 = line10 + line11 + line9, line=5
    // line12 = line12 + line13 + line11, line=6

    // line14 = line14 + line15 + line13, line=7

    // line16 = line16 + line17 + line15, line=8
    progress->vfilter_stage1 = line;
}

#undef LIFT1
#undef LIFT2

/* Haar, VC2_TRANSFORM_HAAR, VC2_TRANSFORM_HAAR_S */

static av_always_inline void dwt_haar(dwtcoef *data,
                                      ptrdiff_t stride, int width, int height,
                                      const ptrdiff_t hstride,
                                      const int s)
{
    int x, y;
    for (y = 0; y < height; y += 2) {
        /* Horizontal synthesis. */
        for (x = 0; x < width; x += 2) {
            data[y*stride + (x+1)*hstride] = (data[y*stride + (x+1)*hstride] << s) -
                                             (data[y*stride + x*hstride] << s);
            data[y*stride + x*hstride] = (data[y*stride + x*hstride] << s) +
                                        ((data[y*stride + (x+1)*hstride] + 1) >> 1);
        }
        for (x = 0; x < width; x += 2) {
            data[(y+1)*stride + (x+1)*hstride] = (data[(y+1)*stride + (x+1)*hstride] << s) -
                                                 (data[(y+1)*stride + x*hstride] << s);
            data[(y+1)*stride + x*hstride] = (data[(y+1)*stride + x*hstride] << s) +
                                            ((data[(y+1)*stride + (x+1)*hstride] + 1) >> 1);
        }

        /* Vertical synthesis. */
        for (x = 0; x < width; x++) {
            data[(y + 1)*stride + x*hstride] = data[(y + 1)*stride + x*hstride] -
                                               data[y*stride + x*hstride];
            data[y*stride + x*hstride] = data[y*stride + x*hstride] +
                                       ((data[(y + 1)*stride + x*hstride] + 1) >> 1);
        }
    }
}

static void vc2_subband_dwt_haar(VC2TransformContext *t, dwtcoef *data,
                                 ptrdiff_t stride, int width, int height,
                                 const ptrdiff_t hstride)
{
    dwt_haar(data, stride, width, height, hstride, 0);
}

static void vc2_subband_dwt_haar_shift(VC2TransformContext *t, dwtcoef *data,
                                       ptrdiff_t stride, int width, int height,
                                       const ptrdiff_t hstride)
{
    dwt_haar(data, stride, width, height, hstride, 1);
}

static void haar_transform(dwtcoef *data,
        ptrdiff_t stride, int width, int height, int hstride,
        int y, struct progress *progress,
        const int shift)
{
    data += stride * progress->vfilter_stage1;
    dwt_haar(data, stride, width, y-progress->vfilter_stage1, hstride, shift);
    progress->vfilter_stage1 = y;
}

av_cold int ff_vc2enc_init_transforms(VC2TransformContext *s, int p_stride,
                                      int p_width, int p_height,
                                      int slice_w, int slice_h)
{
    s->vc2_subband_dwt[VC2_TRANSFORM_9_7]    = vc2_subband_dwt_97;
    s->vc2_subband_dwt[VC2_TRANSFORM_5_3]    = vc2_subband_dwt_53;
    s->vc2_subband_dwt[VC2_TRANSFORM_HAAR]   = vc2_subband_dwt_haar;
    s->vc2_subband_dwt[VC2_TRANSFORM_HAAR_S] = vc2_subband_dwt_haar_shift;

    /* Pad by the slice size, only matters for non-Haar wavelets */
    s->buffer = av_calloc((p_stride + slice_w)*(p_height + slice_h), sizeof(dwtcoef));
    if (!s->buffer)
        return 1;

    s->padding = (slice_h >> 1)*p_stride + (slice_w >> 1);
    s->buffer += s->padding;

    return 0;
}

void ff_vc2enc_reset_transforms(VC2TransformContext *s)
{
    int i;
    for (i = 0; i < MAX_DWT_LEVELS; i++) {
        s->progress[i].hfilter =
            s->progress[i].vfilter_stage1 =
            s->progress[i].vfilter_stage2 = 0;
    }
}

void ff_vc2enc_transform(VC2TransformContext *t, dwtcoef *data,
        ptrdiff_t stride, int width, int height,
        int y, const int depth, const enum VC2TransformType type)
{
    int level, y_l = y;

    switch (type) {
        case VC2_TRANSFORM_9_7:
            for (level = 0; level < depth; level++) {
                ptrdiff_t stride_l = stride << level;
                int width_l = width >> level;
                int height_l = height >> level;
                int hstride = 1 << level;
                y_l &= ~1;

                deslauriers_dubuc_9_7_transform(data, stride_l,
                        width_l/2, height_l/2,
                        hstride, y_l, &t->progress[level]);

                    y_l = t->progress[level].vfilter_stage1 / 2;
            }
            break;

        case VC2_TRANSFORM_5_3:
            for (level = 0; level < depth; level++) {
                ptrdiff_t stride_l = stride << level;
                int width_l = width >> level;
                int height_l = height >> level;
                int hstride = 1 << level;

                if (y_l != height_l
                        && y_l < t->progress[level].hfilter + 2)
                    break;

                legall_5_3_transform(data, stride_l,
                        width_l/2, height_l/2,
                        hstride, y_l, &t->progress[level]);

                if (y == height)
                    y_l /= 2;
                else
                    y_l = t->progress[level].vfilter_stage1/2;
            }
            break;

        case VC2_TRANSFORM_HAAR:
            for (level = 0; level < depth; level++) {
                int hstride = 1 << level;
                int y_l = (y >> level) & ~1;
                haar_transform(data, stride << level,
                        width >> level, height >> level,
                        hstride, y_l, &t->progress[level], 0);
            }
            break;

        case VC2_TRANSFORM_HAAR_S:
            for (level = 0; level < depth; level++) {
                int hstride = 1 << level;
                int y_l = (y >> level) & ~1;
                haar_transform(data, stride << level,
                        width >> level, height >> level,
                        hstride, y_l, &t->progress[level], 1);
            }
            break;
    }
}

av_cold void ff_vc2enc_free_transforms(VC2TransformContext *s)
{
    av_free(s->buffer - s->padding);
    s->buffer = NULL;
}
