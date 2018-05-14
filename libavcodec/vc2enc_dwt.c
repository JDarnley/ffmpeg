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

static void legall_5_3_transform(dwtcoef *data, ptrdiff_t stride,
        int width, int height, int hstride, int y, struct progress *progress)
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

    /* Fudge it for the progress use in vc2enc.c */
    if (y == synth_height)
        progress->vfilter_stage1 = y;
    else
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

static void haar_transform(dwtcoef *data,
        ptrdiff_t stride, int width, int height, int hstride,
        int y, struct progress *progress,
        const int shift)
{
    y &= ~1;
    if (y < progress->vfilter_stage1 + 2)
        return;
    data += stride * progress->vfilter_stage1;
    dwt_haar(data, stride, width, y-progress->vfilter_stage1, hstride, shift);
    progress->vfilter_stage1 = y;
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

                deslauriers_dubuc_9_7_transform(data, stride_l,
                        width_l/2, height_l/2,
                        hstride, y_l, &t->progress[level]);

                    y_l = t->progress[level].vfilter_stage1 / 2 & ~1;
            }
            break;

        case VC2_TRANSFORM_5_3:
            for (level = 0; level < depth; level++) {
                ptrdiff_t stride_l = stride << level;
                int width_l = width >> level;
                int height_l = height >> level;
                int hstride = 1 << level;

                legall_5_3_transform(data, stride_l,
                        width_l/2, height_l/2,
                        hstride, y_l, &t->progress[level]);

                y_l = t->progress[level].vfilter_stage1/2 & ~1;
            }
            break;

        case VC2_TRANSFORM_HAAR:
            for (level = 0; level < depth; level++) {
                int hstride = 1 << level;
                haar_transform(data, stride << level,
                        width >> level, height >> level,
                        hstride, y >> level, &t->progress[level], 0);
            }
            break;

        case VC2_TRANSFORM_HAAR_S:
            for (level = 0; level < depth; level++) {
                int hstride = 1 << level;
                haar_transform(data, stride << level,
                        width >> level, height >> level,
                        hstride, y >> level, &t->progress[level], 1);
            }
            break;
    }
}
