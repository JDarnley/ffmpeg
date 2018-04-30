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
    int level;

    switch (type) {
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
