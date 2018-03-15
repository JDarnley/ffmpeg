/*
 * Copyright (C) 2018 Open Broadcast Systems Ltd.
 * Author        2018 James Darnley <jdarnley@obe.tv>
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

#include "dirac.h"
#include "vc2enc_dwt.h"
#include "vc2enc_new_dwt.h"

static inline void haar_horizontal_compose(dwtcoef *line, dwtcoef *temp,
        int width, ptrdiff_t stride, const int shift)
{
    int x;
    for (x = 0; x < width; x += 2) {
        temp[x+1] = (line[x+1] << shift) - (line[x] << shift);
        temp[x]   = (line[x] << shift) + (temp[x+1] + 1 >> 1);
    }
}

static void haar_shift_horizontal_compose(dwtcoef *line, dwtcoef *temp,
        int width, ptrdiff_t stride)
{
    haar_horizontal_compose(line, temp, width, stride, 1);
}

static void haar_noshift_horizontal_compose(dwtcoef *line, dwtcoef *temp,
        int width, ptrdiff_t stride)
{
    haar_horizontal_compose(line, temp, width, stride, 0);
}

static void haar_vertical_compose(dwtcoef *line0, dwtcoef *line1,
        dwtcoef *temp0, dwtcoef *temp1,
        int width, ptrdiff_t stride)
{
    int x;
    for (x = 0; x < width; x++) {
        line1[x] = temp1[x] - temp0[x];
        line0[x] = temp0[x] + (line1[x] + 1 >> 1);
    }
}

static void haar_compose(struct VC2NewDWTContext *d, struct VC2NewDWTCompose *cs,
        int width, int height, ptrdiff_t stride, ptrdiff_t hstride)
{
    int y = cs->y;
    dwtcoef *b0 = d->buffer + (y-1)*stride;
    dwtcoef *b1 = d->buffer + (y  )*stride;
    dwtcoef *t0 = d->temp;
    dwtcoef *t1 = d->temp + width;

    haar_shift_horizontal_compose(b0, t0, width, stride);
    haar_shift_horizontal_compose(b1, t1, width, stride);
    haar_vertical_compose(b0, b1, t0, t1, width, stride);

    cs->y += 2;
}

int ff_vc2enc_new_dwt_reset(struct VC2NewDWTContext *d, struct VC2NewDWTPlane *p,
        enum VC2TransformType type, int decomposition_count)
{
    int level;
    int ret = 0;

    d->type = p->type = type;
    d->buffer = p->buf;
    d->width  = p->width;
    d->height = p->height;
    d->stride = p->stride;
    d->temp   = p->tmp + 8;
    d->decomposition_count = decomposition_count;

    for (level = d->decomposition_count - 1; level >= 0; level--) {
        int height_l = d->height >> level;
        int stride_l = d->stride << level;

        switch(type) {
            case VC2_TRANSFORM_HAAR:
            case VC2_TRANSFORM_HAAR_S:
                d->cs[level].y = 1;
                break;
        }
    }

    switch(type) {
        case VC2_TRANSFORM_HAAR:
        case VC2_TRANSFORM_HAAR_S:
            d->support = 1; /* Why is this 1? */
        break;

        default:
            ret = AVERROR(EINVAL);
    }

    if (ret) {
        av_log(NULL, AV_LOG_ERROR, "Unknown wavelet type %d\n", type);
    }

    return ret;
}

void ff_vc2enc_new_dwt_transform(struct VC2NewDWTContext *d, int y)
{
    int level, support = d->support;

    for (level = 0; level < d->decomposition_count; level++) {
        int width_l   = d->width >> level;
        int height_l  = d->height >> level;
        int stride_l  = d->stride << level;
        int hstride_l = 1 << level;

        while(d->cs[level].y <= FFMIN((y >> level) + support, height_l))
            haar_compose(d, &d->cs[level], width_l, height_l, stride_l, hstride_l);
    }
}
