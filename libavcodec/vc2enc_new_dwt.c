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

/* LeGall (5,3), VC2_TRANSFORM_5_3 */

#define legall5_3_h_lift1(val0, val1, val2)\
    ((val1) + ((val0) + (val2) + 2 >> 2))

#define legall5_3_h_lift2(val0, val1, val2)\
    ((val1) - ((val0) + (val2) + 1 >> 1))

static inline void legall5_3_horizontal_compose(dwtcoef *line,
        int width, ptrdiff_t stride)
{
    int x;
    int w2 = width / 2;

    /* TODO: should the extra shift-of-1 be done inline or separately? */
    /* TODO: see if this is faster with some temporary variables. */

    /* Lifting stage 2. */
    for (x = 0; x < w2-1; x++)
        line[2*x+1] = legall5_3_h_lift2(line[2*x] << 1, line[2*x+1] << 1, line[2*x+2] << 1);
    line[width-1] = legall5_3_h_lift2(line[width-2] << 1, line[width-1] << 1, line[width-2] << 1);

    /* Lifting stage 1. */
    line[0] = legall5_3_h_lift1(line[1], line[0] << 1, line[1]);
    for (x = 1; x < w2-1; x++)
        line[2*x] = legall5_3_h_lift1(line[2*x-1], line[2*x] << 1, line[2*x+1]);
    line[width-2] = legall5_3_h_lift1(line[width-3], line[width-2] << 1, line[width-1]);
}

static inline void legall5_3_vertical_compose1(/* dwtcoef *line0, dwtcoef *line1, dwtcoef *line2, */
        dwtcoef *temp0, dwtcoef *temp1, dwtcoef *temp2
        int width, ptrdiff_t stride)
{
    int x;
    for (x = 0; x < width; x++)
        temp1[x] = legall5_3_h_lift1(temp0[x], temp1[x], temp2[x]);
}

static inline void legall5_3_vertical_compose2(/* dwtcoef *line0, dwtcoef *line1, dwtcoef *line2, */
        dwtcoef *temp0, dwtcoef *temp1, dwtcoef *temp2
        int width, ptrdiff_t stride)
{
    int x;
    for (x = 0; x < width; x++)
        temp1[x] = legall5_3_h_lift2(temp0[x], temp1[x], temp2[x]);
}

static void legaal5_3_compose(struct VC2NewDWTContext *d, struct VC2NewDWTCompose *cs,
        int width, int height, ptrdiff_t stride, ptrdiff_t hstride)
{
    int y = cs->y;
    dwtcoef *line[4] = {
        d->buffer + avpriv_mirror(y-1, height-1)*stride,
        d->buffer,
        d->buffer + avpriv_mirror(y+1, height-1)*stride,
        d->buffer + avpriv_mirror(y+2, height-1)*stride,
    };
    dwtcoef *temp[4] = {
        d->temp + 1*width,
        d->temp,
        d->temp + 1*width,
        d->temp + 2*width,
        d->temp + 3*width,
    };

    legall5_3_horizontal_compose(line[0], temp[1], width);
    legall5_3_horizontal_compose(line[1], temp[2], width);
    legall5_3_horizontal_compose(line[2], temp[3], width);
    legall5_3_horizontal_compose(line[3], temp[4], width);

    legall5_3_vertical_compose2(t[2], t[3], t[4]);
    legall5_3_vertical_compose1(t[1], t[2], t[3]);

    cs->y += 2;
}

/* Haar, with and without shift, VC2_TRANSFORM_HAAR, VC2_TRANSFORM_HAAR_S */

static inline void haar_horizontal_compose(dwtcoef *line, dwtcoef *temp,
        int width, const int shift)
{
    int x;
    for (x = 0; x < width; x += 2) {
        dwtcoef val = (line[x+1] << shift) - (line[x] << shift);
        temp[x+1]   = val;
        temp[x]     = (line[x] << shift) + (val + 1 >> 1);
    }
}

static inline void haar_vertical_compose(dwtcoef *line0, dwtcoef *line1,
        dwtcoef *temp0, dwtcoef *temp1,
        int width, ptrdiff_t stride)
{
    int x;
    for (x = 0; x < width; x++) {
        dwtcoef val = temp1[x] - temp0[x];
        line1[x]    = val;
        line0[x]    = temp0[x] + (val + 1 >> 1);
    }
}

static inline void haar_compose(struct VC2NewDWTContext *d, struct VC2NewDWTCompose *cs,
        int width, int height, ptrdiff_t stride, ptrdiff_t hstride, const int shift)
{
    int y = cs->y;
    dwtcoef *b0 = d->buffer + (y-1)*stride;
    dwtcoef *b1 = d->buffer + (y  )*stride;
    dwtcoef *t0 = d->temp;
    dwtcoef *t1 = d->temp + width;

    haar_horizontal_compose(b0, t0, width, shift);
    haar_horizontal_compose(b1, t1, width, shift);
    haar_vertical_compose(b0, b1, t0, t1, width, stride);

    cs->y += 2;
}

static void haar_shift_compose(struct VC2NewDWTContext *d, struct VC2NewDWTCompose *cs,
        int width, int height, ptrdiff_t stride, ptrdiff_t hstride)
{
    haar_compose(d, cs, width, height, stride, hstride, 1);
}

static void haar_noshift_compose(struct VC2NewDWTContext *d, struct VC2NewDWTCompose *cs,
        int width, int height, ptrdiff_t stride, ptrdiff_t hstride)
{
    haar_compose(d, cs, width, height, stride, hstride, 0);
}

void ff_vc2enc_new_dwt_reset(struct VC2NewDWTContext *d)
{
    int level;

    for (level = d->decomposition_count - 1; level >= 0; level--) {
        int height_l = d->height >> level;
        int stride_l = d->stride << level;

        switch(d->type) {
            case VC2_TRANSFORM_5_3:
                d->cs[level].y = -1;
                break;

            case VC2_TRANSFORM_HAAR:
            case VC2_TRANSFORM_HAAR_S:
                d->cs[level].y = 1;
                break;
        }
    }
}

int ff_vc2enc_new_dwt_init(void *logctx,
        struct VC2NewDWTContext *d, struct VC2NewDWTPlane *p,
        enum VC2TransformType type, int decomposition_count)
{
    int ret = 0;

    d->type = p->type = type;
    d->buffer = p->buf;
    d->width  = p->width;
    d->height = p->height;
    d->stride = p->stride;
    d->temp   = p->tmp + 8;
    d->decomposition_count = decomposition_count;

    switch (type) {
        case VC2_TRANSFORM_5_3:
            d->compose = legaal5_3_compose;
            d->support = 3;
            break;

        case VC2_TRANSFORM_HAAR:
            d->compose = haar_noshift_compose;
            d->support = 1; /* Why is this 1? */
            break;

        case VC2_TRANSFORM_HAAR_S:
            d->compose = haar_shift_compose;
            d->support = 1; /* Why is this 1? */
            break;

        default:
            av_log(logctx, AV_LOG_ERROR, "Unknown wavelet type %d\n", type);
            ret = AVERROR(EINVAL);
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
            d->compose(d, &d->cs[level], width_l, height_l, stride_l, hstride_l);
    }
}
