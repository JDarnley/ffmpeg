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
