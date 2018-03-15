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

#ifndef AVCODEC_VC2ENC_NEW_DWT_H
#define AVCODEC_VC2ENC_NEW_DWT_H

#define MAX_DWT_SUPPORT 8
//#define MAX_COMPOSITIONS 8

struct VC2NewDWTCompose {
    dwtcoef *b[MAX_DWT_SUPPORT];
    int y;
};

struct VC2NewDWTPlane {
    dwtcoef *buf;
    dwtcoef *buf_base;
    dwtcoef *tmp;
    ptrdiff_t stride;
    int width, height;
    enum VC2TransformType type;
};

struct VC2NewDWTContext {
    dwtcoef *buffer;
    dwtcoef *temp;
    ptrdiff_t stride;
    int width, height;
    int decomposition_count;
    int support;
    enum VC2TransformType type;

    struct VC2NewDWTCompose cs[MAX_DWT_LEVELS];
};

int ff_vc2enc_new_dwt_reset(struct VC2NewDWTContext *d, struct VC2NewDWTPlane *p,
        enum VC2TransformType type, int decomposition_count);

void ff_vc2enc_new_dwt_transform(struct VC2NewDWTContext *d, int y);

#endif /* AVCODEC_VC2ENC_NEW_DWT_H */
