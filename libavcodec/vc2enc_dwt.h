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

#ifndef AVCODEC_VC2ENC_DWT_H
#define AVCODEC_VC2ENC_DWT_H

#include <stddef.h>
#include <stdint.h>

#include "dirac.h"

typedef int32_t dwtcoef;

enum VC2TransformType {
    VC2_TRANSFORM_9_7    = 0,   /* Deslauriers-Dubuc (9,7)  */
    VC2_TRANSFORM_5_3    = 1,   /* LeGall (5,3)             */
    VC2_TRANSFORM_13_7   = 2,   /* Deslauriers-Dubuc (13,7) */
    VC2_TRANSFORM_HAAR   = 3,   /* Haar without shift       */
    VC2_TRANSFORM_HAAR_S = 4,   /* Haar with 1 shift/lvl    */
    VC2_TRANSFORM_FIDEL  = 5,   /* Fidelity filter          */
    VC2_TRANSFORM_9_7_I  = 6,   /* Daubechies (9,7)         */

    VC2_TRANSFORMS_NB
};

typedef struct VC2TransformContext {
    dwtcoef *buffer;
    int padding;
    struct progress {
        int hfilter, vfilter_stage1, vfilter_stage2, deinterleave;
    } progress[MAX_DWT_LEVELS];
    void (*haar_block)(dwtcoef *data, ptrdiff_t stride, int width, int height);
} VC2TransformContext;

int  ff_vc2enc_init_transforms(VC2TransformContext *t, int p_stride, int p_height,
                               int slice_w, int slice_h);
void ff_vc2enc_reset_transforms(VC2TransformContext *s);
void ff_vc2enc_transform(VC2TransformContext *t, dwtcoef *data,
        ptrdiff_t stride, int width, int height,
        int y, const int depth, const enum VC2TransformType type);
void ff_vc2enc_free_transforms(VC2TransformContext *t);

void ff_vc2enc_init_transforms_x86(VC2TransformContext *s);

#endif /* AVCODEC_VC2ENC_DWT_H */
