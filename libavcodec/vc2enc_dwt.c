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
#include "vc2enc_dwt.h"

static void vc2_subband_dwt_97(dwtcoef *synth, dwtcoef *data,
                               ptrdiff_t stride, int width, int height,
                               const ptrdiff_t hstride)
{
    int x, y;
    dwtcoef *datal = data, *synthl = synth;
    const ptrdiff_t synth_width  = width  << 1;
    const ptrdiff_t synth_height = height << 1;

    /*
     * Shift in one bit that is used for additional precision and copy
     * the data to the buffer.
     */
    for (y = 0; y < synth_height; y++) {
        for (x = 0; x < synth_width; x++)
            synthl[x] = datal[x] << 1;
        synthl += synth_width;
        datal += stride;
    }

    /* Horizontal synthesis. */
    synthl = synth;
    for (y = 0; y < synth_height; y++) {
        /* Lifting stage 2. */
        synthl[1] -= (8*synthl[0] + 9*synthl[2] - synthl[4] + 8) >> 4;
        for (x = 1; x < width - 2; x++)
            synthl[2*x + 1] -= (9*synthl[2*x] + 9*synthl[2*x + 2] - synthl[2*x + 4] -
                                synthl[2 * x - 2] + 8) >> 4;
        synthl[synth_width - 1] -= (17*synthl[synth_width - 2] -
                                    synthl[synth_width - 4] + 8) >> 4;
        synthl[synth_width - 3] -= (8*synthl[synth_width - 2] +
                                    9*synthl[synth_width - 4] -
                                    synthl[synth_width - 6] + 8) >> 4;
        /* Lifting stage 1. */
        synthl[0] += (synthl[1] + synthl[1] + 2) >> 2;
        for (x = 1; x < width - 1; x++)
            synthl[2*x] += (synthl[2*x - 1] + synthl[2*x + 1] + 2) >> 2;

        synthl[synth_width - 2] += (synthl[synth_width - 3] +
                                    synthl[synth_width - 1] + 2) >> 2;
        synthl += synth_width;
    }

    /* Vertical synthesis: Lifting stage 2. */
    synthl = synth + synth_width;
    for (x = 0; x < synth_width; x++)
        synthl[x] -= (8*synthl[x - synth_width] + 9*synthl[x + synth_width] -
                      synthl[x + 3 * synth_width] + 8) >> 4;

    synthl = synth + (synth_width << 1);
    for (y = 1; y < height - 2; y++) {
        for (x = 0; x < synth_width; x++)
            synthl[x + synth_width] -= (9*synthl[x] +
                                        9*synthl[x + 2 * synth_width] -
                                        synthl[x - 2 * synth_width] -
                                        synthl[x + 4 * synth_width] + 8) >> 4;
        synthl += synth_width << 1;
    }

    synthl = synth + (synth_height - 1) * synth_width;
    for (x = 0; x < synth_width; x++) {
        synthl[x] -= (17*synthl[x - synth_width] -
                      synthl[x - 3*synth_width] + 8) >> 4;
                      synthl[x - 2*synth_width] -= (9*synthl[x - 3*synth_width] +
                      8*synthl[x - 1*synth_width] - synthl[x - 5*synth_width] + 8) >> 4;
    }

    /* Vertical synthesis: Lifting stage 1. */
    synthl = synth;
    for (x = 0; x < synth_width; x++)
        synthl[x] += (synthl[x + synth_width] + synthl[x + synth_width] + 2) >> 2;

    synthl = synth + (synth_width << 1);
    for (y = 1; y < height - 1; y++) {
        for (x = 0; x < synth_width; x++)
            synthl[x] += (synthl[x - synth_width] + synthl[x + synth_width] + 2) >> 2;
        synthl += synth_width << 1;
    }

    synthl = synth + (synth_height - 2) * synth_width;
    for (x = 0; x < synth_width; x++)
        synthl[x] += (synthl[x - synth_width] + synthl[x + synth_width] + 2) >> 2;

    //deinterleave(data, stride, width, height, synth);
}

static void vc2_subband_dwt_53(dwtcoef *synth, dwtcoef *data,
                               ptrdiff_t stride, int width, int height,
                               const ptrdiff_t hstride)
{
    int x, y;
    dwtcoef *synthl = synth, *datal = data;
    const ptrdiff_t synth_width  = width  << 1;
    const ptrdiff_t synth_height = height << 1;

    /*
     * Shift in one bit that is used for additional precision and copy
     * the data to the buffer.
     */
    for (y = 0; y < synth_height; y++) {
        for (x = 0; x < synth_width; x++)
            synthl[x] = datal[x] << 1;
        synthl += synth_width;
        datal  += stride;
    }

    /* Horizontal synthesis. */
    synthl = synth;
    for (y = 0; y < synth_height; y++) {
        /* Lifting stage 2. */
        for (x = 0; x < width - 1; x++)
            synthl[2 * x + 1] -= (synthl[2 * x] + synthl[2 * x + 2] + 1) >> 1;

        synthl[synth_width - 1] -= (2*synthl[synth_width - 2] + 1) >> 1;

        /* Lifting stage 1. */
        synthl[0] += (2*synthl[1] + 2) >> 2;
        for (x = 1; x < width - 1; x++)
            synthl[2 * x] += (synthl[2 * x - 1] + synthl[2 * x + 1] + 2) >> 2;

        synthl[synth_width - 2] += (synthl[synth_width - 3] + synthl[synth_width - 1] + 2) >> 2;

        synthl += synth_width;
    }

    /* Vertical synthesis: Lifting stage 2. */
    synthl = synth + synth_width;
    for (x = 0; x < synth_width; x++)
        synthl[x] -= (synthl[x - synth_width] + synthl[x + synth_width] + 1) >> 1;

    synthl = synth + (synth_width << 1);
    for (y = 1; y < height - 1; y++) {
        for (x = 0; x < synth_width; x++)
            synthl[x + synth_width] -= (synthl[x] + synthl[x + synth_width * 2] + 1) >> 1;
        synthl += (synth_width << 1);
    }

    synthl = synth + (synth_height - 1) * synth_width;
    for (x = 0; x < synth_width; x++)
        synthl[x] -= (2*synthl[x - synth_width] + 1) >> 1;

    /* Vertical synthesis: Lifting stage 1. */
    synthl = synth;
    for (x = 0; x < synth_width; x++)
        synthl[x] += (2*synthl[synth_width + x] + 2) >> 2;

    synthl = synth + (synth_width << 1);
    for (y = 1; y < height - 1; y++) {
        for (x = 0; x < synth_width; x++)
            synthl[x] += (synthl[x + synth_width] + synthl[x - synth_width] + 2) >> 2;
        synthl += (synth_width << 1);
    }

    synthl = synth + (synth_height - 2)*synth_width;
    for (x = 0; x < synth_width; x++)
        synthl[x] += (synthl[x - synth_width] + synthl[x + synth_width] + 2) >> 2;


    //deinterleave(data, stride, width, height, synth);
}

static av_always_inline void dwt_haar(dwtcoef *data,
                                      ptrdiff_t stride, int width, int height,
                                      const ptrdiff_t hstride,
                                      const int s)
{
    int x, y;
    const ptrdiff_t synth_width  = width  << 1;
    const ptrdiff_t synth_height = height << 1;

    /* Horizontal synthesis. */
    for (y = 0; y < synth_height; y++) {
        for (x = 0; x < synth_width; x += 2) {
            data[y*stride + (x+1)*hstride] = (data[y*stride + (x+1)*hstride] << s) -
                                             (data[y*stride + x*hstride] << s);
            data[y*stride + x*hstride] = (data[y*stride + x*hstride] << s) +
                                        ((data[y*stride + (x+1)*hstride] + 1) >> 1);
        }
    }

    /* Vertical synthesis. */
    for (x = 0; x < synth_width; x++) {
        for (y = 0; y < synth_height; y += 2) {
            data[(y + 1)*stride + x*hstride] = data[(y + 1)*stride + x*hstride] -
                                               data[y*stride + x*hstride];
            data[y*stride + x*hstride] = data[y*stride + x*hstride] +
                                       ((data[(y + 1)*stride + x*hstride] + 1) >> 1);
        }
    }
}

static void vc2_subband_dwt_haar(dwtcoef *synth, dwtcoef *data,
                                 ptrdiff_t stride, int width, int height,
                                 const ptrdiff_t hstride)
{
    dwt_haar(data, stride, width, height, hstride, 0);
}

static void vc2_subband_dwt_haar_shift(dwtcoef *synth, dwtcoef *data,
                                       ptrdiff_t stride, int width, int height,
                                       const ptrdiff_t hstride)
{
    dwt_haar(data, stride, width, height, hstride, 1);
}

av_cold int ff_vc2enc_init_transforms(VC2TransformContext *s, int p_stride,
                                      int p_height, int slice_w, int slice_h)
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

av_cold void ff_vc2enc_free_transforms(VC2TransformContext *s)
{
    av_free(s->buffer - s->padding);
    s->buffer = NULL;
}
