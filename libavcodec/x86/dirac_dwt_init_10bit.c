/*
 * x86 optimized discrete wavelet transform
 * Copyright (c) 2018 James Darnley
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

#include "libavutil/x86/asm.h"
#include "libavutil/x86/cpu.h"
#include "libavcodec/dirac_dwt.h"

void ff_legall53_vertical_lo_sse2(int32_t *b0, int32_t *b1, int32_t *b2, int width);

void ff_horizontal_compose_haar_10bit_sse2(int32_t *b0, int32_t *b1, int width_align);
void ff_horizontal_compose_haar_10bit_avx(int32_t *b0, int32_t *b1, int width_align);

void ff_vertical_compose_haar_10bit_sse2(int32_t *b0, int32_t *b1, int width_align);
void ff_vertical_compose_haar_10bit_avx(int32_t *b0, int32_t *b1, int width_align);

static void vertical_compose_haar_sse2(int32_t *b0, int32_t *b1, int width)
{
    int i, width_align = width & ~3;
    ff_vertical_compose_haar_10bit_sse2(b0, b1, width_align);
    for(i=width_align; i<width; i++) {
        b0[i] = COMPOSE_HAARiL0(b0[i], b1[i]);
        b1[i] = COMPOSE_HAARiH0(b1[i], b0[i]);
    }
}

static void vertical_compose_haar_avx(int32_t *b0, int32_t *b1, int width)
{
    int i, width_align = width & ~3;
    ff_vertical_compose_haar_10bit_avx(b0, b1, width_align);
    for(i=width_align; i<width; i++) {
        b0[i] = COMPOSE_HAARiL0(b0[i], b1[i]);
        b1[i] = COMPOSE_HAARiH0(b1[i], b0[i]);
    }
}

static void horizontal_compose_haar_sse2(int32_t *b, int32_t *tmp, int width)
{
    int i = width/2 & ~3;
    ff_horizontal_compose_haar_10bit_sse2(b, tmp, width);
    for (; i < width/2; i++) {
        b[2*i  ] = (tmp[i] + 1) >> 1;
        b[2*i+1] = (COMPOSE_HAARiH0(b[i + width/2], tmp[i]) + 1) >> 1;
    }
}

static void horizontal_compose_haar_avx(int32_t *b, int32_t *tmp, int width)
{
    int i = width/2 & ~3;
    ff_horizontal_compose_haar_10bit_avx(b, tmp, width);
    for (; i < width/2; i++) {
        b[2*i  ] = (tmp[i] + 1) >> 1;
        b[2*i+1] = (COMPOSE_HAARiH0(b[i + width/2], tmp[i]) + 1) >> 1;
    }
}

static void legall53_vertical_lo_sse2(int32_t *b0, int32_t *b1, int32_t *b2, int width)
{
    int i = width & ~3;
    ff_legall53_vertical_lo_sse2(b0, b1, b2, i);
    for(; i<width; i++)
        b1[i] = COMPOSE_53iL0(b0[i], b1[i], b2[i]);
}

av_cold void ff_spatial_idwt_init_10bit_x86(DWTContext *d, enum dwt_type type)
{
#if HAVE_X86ASM
    int cpu_flags = av_get_cpu_flags();

    if (EXTERNAL_SSE2(cpu_flags)) {
        switch (type) {
            case DWT_DIRAC_LEGALL5_3:
                d->vertical_compose_l0 = (void*)legall53_vertical_lo_sse2;
                break;
            case DWT_DIRAC_HAAR0:
                d->vertical_compose = (void*)vertical_compose_haar_sse2;
                break;
            case DWT_DIRAC_HAAR1:
                d->horizontal_compose = (void*)horizontal_compose_haar_sse2;
                d->vertical_compose = (void*)vertical_compose_haar_sse2;
                break;
        }
    }

    if (EXTERNAL_AVX(cpu_flags)) {
        switch (type) {
            case DWT_DIRAC_HAAR0:
                d->vertical_compose = (void*)vertical_compose_haar_avx;
                break;
            case DWT_DIRAC_HAAR1:
                d->horizontal_compose = (void*)horizontal_compose_haar_avx;
                d->vertical_compose = (void*)vertical_compose_haar_avx;
                break;
        }
    }

#endif // HAVE_X86ASM
}
